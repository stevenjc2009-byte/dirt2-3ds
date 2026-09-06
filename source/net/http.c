/*---------------------------------------------------------------------------------
 * http.c -- see http.h for the module contract, the TLS-1.1-ceiling reason
 * this exists at all, and the romfs/soc lifecycle rules this file follows.
 *
 * THE curl 77 (CURLE_SSL_CACERT_BADFILE) TRAP, AND WHY THIS FILE AVOIDS IT.
 *   The sibling project mc (Blocksmith) shipped this exact libcurl+mbedtls
 *   route first and hit CURLE_SSL_CACERT_BADFILE on real hardware with
 *   CURLOPT_CAINFO pointed at a "romfs:/cacert.pem" PATH. Their own
 *   investigation (source/app/updater.c, the block above
 *   applyCommonOptions()) narrowed it to: libcurl does not open a CAINFO
 *   path when the option is set, it opens it LAZILY inside
 *   curl_easy_perform(), during the handshake -- and curl 77 is the single
 *   code mbedtls's curl backend returns for THREE different underlying
 *   failures that all look identical from the outside: the file could not
 *   be opened, the ~325 KB allocation burst building 121 chained
 *   mbedtls_x509_crt nodes ran out of heap, or the bundle content itself is
 *   malformed. A bare "curl 77" cannot tell those apart.
 *
 *   This file sidesteps the whole "open a path lazily" failure class by
 *   never handing curl a path at all: the CA bundle is read into memory
 *   once, up front, in http_net_init() (see load_cert_bundle() below), and
 *   handed to curl as an in-memory CURLOPT_CAINFO_BLOB in http_get(). There
 *   is no romfs file access left at curl_easy_perform() time for a lazy
 *   open to fail. This matches the fix Blocksmith's own v1.8.18 landed
 *   after finding the same problem.
 *
 *   What this file does NOT rule out, because it cannot: the OOM-during-
 *   parse branch of that same curl-77 umbrella. mbedtls_x509_crt_parse()
 *   still has to walk and allocate all 121 certificates in romfs/cacert.pem
 *   (copied in from mc unmodified, per this project's asset rule) during
 *   curl's TLS handshake, regardless of whether the bytes arrived via a
 *   path or a blob -- mc's own measurement against mbedTLS 2.28.8 (the
 *   version this portlib ships) put that parse at a ~325 KB peak and
 *   ~324 KB retained for the life of the connection. This project has not
 *   independently reproduced that measurement and has not proven this
 *   module clear of it on real DiRT2 hardware; see update.h and this
 *   project's completion report for what remains unverified.
 *---------------------------------------------------------------------------------*/
#include "net/http.h"

#include <3ds.h>
#include <curl/curl.h>
#include <malloc.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "version.h"

/* 1 MiB, aligned to 0x1000 below via memalign(). This is the size every
 * devkitPro socket example (and both sibling projects that already ship
 * networking, mc and skywave) is written against; nothing about this
 * module's single blocking request needs that much buffer, but "smaller
 * almost certainly works" is not worth gambling on a codepath this project
 * has zero hours of hardware testing on yet. Revisit with a real
 * measurement if RAM ever gets tight. */
#define SOC_BUFFER_SIZE (0x100000)

/* GitHub's "/releases/latest" redirects exactly once in practice (verified
 * directly against this project's own repo -- see the test notes in
 * update.h). 5 is generous headroom for GitHub changing its redirect shape
 * later without this becoming a hang; it is a hard cap, not a tuned value. */
#define HTTP_MAX_REDIRECTS 5L

/* A console with no network (aeroplane mode, no AP in range) must not make
 * a pause-menu "Checking..." screen look frozen forever. Connect and total
 * are separate so a slow DNS/handshake fails fast on its own budget rather
 * than eating the whole allowance and leaving no time for the transfer. */
#define HTTP_CONNECT_TIMEOUT_S 5L
#define HTTP_TOTAL_TIMEOUT_S   10L

/* Where the CA bundle lives once installed into RomFs -- see this
 * project's romfs/cacert.pem (copied verbatim from mc's romfs/cacert.pem,
 * per this project's "use the exact supplied asset" rule; not regenerated
 * or trimmed here even though it is the same 121-certificate, ~185 KB
 * bundle mc's own OOM investigation is still open against -- see the file
 * header above and this task's completion report). */
#define CERT_BUNDLE_ROMFS "romfs:/cacert.pem"

/* Grown geometrically rather than sized with fseek(SEEK_END)/ftell() first,
 * for the same reason this project's model.c avoids trusting a romfs
 * devoptab's reported length for anything that matters: the only thing
 * that can be trusted is what fread() actually delivers. Starts comfortably
 * below the ~185 KB the shipped bundle needs (one growth step covers it)
 * and stops well short of a runaway read -- the bundle this project ships
 * is a known, bounded size, so anything past the ceiling below means
 * something other than a normal cacert.pem is being read, not that the
 * ceiling is too tight. */
#define CA_BUNDLE_INITIAL_CAP  (64 * 1024)
#define CA_BUNDLE_HARD_CEILING (512 * 1024)

static u32   *s_soc_buf        = NULL;
static bool   s_soc_up         = false;
static bool   s_romfs_owned    = false;
static bool   s_curl_global_up = false;
static bool   s_net_up         = false;

static unsigned char *s_ca_bundle     = NULL;
static size_t          s_ca_bundle_len = 0; /* includes the trailing NUL --
                                                see load_cert_bundle(). */

/* Reads CERT_BUNDLE_ROMFS whole into s_ca_bundle/s_ca_bundle_len. Called
 * exactly once, from http_net_init(), while its own romfsInit() a few
 * lines above (in the caller) is known to be in effect. Returns false and
 * leaves the two statics untouched on any failure -- the caller is
 * responsible for treating that as a full http_net_init() failure, since
 * there is no fallback source for these bytes in this module's scope. */
static bool load_cert_bundle(void) {
    size_t cap = CA_BUNDLE_INITIAL_CAP;
    unsigned char *buf = (unsigned char *)malloc(cap);
    if (!buf) return false;

    FILE *f = fopen(CERT_BUNDLE_ROMFS, "rb");
    if (!f) {
        free(buf);
        return false;
    }

    size_t total = 0;
    for (;;) {
        if (total == cap) {
            if (cap >= CA_BUNDLE_HARD_CEILING) {
                fclose(f);
                free(buf);
                return false;
            }
            size_t new_cap = cap * 2;
            if (new_cap > CA_BUNDLE_HARD_CEILING) new_cap = CA_BUNDLE_HARD_CEILING;
            unsigned char *grown = (unsigned char *)realloc(buf, new_cap);
            if (!grown) {
                fclose(f);
                free(buf);
                return false;
            }
            buf = grown;
            cap = new_cap;
        }

        size_t n = fread(buf + total, 1, cap - total, f);
        total += n;
        if (n == 0) break; /* EOF or a read error -- either way, nothing more is coming */
    }
    fclose(f);

    if (total == 0) {
        free(buf);
        return false;
    }

    /* Always in-bounds: the loop above only ever stops filling exactly to
     * `cap` by growing FIRST (see the `total == cap` check), so a normal
     * exit always leaves at least one spare byte past `total`. */
    buf[total] = '\0';

    s_ca_bundle = buf;
    /* mbedtls_x509_crt_parse()'s own documented PEM-vs-DER contract: PEM
     * input must be NUL-terminated AND buflen must count that terminator,
     * which is how it tells "this is PEM text" apart from "this is a raw
     * DER blob". +1 here is that contract, not an arbitrary pad. */
    s_ca_bundle_len = total + 1;
    return true;
}

bool http_net_init(void) {
    if (s_net_up) return true; /* idempotent -- see http.h */

    s_soc_buf = (u32 *)memalign(0x1000, SOC_BUFFER_SIZE);
    if (!s_soc_buf) return false;

    if (R_FAILED(socInit(s_soc_buf, SOC_BUFFER_SIZE))) {
        free(s_soc_buf);
        s_soc_buf = NULL;
        return false;
    }
    s_soc_up = true;

    /* See http.h's ROMFS section: this module owns the mount only if it is
     * the one that actually created it. If romfs is already mounted (a
     * future main.c, or a second init on this same run for some reason),
     * romfsInit() fails here and s_romfs_owned stays false, but the
     * fopen() inside load_cert_bundle() below still works against whatever
     * mount already exists. */
    s_romfs_owned = R_SUCCEEDED(romfsInit());

    if (!load_cert_bundle()) {
        if (s_romfs_owned) {
            romfsExit();
            s_romfs_owned = false;
        }
        socExit();
        s_soc_up = false;
        free(s_soc_buf);
        s_soc_buf = NULL;
        return false;
    }

    if (curl_global_init(CURL_GLOBAL_DEFAULT) != CURLE_OK) {
        free(s_ca_bundle);
        s_ca_bundle = NULL;
        s_ca_bundle_len = 0;
        if (s_romfs_owned) {
            romfsExit();
            s_romfs_owned = false;
        }
        socExit();
        s_soc_up = false;
        free(s_soc_buf);
        s_soc_buf = NULL;
        return false;
    }
    s_curl_global_up = true;

    s_net_up = true;
    return true;
}

void http_net_exit(void) {
    if (!s_net_up) return; /* covers "never initialized" and "already exited" */

    if (s_curl_global_up) {
        curl_global_cleanup();
        s_curl_global_up = false;
    }

    free(s_ca_bundle);
    s_ca_bundle = NULL;
    s_ca_bundle_len = 0;

    if (s_romfs_owned) {
        romfsExit();
        s_romfs_owned = false;
    }

    if (s_soc_up) {
        socExit();
        s_soc_up = false;
    }
    free(s_soc_buf);
    s_soc_buf = NULL;

    s_net_up = false;
}

int http_get(const char *url, HttpResponse *out) {
    if (!out) return HTTP_ERR_NOT_INITIALIZED;
    memset(out, 0, sizeof(*out));

    if (!s_net_up || !url) return HTTP_ERR_NOT_INITIALIZED;

    CURL *curl = curl_easy_init();
    if (!curl) return HTTP_ERR_NOT_INITIALIZED;

    curl_easy_setopt(curl, CURLOPT_URL, url);

    /* See http.h's "WHAT THIS IS NOT" section: a HEAD-shaped probe,
     * verified against the real repo to redirect identically to a GET. */
    curl_easy_setopt(curl, CURLOPT_NOBODY, 1L);

    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
    curl_easy_setopt(curl, CURLOPT_MAXREDIRS, HTTP_MAX_REDIRECTS);

    /* GitHub 403s any request with no User-Agent at all. */
    curl_easy_setopt(curl, CURLOPT_USERAGENT, "dirt2-3ds/" DIRT2_VERSION);

    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, HTTP_CONNECT_TIMEOUT_S);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, HTTP_TOTAL_TIMEOUT_S);

    /* In-memory CA bundle -- see this file's header comment for why this,
     * and never CURLOPT_CAINFO with a path, is used. NOCOPY is safe here
     * specifically because s_ca_bundle is freed nowhere but http_net_exit(),
     * and http.h's documented contract is that no http_get() call is still
     * in flight when that runs (update_check() is a single blocking
     * call), so this buffer always outlives the handle using it. */
    struct curl_blob ca_blob;
    ca_blob.data  = s_ca_bundle;
    ca_blob.len   = s_ca_bundle_len;
    ca_blob.flags = CURL_BLOB_NOCOPY;
    curl_easy_setopt(curl, CURLOPT_CAINFO_BLOB, &ca_blob);
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 1L);
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 2L);

    CURLcode result = curl_easy_perform(curl);

    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &out->http_status);

    char *effective = NULL;
    curl_easy_getinfo(curl, CURLINFO_EFFECTIVE_URL, &effective);
    if (effective) {
        /* snprintf, not a raw copy: `effective` points into curl's own
         * handle memory, and bounding the copy to out's fixed buffer here
         * is this project's boundary-validation rule, not a defence
         * against any specific thing curl is documented to do wrong. */
        snprintf(out->effective_url, sizeof(out->effective_url), "%s", effective);
    }

    curl_easy_cleanup(curl);
    return (int)result;
}

const char *http_strerror(int curl_code) {
    if (curl_code < 0) return "http.c: not initialized (see http_net_init)";
    return curl_easy_strerror((CURLcode)curl_code);
}
