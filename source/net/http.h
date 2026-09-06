/*---------------------------------------------------------------------------------
 * http.h -- a small HTTPS GET wrapper over libcurl (3ds-curl + 3ds-mbedtls).
 *
 * WHY LIBCURL AND NOT libctru's httpc/sslc.
 *   The 3DS's own ssl:C system service tops out at TLS 1.1. GitHub requires
 *   TLS 1.2+, and this has been MEASURED on real hardware by the sibling
 *   project skywave (a TLS "protocol version" alert from github.com,
 *   api.github.com and both githubusercontent.com asset hosts).
 *   SSLCOPT_DisableVerify does not help -- it turns off certificate
 *   checking, not protocol negotiation, and the console still cannot speak
 *   TLS 1.2 to ssl:C at all. libcurl built against 3ds-mbedtls runs its own
 *   TLS stack over raw soc:U sockets and never touches ssl:C, which is the
 *   same route Universal-Updater and FBI ship. See update.h for the single
 *   caller this module exists for.
 *
 * OWNERSHIP / LIFECYCLE CONTRACT.
 *   http_net_init() brings up three global, 3DS-wide resources this module
 *   needs and nothing else in this codebase currently touches (soc:U,
 *   libcurl's global state, and -- see below -- a romfs mount for the CA
 *   bundle). http_net_exit() must be the one that tears them back down, and
 *   it is the caller's job to make sure no http_get() call is still in
 *   flight when it does; update_check() (update.h) is written as a single
 *   blocking init -> get -> exit sequence specifically so that is trivially
 *   true and there is never an overlap to reason about.
 *
 *   A second http_net_init() call while already up is NOT an error: it is a
 *   deliberate no-op that returns true without touching socInit, the soc
 *   buffer, or the cert bundle again. The alternative (re-running the whole
 *   sequence) is exactly how a double socInit or a leaked soc buffer would
 *   happen, so this guards the whole class rather than trusting every call
 *   site to only ever call it once.
 *
 * ROMFS.
 *   As of this writing nothing else in this codebase mounts romfs (see
 *   source/model/model.h's own integration note, item 1: "no romfs mount
 *   exists anywhere in this codebase yet"). The CA bundle this module needs
 *   (romfs/cacert.pem) has nowhere else to come from, so http_net_init()
 *   mounts romfs itself and only unmounts it in http_net_exit() if THIS
 *   call was the one that actually mounted it (tracked internally). That
 *   guard matters: if a future main.c also calls romfsInit() for model
 *   assets, this module must never be the one that yanks romfs out from
 *   under it on exit. When that day comes, the cleaner fix is for main.c to
 *   own the one romfs mount for the whole app and for this module to stop
 *   calling romfsInit()/romfsExit() at all -- flagged here rather than done
 *   now, since it is a cross-module lifecycle decision outside this file's
 *   owned scope.
 *
 * WHAT THIS IS NOT.
 *   This is a HEAD-shaped probe, not a general body-fetching client: it
 *   sends CURLOPT_NOBODY so the only thing the transfer costs is DNS +
 *   TLS handshake + response headers, no page body. Verified directly
 *   against the real repo (see the update.h test notes) that GitHub's
 *   "/releases/latest" redirect answers a HEAD request with the identical
 *   302 -> tag URL and HTTP status a GET gets, so this loses nothing for
 *   update_check()'s purposes. Nothing else in this codebase needs an HTTP
 *   body yet, so a body-capturing variant is left unbuilt rather than
 *   added speculatively -- add one only when a real caller needs it.
 *---------------------------------------------------------------------------------*/
#ifndef DIRT2_NET_HTTP_H
#define DIRT2_NET_HTTP_H

#include <stdbool.h>

/* Long enough for any realistic GitHub redirect target (owner/repo/tag are
 * all bounded, GitHub's own URLs are nowhere close to this) with generous
 * headroom left over -- truncation here is silently wrong data (a chopped
 * tag), so this is sized to never need to truncate rather than sized
 * tightly and truncated defensively. */
#define HTTP_EFFECTIVE_URL_MAX 512

typedef struct {
    /* HTTP status of the FINAL request in the redirect chain (the one
     * effective_url names), 0 if the transfer never got a response at all. */
    long http_status;

    /* The URL curl actually ended up at after following every redirect,
     * capped at HTTP_MAX_REDIRECTS hops (see http.c). This is the whole
     * point of this wrapper for update_check()'s purposes: GitHub's
     * "/releases/latest" 302s to "/releases/tag/vX.Y.Z", and the tag lives
     * only in this final URL, nowhere in the (unfetched) response body.
     * Left as an empty string if the transfer failed before curl had an
     * effective URL to report. */
    char effective_url[HTTP_EFFECTIVE_URL_MAX];
} HttpResponse;

/* Returned by http_get() when it cannot even attempt the request: called
 * before a successful http_net_init() (or after http_net_exit()), or
 * curl_easy_init() itself failed (an allocation failure, in practice never
 * seen but checked anyway per this project's "handle errors explicitly"
 * rule). Deliberately negative and outside the range libcurl's own CURLcode
 * enum uses (every real CURLcode is >= 0), so a caller checking "was this a
 * real curl error I can look up with http_strerror()" can tell the two
 * apart with `code >= 0`. */
#define HTTP_ERR_NOT_INITIALIZED (-1)

/* Brings up soc:U, libcurl's global state, and the romfs-backed CA bundle
 * this module verifies every connection against. Must be called once
 * before the first http_get(), paired with exactly one http_net_exit().
 * Safe to call again while already up (see the file header) -- returns
 * true immediately without re-touching anything.
 *
 * Returns false if any step fails; everything that step and the ones
 * before it brought up is rolled back before returning, so a failed
 * http_net_init() never leaves a partial resource behind for
 * http_net_exit() to mishandle. */
bool http_net_init(void);

/* Tears down whatever http_net_init() brought up, in reverse order. A
 * no-op if http_net_init() was never called or already failed -- calling
 * this defensively "just in case" is always safe. */
void http_net_exit(void);

/* Blocking HEAD-shaped HTTPS GET of `url` (see the file header for why no
 * body). Requires http_net_init() to have already succeeded.
 *
 * Fills `out` regardless of outcome (zeroed first, so a failed call never
 * leaves stale data in it). Returns 0 (matches libcurl's own CURLE_OK) on a
 * completed transfer -- note this does NOT mean HTTP 2xx, only that a
 * response was received; check out->http_status for that. On a transport
 * failure, returns the underlying CURLcode as a positive int so the caller
 * can show a real number (see http_strerror()) instead of a bare "failed".
 * Returns HTTP_ERR_NOT_INITIALIZED if http_net_init() has not (yet, or any
 * longer) succeeded, or if `out` is NULL. */
int http_get(const char *url, HttpResponse *out);

/* Human-readable text for a curl_code returned by http_get(), e.g. "SSL
 * peer certificate or SSH remote key was not OK" for CURLE_PEER_FAILED_
 * VERIFICATION. Wraps curl_easy_strerror() so callers outside this module
 * (update.c in particular) never need their own '#include <curl/curl.h>' --
 * libcurl stays a private implementation detail of this one file. Returns
 * a fixed string, never NULL, for HTTP_ERR_NOT_INITIALIZED and any other
 * code libcurl itself would not recognise. */
const char *http_strerror(int curl_code);

#endif /* DIRT2_NET_HTTP_H */
