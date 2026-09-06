/*---------------------------------------------------------------------------------
 * update.c -- see update.h for the module contract, why the check is
 * blocking, and why the web redirect URL is used instead of the REST API.
 *---------------------------------------------------------------------------------*/
#include "update/update.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "net/http.h"
#include "version.h"

/* This project's own repo. A literal here, not a build-time define, for
 * the same reason DIRT2_VERSION lives in exactly one header (version.h):
 * one obvious place to look, not a define threaded through the build
 * system for a string that changes approximately never. */
#define DIRT2_RELEASES_LATEST_URL \
    "https://github.com/stevenjc2009-byte/dirt2-3ds/releases/latest"

/* The literal GitHub puts between a release's owning repo and its tag in
 * every "releases/tag/..." URL this app will ever be redirected to. Used
 * as a fixed anchor to search for below rather than trying to parse the
 * URL's path segments generically -- the shape is exactly this one string
 * on every real GitHub release URL, so anchoring on it directly is simpler
 * and no less correct than a general path splitter would be. */
#define TAG_URL_MARKER "/tag/"

/* Hard cap on the number of dot-separated segments compared, purely to
 * make the loop below provably finite against adversarial/malformed input
 * (e.g. a string of garbage with no digits or dots at all) without relying
 * on reasoning about every possible input shape. A real version string
 * never comes close to this many segments; it has no other significance. */
#define VERSION_COMPARE_MAX_SEGMENTS 8

int update_version_compare(const char *a, const char *b) {
    if (!a) a = "";
    if (!b) b = "";

    const char *pa = a;
    const char *pb = b;

    /* Walk both strings in lockstep, one dot-separated segment at a time,
     * until BOTH are exhausted -- not just one, so "1.2" vs "1.2.1"
     * correctly treats the missing third segment of "1.2" as 0 rather than
     * stopping the compare early and calling them equal. */
    for (int segment = 0; segment < VERSION_COMPARE_MAX_SEGMENTS; segment++) {
        if (*pa == '\0' && *pb == '\0') return 0;

        char *next_a = NULL;
        char *next_b = NULL;
        /* strtoul on a non-digit or empty segment (e.g. two dots in a
         * row, or a stray letter) leaves next_* == the segment's start and
         * returns 0 -- exactly the "malformed segment parses as 0" rule
         * update.h documents, for free, with no special-casing needed
         * here. */
        unsigned long va = strtoul(pa, &next_a, 10);
        unsigned long vb = strtoul(pb, &next_b, 10);

        if (va != vb) return (va < vb) ? -1 : 1;

        pa = next_a;
        pb = next_b;

        /* Consume exactly one separator character past the digits just
         * parsed: a '.' if one is there (the normal case), otherwise a
         * single character of anything else that isn't the end of the
         * string. That second case is what makes this loop provably
         * terminate on malformed input like "1.2xyz" -- 'x' is neither a
         * digit strtoul() will consume nor a '.', so without this it would
         * never advance and the loop above would re-parse the same
         * "xyz..." segment as 0 forever. A string already at '\0' is left
         * untouched -- there is nothing left to skip past. */
        if (*pa == '.') pa++; else if (*pa != '\0') pa++;
        if (*pb == '.') pb++; else if (*pb != '\0') pb++;
    }

    return 0; /* segment budget exhausted without a difference -- treat as equal */
}

bool update_parse_tag_from_url(const char *url, char *out, size_t out_size) {
    if (!url || !out || out_size == 0) return false;

    const char *marker = strstr(url, TAG_URL_MARKER);
    if (!marker) return false;

    const char *tag_start = marker + strlen(TAG_URL_MARKER);
    if (*tag_start == '\0') return false; /* "/tag/" right at the end of the URL */

    /* The tag runs up to the next '/' (a trailing slash after the tag) or
     * the end of the string. */
    const char *tag_end = strchr(tag_start, '/');
    size_t tag_len = tag_end ? (size_t)(tag_end - tag_start) : strlen(tag_start);
    if (tag_len == 0) return false; /* ".../tag//..." or ".../tag/" with a bare trailing slash */

    /* Strip exactly one leading 'v'/'V' -- GitHub's own tag convention for
     * this project ("v1.0.2"), but tolerate a tag published without it. */
    if ((tag_start[0] == 'v' || tag_start[0] == 'V') && tag_len > 1) {
        tag_start++;
        tag_len--;
    }

    if (tag_len >= out_size) return false; /* would truncate -- report no tag rather than a wrong one */

    memcpy(out, tag_start, tag_len);
    out[tag_len] = '\0';
    return true;
}

void update_check(UpdateResult *out) {
    if (!out) return;
    memset(out, 0, sizeof(*out));
    snprintf(out->current_version, sizeof(out->current_version), "%s", DIRT2_VERSION);

    if (!http_net_init()) {
        out->status = UPDATE_ERR_INIT;
        snprintf(out->detail, sizeof(out->detail),
                 "Could not start networking (soc/romfs/cert bundle)");
        return;
    }

    HttpResponse resp;
    int curl_code = http_get(DIRT2_RELEASES_LATEST_URL, &resp);
    out->curl_code   = curl_code;
    out->http_status = resp.http_status;

    if (curl_code != 0) {
        out->status = UPDATE_ERR_NET;
        snprintf(out->detail, sizeof(out->detail), "curl %d: %.60s",
                 curl_code, http_strerror(curl_code));
        http_net_exit();
        return;
    }

    if (resp.http_status < 200 || resp.http_status >= 400) {
        out->status = UPDATE_ERR_HTTP;
        snprintf(out->detail, sizeof(out->detail), "GitHub answered with HTTP %ld",
                 resp.http_status);
        http_net_exit();
        return;
    }

    char tag[32];
    if (!update_parse_tag_from_url(resp.effective_url, tag, sizeof(tag))) {
        out->status = UPDATE_ERR_PARSE;
        snprintf(out->detail, sizeof(out->detail), "No version tag in: %.70s",
                 resp.effective_url);
        http_net_exit();
        return;
    }

    http_net_exit();

    /* Explicit bounds checks rather than letting snprintf silently
     * truncate: a chopped tag or URL is silently WRONG data (e.g. a real
     * "1.0.23" arriving in out->latest_version as "1.0.2" would misreport
     * both the version shown to the player and update_version_compare()'s
     * answer), and this project's rule is to fail loudly at a boundary
     * rather than hand back a value that looks fine but isn't. Both
     * ceilings (tag[32] here, HTTP_EFFECTIVE_URL_MAX in http.h) are already
     * far larger than anything a real GitHub URL for this repo produces,
     * so hitting either in practice would itself be a sign something odd
     * is being served -- worth surfacing via UPDATE_ERR_PARSE, not hiding. */
    if (strlen(tag) >= sizeof(out->latest_version)) {
        out->status = UPDATE_ERR_PARSE;
        snprintf(out->detail, sizeof(out->detail),
                 "Version tag too long to report (%zu bytes)", strlen(tag));
        return;
    }
    if (strlen(resp.effective_url) >= sizeof(out->release_url)) {
        out->status = UPDATE_ERR_PARSE;
        snprintf(out->detail, sizeof(out->detail),
                 "Release URL too long to report (%zu bytes)", strlen(resp.effective_url));
        return;
    }

    memcpy(out->latest_version, tag, strlen(tag) + 1);
    memcpy(out->release_url, resp.effective_url, strlen(resp.effective_url) + 1);
    out->update_available = update_version_compare(tag, DIRT2_VERSION) > 0;
    out->status = UPDATE_OK;
}
