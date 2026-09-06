/*---------------------------------------------------------------------------------
 * update.h -- "is a newer release published" check against this project's
 * own GitHub repo. Scope is deliberately narrow: CHECK AND REPORT ONLY.
 * No download, no install -- that is a separate, later feature if steve
 * asks for one, not implied by this one.
 *
 * BLOCKING IS DELIBERATE.
 *   update_check() runs the whole HTTPS round trip on the calling thread
 *   and does not return until it is done (up to HTTP_CONNECT_TIMEOUT_S +
 *   HTTP_TOTAL_TIMEOUT_S worst case, see http.c). This is intentional, not
 *   a shortcut: the only caller this exists for is a pause-menu "Checking
 *   for updates..." screen that has already drawn its "Checking..." frame
 *   before calling this, so a multi-second stall reads as normal loading
 *   behaviour, not a hang. A background-thread version is real complexity
 *   (thread lifetime, a way to poll or be notified of completion, making
 *   sure http_net_init()/http_net_exit() cannot race a second check) that
 *   this feature's current scope does not need yet. Revisit if a caller
 *   ever needs the game loop to keep running during the check.
 *
 * WHY THE WEB URL, NOT api.github.com.
 *   https://github.com/<owner>/<repo>/releases/latest 302-redirects to
 *   .../releases/tag/vX.Y.Z and is served by the website, not the REST
 *   API, so it does NOT spend any of GitHub's unauthenticated API quota
 *   (60 requests/hour, shared per source IP -- easy to exhaust from a
 *   household NAT with more than one device checking). The version tag is
 *   read out of the final redirected-to URL; no JSON parsing needed at
 *   all, which is also why this module has no JSON dependency.
 *---------------------------------------------------------------------------------*/
#ifndef DIRT2_UPDATE_UPDATE_H
#define DIRT2_UPDATE_UPDATE_H

#include <stdbool.h>
#include <stddef.h>

typedef enum {
    UPDATE_OK,        /* check completed; see update_available for the answer */
    UPDATE_ERR_INIT,  /* http_net_init() failed -- soc/romfs/cert bundle */
    UPDATE_ERR_NET,   /* the request itself failed (DNS, timeout, TLS, ...) */
    UPDATE_ERR_HTTP,  /* a response came back but its status was not usable */
    UPDATE_ERR_PARSE  /* got a response, but no version tag could be read out of it */
} UpdateStatus;

typedef struct {
    UpdateStatus status;
    bool update_available; /* only meaningful when status == UPDATE_OK */

    char current_version[16]; /* this build's own DIRT2_VERSION (version.h) */
    char latest_version[16];  /* the tag GitHub reports, only set on UPDATE_OK */
    char release_url[160];    /* the final effective URL, only set on UPDATE_OK */

    /* Always filled on any non-UPDATE_OK status with something a player
     * (or steve, reading a screenshot with no debugger attached) can act
     * on -- e.g. "curl 28: Timeout was reached" or "GitHub answered HTTP
     * 404". A generic "check failed" on a console with no console output
     * is not diagnosable from the field, so every failure path is required
     * to fill this with specifics rather than leaving it blank. */
    char detail[96];

    /* Raw diagnostic numbers behind `detail`, for a caller (or a future
     * on-screen debug overlay) that wants to show or log the exact code
     * rather than just its text. curl_code mirrors http_get()'s own return
     * value (0 == success, a positive CURLcode, or HTTP_ERR_NOT_INITIALIZED);
     * http_status is 0 whenever no response was ever received. */
    int  curl_code;
    long http_status;
} UpdateResult;

/* Runs the whole check (see BLOCKING IS DELIBERATE above) and fills `out`.
 * Always fully initializes `out`, including current_version, regardless of
 * how far the check got before failing. Safe to call repeatedly -- each
 * call owns its own http_net_init()/http_net_exit() pair internally, see
 * http.h for why that pairing is safe to repeat. */
void update_check(UpdateResult *out);

/* Dotted-numeric version compare, exposed specifically so it is testable
 * without any networking (see this project's host test in the scratchpad
 * dir, not part of this repo). A plain strcmp() is wrong here: it says
 * "1.0.10" < "1.0.9" because '1' < '9' lexicographically at the first
 * differing character, which is exactly backwards for a real release
 * ordering. This instead parses each dot-separated run of digits as an
 * unsigned integer and compares those numerically, segment by segment.
 *
 * Returns <0 if a<b, 0 if a==b, >0 if a>b (memcmp/strcmp-style). Missing
 * trailing segments compare as 0 (so "1.2" == "1.2.0"). A segment with no
 * digits in it (empty, or leading non-digit garbage) parses as 0 for that
 * segment -- this is a permissive best-effort compare for a value that
 * already came from a well-formed GitHub tag by the time it gets here, not
 * a validator; a NULL argument is treated the same as an empty string. */
int update_version_compare(const char *a, const char *b);

/* Extracts the release tag from a GitHub "release" URL shaped like
 * ".../releases/tag/vX.Y.Z" (with or without a trailing slash, and with or
 * without the leading 'v' -- both are stripped if present). Also exposed
 * for host testing, same reasoning as update_version_compare() above.
 *
 * Returns true and NUL-terminates `out` on success. Returns false, leaving
 * `out` untouched, if `url` has no recognisable "/tag/" segment, the
 * segment after it is empty, or the tag would not fit in `out_size` --
 * silently truncating a version string is worse than reporting no tag at
 * all, since a truncated "1.0" read from a real "1.0.23" would wrongly
 * claim no update is available. */
bool update_parse_tag_from_url(const char *url, char *out, size_t out_size);

#endif /* DIRT2_UPDATE_UPDATE_H */
