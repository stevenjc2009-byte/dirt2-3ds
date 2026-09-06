/*---------------------------------------------------------------------------------
 * qr.h -- thin project-specific wrapper around the vendored qrcodegen library,
 * for drawing a scannable QR code on the bottom screen that points at a
 * GitHub release URL so steve can grab a build with the console's own camera
 * and FBI.
 *
 * WHY A WRAPPER INSTEAD OF CALLING qrcodegen DIRECTLY FROM main.c/hud.c.
 *   qrcodegen's API is byte-buffer-and-bit-twiddly (qrcodegen_getModule()
 *   returns bool per (x,y), packed into a bitstream the caller must not poke
 *   at directly) and needs two scratch buffers sized off
 *   qrcodegen_BUFFER_LEN_FOR_VERSION(maxVersion). None of that is this
 *   project's business to get right at every call site. qr_encode_url()
 *   hides all of it behind one call and one plain byte-per-module grid that
 *   any renderer can walk with a trivial nested loop.
 *
 * VERSION CAP AND CHARACTER BUDGET.
 *   Capped at QR version 8 (49x49 modules) at error-correction level MEDIUM.
 *   Measured directly against the real qrcodegen library (see the capacity
 *   probe referenced in qr.c's header) rather than assumed from a table:
 *   version 8-M holds up to 152 bytes in byte mode (the mode GitHub release
 *   URLs force, since they contain lowercase letters, which are outside the
 *   QR alphanumeric charset). A release URL is asserted by the caller to
 *   stay under 120 characters
 *   (e.g. "https://github.com/stevenjc2009-byte/dirt2-3ds/releases/tag/v1.0.2",
 *   66 characters) so version 8 leaves 32 characters of headroom above that
 *   budget -- enough for a longer repo/tag name without needing a bigger,
 *   denser (harder to camera-scan) code. Going to version 9+ was rejected:
 *   it buys headroom this project's URLs will not use, at the cost of a
 *   bigger, harder-to-scan code on a small bottom screen.
 *
 * NO DYNAMIC ALLOCATION.
 *   qr_encode_url() uses only its own stack-local scratch buffers (sized by
 *   the qrcodegen_BUFFER_LEN_FOR_VERSION(QR_VERSION_CAP) compile-time
 *   constant, so this is a fixed-size array, not a VLA) plus the
 *   caller-supplied QrCode. No malloc, no static mutable state -- safe to
 *   call from anywhere, any number of times, on a console with no virtual
 *   memory to fall back on.
 *---------------------------------------------------------------------------------*/
#ifndef DIRT2_UI_QR_H
#define DIRT2_UI_QR_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* QR Model 2 version cap. Version N is (4*N + 17) modules per side; see the
 * character-budget derivation in this file's header comment and in qr.c. */
#define QR_VERSION_CAP   8

/* Modules per side at QR_VERSION_CAP: 4*8 + 17 = 49. */
#define QR_MAX_MODULES   49

/* One QR code, decoded into a plain grid. `size` is the actual side length
 * for THIS code (it can be smaller than QR_MAX_MODULES for a short URL --
 * qrcodegen always picks the smallest version that fits) and is 0 if
 * encoding failed, in which case `modules` is all zero and must not be
 * drawn.
 *
 * ONE BYTE PER MODULE, NOT A BITFIELD.
 *   At QR_MAX_MODULES=49 this is 49*49 = 2401 bytes. A packed bitfield would
 *   be ~301 bytes -- a real difference on a console with 128 MB, but not one
 *   that matters here: this struct is a handful of these at most (one QR
 *   code on screen at a time), not a hot per-frame array. A byte-per-module
 *   grid means the renderer indexes it with a plain `modules[y*size+x]` and
 *   there is no bit-shift/mask arithmetic to get wrong on the one piece of
 *   code steve actually needs to scan correctly. Wasteful but simple beats
 *   compact but buggy for a fixed-function display buffer like this. */
typedef struct QrCode {
    int size;                                       /* modules per side; 0 == encode failed */
    unsigned char modules[QR_MAX_MODULES * QR_MAX_MODULES]; /* 1 == dark module, 0 == light */
} QrCode;

/* Encodes `url` as a QR code (error-correction level MEDIUM, auto mask,
 * smallest version up to QR_VERSION_CAP that fits) into `out`.
 *
 * Returns false and sets out->size = 0 (modules left zeroed) for:
 *   - out == NULL (nothing to write into)
 *   - url == NULL
 *   - url is the empty string
 *   - url is too long to fit any version up to QR_VERSION_CAP at ECC MEDIUM
 * It never overflows `out` and never allocates. */
bool qr_encode_url(QrCode *out, const char *url);

#ifdef __cplusplus
}
#endif

#endif /* DIRT2_UI_QR_H */
