/*---------------------------------------------------------------------------------
 * qr.c -- implementation of the qr.h wrapper. See qr.h for the API contract,
 * the version-cap/character-budget reasoning, and why this is a byte-grid
 * rather than a bitfield.
 *
 * UPSTREAM LIBRARY.
 *   Wraps source/ui/qrcodegen.c / qrcodegen.h, vendored verbatim (see the
 *   vendoring note at the top of each of those files for the exact commit).
 *   That library does the actual Reed-Solomon encoding, mask selection and
 *   module placement; this file only adapts its bit-packed output to a
 *   plain byte-per-module grid and enforces the fixed version cap.
 *
 * CAPACITY NUMBERS WERE MEASURED, NOT RECALLED.
 *   The 152-byte-mode-character figure for version 8 at ECC MEDIUM (quoted
 *   in qr.h) was obtained by actually calling qrcodegen_encodeText() with
 *   maxVersion=8, ecl=MEDIUM, boostEcl=true against strings of increasing
 *   length built from lowercase letters (which force byte mode, same as a
 *   real GitHub URL) and finding where it starts returning false, rather
 *   than trusted from memory -- a subtly wrong recollection of a QR
 *   capacity table is exactly the kind of "looks plausible, is wrong"
 *   mistake this module exists to avoid. See the scratchpad capacity probe
 *   used during development for the exact numbers across versions 5-12.
 *---------------------------------------------------------------------------------*/
#include "ui/qr.h"
#include "ui/qrcodegen.h"

#include <string.h>

bool qr_encode_url(QrCode *out, const char *url) {
    if (out == NULL) {
        return false;
    }

    out->size = 0;
    memset(out->modules, 0, sizeof(out->modules));

    if (url == NULL || url[0] == '\0') {
        return false;
    }

    /* Fixed-size scratch buffers, sized by a compile-time constant (NOT a
     * VLA) -- qrcodegen needs one buffer to build the bitstream in and a
     * second to hold the finished QR code before qrcodegen_getModule() can
     * be queried. Neither escapes this function; nothing here is malloc'd
     * or static. */
    uint8_t tempBuffer[qrcodegen_BUFFER_LEN_FOR_VERSION(QR_VERSION_CAP)];
    uint8_t qrcode[qrcodegen_BUFFER_LEN_FOR_VERSION(QR_VERSION_CAP)];

    bool ok = qrcodegen_encodeText(
        url,
        tempBuffer,
        qrcode,
        qrcodegen_Ecc_MEDIUM,
        qrcodegen_VERSION_MIN,
        QR_VERSION_CAP,
        qrcodegen_Mask_AUTO,
        true /* boostEcl: use a higher ECC level than MEDIUM for free if the
                chosen version has room to spare -- never reduces capacity,
                only improves how much camera-scan damage the code tolerates */);

    if (!ok) {
        /* url is NULL/empty (already handled above) or too long to fit any
         * version up to QR_VERSION_CAP at ECC MEDIUM -- out->size stays 0. */
        return false;
    }

    int size = qrcodegen_getSize(qrcode);
    out->size = size;

    for (int y = 0; y < size; y++) {
        for (int x = 0; x < size; x++) {
            out->modules[y * QR_MAX_MODULES + x] =
                qrcodegen_getModule(qrcode, x, y) ? 1 : 0;
        }
    }

    return true;
}
