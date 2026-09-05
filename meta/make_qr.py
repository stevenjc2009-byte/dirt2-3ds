"""Generate the install QR codes FBI's "Scan QR Code" reads.

Two are written per release, following the same pattern as raytracer3ds:

    meta/qr-latest.png    -> /releases/latest/download/dirt2.cia
    meta/qr-<version>.png -> /releases/download/<version>/dirt2.cia

The "latest" one is deliberately built on GitHub's `/releases/latest/download/`
form rather than a pinned tag, so the README's QR keeps working after the next
release without being regenerated. The pinned one exists so an older version can
still be installed on purpose -- a QR in a README is otherwise a moving target.

Self-locating: paths derive from __file__, so this works from any working
directory.

    python meta/make_qr.py v1.0.0
"""

import sys
from pathlib import Path

import qrcode

ROOT = Path(__file__).resolve().parent.parent
META = ROOT / "meta"

OWNER_REPO = "stevenjc2009-byte/dirt2-3ds"
ASSET = "dirt2.cia"

version = sys.argv[1] if len(sys.argv) > 1 else "v1.0.0"

targets = {
    META / "qr-latest.png":
        f"https://github.com/{OWNER_REPO}/releases/latest/download/{ASSET}",
    META / f"qr-{version}.png":
        f"https://github.com/{OWNER_REPO}/releases/download/{version}/{ASSET}",
}

for path, url in targets.items():
    # box_size 8 keeps the modules large enough for the 3DS's camera, which is
    # low resolution and focuses poorly at close range; border 4 is the spec
    # minimum quiet zone and scanners fail without it.
    qr = qrcode.QRCode(box_size=8, border=4)
    qr.add_data(url)
    qr.make(fit=True)
    qr.make_image(fill_color="black", back_color="white").save(path)
    print(f"{path.name}: {url}")
