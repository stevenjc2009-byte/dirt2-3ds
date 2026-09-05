"""Generate the PLACEHOLDER icon, banner and banner jingle the CIA build needs.

Copied in from raytracer3ds/meta/make_assets.py and adapted (copy-don't-depend:
nothing here points outside this project). Same reason it exists there --
`make cia` hard-fails without cia/banner.png, cia/banner.wav and an icon, and
those three files are the only thing standing between a working build and an
installable title.

THIS IS PLACEHOLDER ART. It is a flat wedge silhouette in the white/yellow of
the DiRT 2 menu card, not a real icon. Replace meta/icon.png and
meta/banner.png with something better and re-run `make cia`.

Self-locating: every path is derived from __file__, so the script works from
any working directory and moves with the project.

    python meta/make_assets.py

Writes, relative to the project root:
    icon.png         48x48, picked up by the Makefile's APP_ICON rule
    meta/icon.png    the same image, kept with the rest of the art
    meta/banner.png  256x128 source
    cia/banner.png   256x128, what bannertool consumes
    cia/banner.wav   0.5 s of silence, 16-bit mono 22050 Hz -- bannertool
                     requires a VALID wav, not an absent or empty one
"""

import struct
import wave
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parent.parent
META = ROOT / "meta"
CIA = ROOT / "cia"
CIA.mkdir(exist_ok=True)

# The DiRT 2 menu card's livery, which is what steve pointed at.
DIRT = (58, 48, 38)
SKY = (28, 30, 38)
WHITE = (232, 232, 226)
YELLOW = (240, 196, 32)
TYRE = (26, 26, 28)


def get_font(size: int) -> ImageFont.ImageFont:
    for path in (r"C:\Windows\Fonts\arialbd.ttf", r"C:\Windows\Fonts\arial.ttf"):
        try:
            return ImageFont.truetype(path, size)
        except OSError:
            continue
    return ImageFont.load_default()


def draw_car(draw: ImageDraw.ImageDraw, x: float, y: float, s: float) -> None:
    """A flat side-on rally-car wedge: body, cabin, wing, two wheels.

    s is the half-length in pixels, so the car spans 2*s wide. Deliberately
    crude -- the rear wing and the roofline are the only two shapes that make
    a rally car readable at 48 px, which is the same reasoning the real mesh
    will follow (see the car-art research page in the vault).
    """
    body = [
        (x - s, y), (x - s * 0.86, y - s * 0.30), (x + s * 0.72, y - s * 0.30),
        (x + s, y), (x + s * 0.90, y + s * 0.20), (x - s * 0.90, y + s * 0.20),
    ]
    draw.polygon(body, fill=WHITE)
    cabin = [
        (x - s * 0.46, y - s * 0.30), (x - s * 0.24, y - s * 0.66),
        (x + s * 0.30, y - s * 0.66), (x + s * 0.46, y - s * 0.30),
    ]
    draw.polygon(cabin, fill=YELLOW)
    # Rear wing -- the single most recognisable feature of the silhouette.
    draw.rectangle([x - s * 1.02, y - s * 0.56, x - s * 0.58, y - s * 0.44], fill=YELLOW)
    draw.rectangle([x - s * 0.86, y - s * 0.44, x - s * 0.78, y - s * 0.30], fill=YELLOW)
    for wx in (x - s * 0.58, x + s * 0.56):
        r = s * 0.26
        draw.ellipse([wx - r, y + s * 0.20 - r, wx + r, y + s * 0.20 + r], fill=TYRE)


# ---- Icon: 48x48 (and a 24x24 the SMDH also carries) ----
icon = Image.new("RGB", (48, 48), SKY)
d = ImageDraw.Draw(icon)
d.rectangle([0, 30, 48, 48], fill=DIRT)
draw_car(d, 24, 30, 17)
icon.save(META / "icon.png")
icon.save(ROOT / "icon.png")

# ---- Banner: 256x128 ----
banner = Image.new("RGB", (256, 128), SKY)
d = ImageDraw.Draw(banner)
d.rectangle([0, 84, 256, 128], fill=DIRT)
draw_car(d, 66, 86, 46)
d.text((136, 44), "DiRT2", font=get_font(30), fill=WHITE)
d.text((137, 76), "CLONE", font=get_font(20), fill=YELLOW)
d.text((136, 100), "PLACEHOLDER ART", font=get_font(10), fill=(150, 150, 158))
banner.save(META / "banner.png")
banner.save(CIA / "banner.png")

print(f"wrote {ROOT / 'icon.png'}, {META / 'banner.png'}, {CIA / 'banner.png'}")

# ---- Banner jingle: 0.5 s of silence ----
# bannertool needs a VALID wav; an absent or zero-length one fails the build.
SAMPLE_RATE = 22050
N_SAMPLES = int(SAMPLE_RATE * 0.5)
with wave.open(str(CIA / "banner.wav"), "w") as w:
    w.setnchannels(1)
    w.setsampwidth(2)
    w.setframerate(SAMPLE_RATE)
    w.writeframes(struct.pack("<%dh" % N_SAMPLES, *([0] * N_SAMPLES)))

print(f"wrote {CIA / 'banner.wav'} ({N_SAMPLES} silent samples)")
