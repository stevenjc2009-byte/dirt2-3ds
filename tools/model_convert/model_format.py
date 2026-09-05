"""
model_format.py -- the DiRT2 model binary layout (".d2m"), in one place.

This is the Python-side mirror of source/model/model.h's documented layout.
convert.py packs a file with this module; the round-trip tests in tests/
unpack one with it. If the layout ever changes, model.h's comment block and
this module must change together -- keep them byte-for-byte identical, that
is the entire point of writing it down twice in two languages instead of
once and hoping.

WHY 64-BYTE HEADER, WHY THESE FIELDS:
  little-endian, fixed-size, every multi-byte field on its own natural
  boundary (no padding surprises), so the 3DS loader (model.c) can read it
  with plain sequential struct-style field access with no per-field
  fixup -- see model.h for the full byte-offset table.

WHY EVERY VERTEX ATTRIBUTE IS EXACTLY 4 FLOATS:
  Measured fact from this machine's prior 3DS work: a 3-component vertex
  attribute declared before a 4-component one freezes real PICA200 hardware.
  This project's existing shader/vertex code (source/render/dirt2.v.pica,
  debugdraw.c's DebugVertex) already sidesteps the whole ordering question by
  making every attribute 4-component -- this format follows the same
  convention for the same reason, rather than trying to get a "correct"
  3-before-4 ordering right and hoping nobody ever reorders the flags.
"""
import struct

MAGIC = b"D2MB"          # "DiRT2 Model Binary" -- the 4 bytes stored at offset 0
VERSION = 1

# Bit flags stored in the header's `flags` field (offset 6, u16).
FLAG_HAS_NORMAL = 1 << 0   # optional NORMAL attribute present
FLAG_HAS_UV = 1 << 1       # optional UV attribute present
FLAG_HAS_COLOR = 1 << 2    # COLOR attribute present -- version 1 ALWAYS sets
                           # this (see convert.py: vertex color is synthesized
                           # white when the source mesh has none), so that
                           # every model this converter produces is
                           # immediately drawable by today's one vertex
                           # shader (dirt2.v.pica), which hard-expects
                           # v0 = position, v1 = color and nothing else. A
                           # future format version could allow omitting color
                           # for a different shader; version 1 does not.
_KNOWN_FLAGS_MASK = FLAG_HAS_NORMAL | FLAG_HAS_UV | FLAG_HAS_COLOR

# Every attribute (position, color, normal, uv) is stored as exactly 4
# little-endian float32s -- see module header comment on why.
ATTR_FLOATS = 4
ATTR_BYTES = ATTR_FLOATS * 4  # 16

HEADER_SIZE = 64

# <  little-endian, no padding (we lay out every field ourselves)
# 4s magic            char[4]  offset  0
# H  version          u16      offset  4
# H  flags            u16      offset  6
# I  vertex_count     u32      offset  8
# I  index_count      u32      offset 12
# I  vertex_stride    u32      offset 16
# I  vertex_data_offset u32    offset 20
# I  index_data_offset  u32    offset 24
# f  bounds_min.x     f32      offset 28
# f  bounds_min.y     f32      offset 32
# f  bounds_min.z     f32      offset 36
# f  bounds_max.x     f32      offset 40
# f  bounds_max.y     f32      offset 44
# f  bounds_max.z     f32      offset 48
# I  reserved0        u32      offset 52 (must be 0)
# I  reserved1        u32      offset 56 (must be 0)
# I  reserved2        u32      offset 60 (must be 0)
HEADER_STRUCT = struct.Struct("<4sHHIIIIIffffffIII")
assert HEADER_STRUCT.size == HEADER_SIZE, (
    f"model_format.py's HEADER_STRUCT is {HEADER_STRUCT.size} bytes, "
    f"expected {HEADER_SIZE} -- keep this in sync with model.h's layout table"
)

# Index buffer element type: GPU_UNSIGNED_SHORT on the citro3d side, so the
# vertex count this format can address is capped at 65535 (0xFFFF is left
# free as a "not an index" sentinel headroom, not used today but cheap to
# reserve).
MAX_VERTEX_COUNT = 0xFFFF


def attribute_order(flags):
    """Returns the fixed, flags-dependent attribute order as a list of
    semantic names, e.g. ["position", "color", "normal"]. POSITION and
    COLOR are always present and always first, in that order -- see
    FLAG_HAS_COLOR's comment above for why COLOR is never optional in this
    version. NORMAL then UV follow, each only if their flag bit is set.
    This is the single source of truth both convert.py (packing) and
    model.c (unpacking / AttrInfo setup) must compute identically."""
    order = ["position", "color"]
    if flags & FLAG_HAS_NORMAL:
        order.append("normal")
    if flags & FLAG_HAS_UV:
        order.append("uv")
    return order


def vertex_stride(flags):
    return len(attribute_order(flags)) * ATTR_BYTES


def pack_header(flags, vertex_count, index_count, vertex_data_offset,
                index_data_offset, bounds_min, bounds_max):
    return HEADER_STRUCT.pack(
        MAGIC, VERSION, flags, vertex_count, index_count,
        vertex_stride(flags), vertex_data_offset, index_data_offset,
        bounds_min[0], bounds_min[1], bounds_min[2],
        bounds_max[0], bounds_max[1], bounds_max[2],
        0, 0, 0,
    )


def unpack_header(data):
    """Parses a 64-byte header blob into a plain dict. Raises ValueError
    with a specific message on anything the loader would also have to
    reject (bad magic, unknown version, unknown flag bits, inconsistent
    stride) -- used by the round-trip tests to exercise the exact failure
    text a human would see, without needing the C loader built."""
    if len(data) < HEADER_SIZE:
        raise ValueError(
            f"truncated header: got {len(data)} bytes, need {HEADER_SIZE}")

    (magic, version, flags, vertex_count, index_count, stride,
     vdata_off, idata_off, minx, miny, minz, maxx, maxy, maxz,
     r0, r1, r2) = HEADER_STRUCT.unpack(data[:HEADER_SIZE])

    if magic != MAGIC:
        raise ValueError(
            f"bad magic: expected {MAGIC!r}, got {magic!r} -- not a DiRT2 "
            f"model file, or it is corrupt")
    if version != VERSION:
        raise ValueError(
            f"unsupported version: file is version {version}, this reader "
            f"understands version {VERSION} -- reconvert the source asset")
    if flags & ~_KNOWN_FLAGS_MASK:
        raise ValueError(
            f"unknown flag bits set: 0x{flags:04x} (known mask "
            f"0x{_KNOWN_FLAGS_MASK:04x}) -- file is from a newer/different "
            f"converter than this reader understands")
    if not (flags & FLAG_HAS_COLOR):
        raise ValueError(
            "HAS_COLOR flag is not set -- version 1 requires every model "
            "to carry a color attribute (see FLAG_HAS_COLOR's comment)")
    if (r0, r1, r2) != (0, 0, 0):
        raise ValueError(
            f"reserved header fields are non-zero (0x{r0:x}, 0x{r1:x}, "
            f"0x{r2:x}) -- file is corrupt or from an incompatible writer")
    expected_stride = vertex_stride(flags)
    if stride != expected_stride:
        raise ValueError(
            f"vertex_stride field ({stride}) does not match what flags "
            f"0x{flags:04x} implies ({expected_stride}) -- file is corrupt")

    return {
        "magic": magic, "version": version, "flags": flags,
        "vertex_count": vertex_count, "index_count": index_count,
        "vertex_stride": stride,
        "vertex_data_offset": vdata_off, "index_data_offset": idata_off,
        "bounds_min": (minx, miny, minz), "bounds_max": (maxx, maxy, maxz),
    }
