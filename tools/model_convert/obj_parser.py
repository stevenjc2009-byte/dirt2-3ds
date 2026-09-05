"""
obj_parser.py -- minimal Wavefront OBJ reader, stdlib only.

WHY OBJ, NOT glTF/GLB:
  OBJ is plain ASCII with three data lines (`v`, `vn`, `vt`) and one face
  line (`f`) that matter here. Parsing it needs nothing beyond string
  splitting -- no JSON schema, no base64-embedded-buffer decoding, no
  accessor/bufferView/componentType indirection, no glTF extension
  surface. glTF is the better format once this project needs skinning,
  animation, or multiple materials in one file; none of that exists in
  dirt2's runtime yet (no bones, no texture pipeline -- see model.h), so
  the extra parser surface would be paid for with nothing to spend it on.
  Blender's stock OBJ exporter is also a one-click, no-plugin path, which
  glTF (2.8+) is too, but OBJ's exported text is trivially diffable/
  readable by a human when something looks wrong -- worth something on a
  from-scratch pipeline nobody has debugged yet.

BLENDER EXPORT WORKFLOW THIS ASSUMES:
  File > Export > Wavefront (.obj). Leave "Forward Axis: -Z, Up Axis: Y"
  at Blender's own default -- that already matches dirt2's Y-up world
  convention (renderer.c's Mtx_LookAt uses world up = (0, 1, 0)), so no
  axis remapping is done anywhere in this pipeline; whatever axes the OBJ
  file states are used verbatim. Turn on "Triangulate Faces" in the
  exporter for anything with concave n-gons -- this parser fan-triangulates
  convex polygons itself (see mesh_builder.py) but will loudly refuse a
  polygon it cannot safely fan-triangulate rather than guess.
  Vertex colors are NOT read from the OBJ (Blender's stock exporter does
  not write them) -- see mesh_builder.py for what fills the color
  attribute instead.

WHAT IS IGNORED ON PURPOSE: `o`/`g` (object/group names), `usemtl`/
`mtllib` (materials), `s` (smoothing groups), blank lines, comments (#).
Everything in a multi-object OBJ is merged into one mesh -- this converter
emits one binary model per invocation; splitting a scene into multiple
models is a Blender-side (separate export) decision, not this tool's.
"""


class ObjParseError(ValueError):
    """Raised for any OBJ content this parser cannot make sense of. Always
    carries a specific, human-actionable message -- never a bare
    IndexError/ValueError from a failed split()."""


class RawMesh:
    """Untriangulated, unwelded OBJ data: raw attribute pools plus a list
    of faces, each face a list of (position_index, normal_index_or_None,
    uv_index_or_None) tuples in file order (may be >3 per face)."""

    def __init__(self):
        self.positions = []   # list of (x, y, z)
        self.normals = []     # list of (x, y, z)
        self.uvs = []         # list of (u, v)
        self.faces = []       # list of list of (pos_idx, norm_idx, uv_idx)
        self.face_lines = []  # source line number per face, for error messages


def _parse_floats(parts, count, line_no, tag):
    if len(parts) < count:
        raise ObjParseError(
            f"line {line_no}: '{tag}' needs at least {count} numbers, "
            f"got {len(parts) - 1}: {' '.join(parts)!r}")
    try:
        return tuple(float(p) for p in parts[1:1 + count])
    except ValueError as exc:
        raise ObjParseError(
            f"line {line_no}: '{tag}' has a non-numeric value: "
            f"{' '.join(parts)!r}") from exc


def _resolve_index(raw, pool_len, line_no, what):
    """OBJ indices are 1-based, and may be negative (relative to the
    CURRENT end of that attribute pool, per the OBJ spec) -- both forms
    are handled here, everything else is a hard parse error rather than a
    guess."""
    try:
        i = int(raw)
    except ValueError as exc:
        raise ObjParseError(
            f"line {line_no}: non-integer {what} index {raw!r} in face") from exc
    if i > 0:
        idx = i - 1
    elif i < 0:
        idx = pool_len + i
    else:
        raise ObjParseError(f"line {line_no}: {what} index 0 is invalid (OBJ is 1-based)")
    if idx < 0 or idx >= pool_len:
        raise ObjParseError(
            f"line {line_no}: {what} index {i} is out of range "
            f"(pool has {pool_len} entries)")
    return idx


def parse_obj(path):
    """Reads the .obj at `path` and returns a RawMesh. Raises
    FileNotFoundError (propagated, message already specific enough) or
    ObjParseError on malformed content. Never returns a partially-useful
    mesh silently -- either it fully parses or it raises."""
    mesh = RawMesh()

    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line_no, raw_line in enumerate(f, start=1):
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            tag = parts[0]

            if tag == "v":
                mesh.positions.append(_parse_floats(parts, 3, line_no, "v"))
            elif tag == "vn":
                mesh.normals.append(_parse_floats(parts, 3, line_no, "vn"))
            elif tag == "vt":
                uv = _parse_floats(parts, 2, line_no, "vt")
                mesh.uvs.append(uv)
            elif tag == "f":
                if len(parts) < 4:
                    raise ObjParseError(
                        f"line {line_no}: 'f' needs at least 3 vertices, "
                        f"got {len(parts) - 1}: {line!r}")
                face = []
                for token in parts[1:]:
                    fields = token.split("/")
                    if len(fields) not in (1, 2, 3) or fields[0] == "":
                        raise ObjParseError(
                            f"line {line_no}: malformed face vertex {token!r}")
                    pos_idx = _resolve_index(fields[0], len(mesh.positions), line_no, "position")
                    uv_idx = None
                    norm_idx = None
                    if len(fields) >= 2 and fields[1] != "":
                        uv_idx = _resolve_index(fields[1], len(mesh.uvs), line_no, "uv")
                    if len(fields) == 3 and fields[2] != "":
                        norm_idx = _resolve_index(fields[2], len(mesh.normals), line_no, "normal")
                    face.append((pos_idx, norm_idx, uv_idx))
                mesh.faces.append(face)
                mesh.face_lines.append(line_no)
            else:
                # o, g, usemtl, mtllib, s, vp, l, and anything unrecognised:
                # deliberately ignored, see module header comment.
                continue

    if not mesh.positions:
        raise ObjParseError(f"'{path}' has no 'v' (vertex position) lines -- not a mesh export")
    if not mesh.faces:
        raise ObjParseError(
            f"'{path}' has {len(mesh.positions)} vertices but no 'f' (face) lines -- "
            f"a points-only or empty mesh cannot be converted")

    return mesh
