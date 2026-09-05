"""
mesh_builder.py -- turns a RawMesh (obj_parser.py, untriangulated and
unwelded) into the flat vertex/index arrays model_format.py's binary
layout expects: triangulated, deduplicated, one flags value, and an AABB.

TRIANGULATION.
  Fan triangulation from each face's first vertex. This is exact and safe
  for a CONVEX polygon; for a non-convex (concave) one, a naive fan can
  produce triangles that fold back over themselves. Rather than silently
  emit folded geometry, every fan triangle's winding is checked against
  the polygon's own Newell-normal: any triangle that disagrees in
  direction, or has ~zero area, aborts the whole conversion with the
  source face's OBJ line number -- see _triangulate_face. The fix, when
  this fires, is enabling "Triangulate Faces" in Blender's OBJ exporter
  (or manually triangulating that face) so this tool never has to guess.

WELDING.
  Two face-vertices are the same output vertex iff they reference the same
  (position, normal, uv) index triple from the OBJ's own pools -- the
  standard definition of "vertex" for a GPU mesh (a hard edge or a UV seam
  legitimately duplicates a position into two GPU vertices; that is
  correct, not a welding failure).

VERTEX COLOR.
  Always synthesized as flat white (1, 1, 1, 1) -- OBJ has no standard
  per-vertex color and Blender's stock exporter does not write one. See
  model_format.py's FLAG_HAS_COLOR comment for why color is nonetheless
  ALWAYS present in the output: today's one vertex shader
  (source/render/dirt2.v.pica) hard-expects v0=position, v1=color, so
  every model this tool produces must supply a color attribute to render
  correctly through it, even if that color carries no information yet.
"""
import math

from model_format import MAX_VERTEX_COUNT
from obj_parser import ObjParseError

_AREA_EPS = 1e-12     # squared-length threshold below which a triangle is "zero area"
_CONVEX_EPS = 1e-9    # dot-product threshold for "this fan triangle agrees with the
                      # polygon's own winding"


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _length_sq(a):
    return _dot(a, a)


def _newell_normal(points):
    nx = ny = nz = 0.0
    n = len(points)
    for i in range(n):
        x0, y0, z0 = points[i]
        x1, y1, z1 = points[(i + 1) % n]
        nx += (y0 - y1) * (z0 + z1)
        ny += (z0 - z1) * (x0 + x1)
        nz += (x0 - x1) * (y0 + y1)
    return (nx, ny, nz)


def _triangulate_face(face, positions, line_no):
    """face: list of (pos_idx, norm_idx, uv_idx). Returns a list of
    (i0, i1, i2) index-triples into `face` (NOT into the OBJ position
    pool) forming a fan triangulation, after verifying every fan triangle
    agrees in winding with the polygon's own Newell normal. Raises
    ObjParseError, citing `line_no`, the moment it finds a triangle it
    cannot trust."""
    n = len(face)
    pts = [positions[v[0]] for v in face]

    if n == 3:
        # A lone triangle is always convex; still reject if degenerate
        # (three collinear/coincident points), same as any fan triangle.
        cross = _cross(_sub(pts[1], pts[0]), _sub(pts[2], pts[0]))
        if _length_sq(cross) < _AREA_EPS:
            raise ObjParseError(
                f"line {line_no}: degenerate triangle (zero area) -- "
                f"two or more vertices coincide or are collinear")
        return [(0, 1, 2)]

    polygon_normal = _newell_normal(pts)
    if _length_sq(polygon_normal) < _AREA_EPS:
        raise ObjParseError(
            f"line {line_no}: {n}-gon has ~zero area (degenerate polygon) -- "
            f"cannot triangulate")

    triangles = []
    for i in range(1, n - 1):
        a, b, c = pts[0], pts[i], pts[i + 1]
        cross = _cross(_sub(b, a), _sub(c, a))
        area_sq = _length_sq(cross)
        if area_sq < _AREA_EPS:
            raise ObjParseError(
                f"line {line_no}: {n}-gon produces a zero-area triangle when "
                f"fan-triangulated from its first vertex -- triangulate this "
                f"face in Blender before exporting")
        if _dot(cross, polygon_normal) < _CONVEX_EPS:
            raise ObjParseError(
                f"line {line_no}: {n}-gon cannot be safely fan-triangulated "
                f"(it is non-convex) -- enable 'Triangulate Faces' in "
                f"Blender's OBJ exporter, or triangulate this face by hand, "
                f"and re-export")
        triangles.append((0, i, i + 1))
    return triangles


class BuiltMesh:
    def __init__(self):
        self.flags = 0
        self.positions = []   # list of (x, y, z), one per OUTPUT vertex
        self.normals = []     # list of (x, y, z), same length, only if HAS_NORMAL
        self.uvs = []         # list of (u, v), same length, only if HAS_UV
        self.indices = []     # list of int, length = 3 * triangle_count
        self.bounds_min = (0.0, 0.0, 0.0)
        self.bounds_max = (0.0, 0.0, 0.0)

    @property
    def vertex_count(self):
        return len(self.positions)

    @property
    def triangle_count(self):
        return len(self.indices) // 3


def build_mesh(raw):
    """raw: obj_parser.RawMesh. Returns a BuiltMesh. Raises ObjParseError
    on anything that would otherwise produce bad/inconsistent geometry
    (mixed presence of normals/uvs across faces, too many unique vertices
    for a 16-bit index, an unsafe n-gon -- see _triangulate_face)."""

    has_normal_votes = {v[1] is not None for face in raw.faces for v in face}
    if has_normal_votes == {True, False}:
        raise ObjParseError(
            "some face vertices supply a normal (the 'vt/vn' slot) and "
            "others don't -- export normals for the whole mesh, or none of it")
    has_normal = True in has_normal_votes

    has_uv_votes = {v[2] is not None for face in raw.faces for v in face}
    if has_uv_votes == {True, False}:
        raise ObjParseError(
            "some face vertices supply a UV coordinate and others don't -- "
            "export UVs for the whole mesh, or none of it")
    has_uv = True in has_uv_votes

    weld = {}          # (pos_idx, norm_idx, uv_idx) -> output vertex index
    out_positions = []
    out_normals = []
    out_uvs = []
    out_indices = []

    for face, line_no in zip(raw.faces, raw.face_lines):
        for tri in _triangulate_face(face, raw.positions, line_no):
            for local_i in tri:
                pos_idx, norm_idx, uv_idx = face[local_i]
                key = (pos_idx, norm_idx, uv_idx)
                out_i = weld.get(key)
                if out_i is None:
                    out_i = len(out_positions)
                    weld[key] = out_i
                    out_positions.append(raw.positions[pos_idx])
                    if has_normal:
                        out_normals.append(raw.normals[norm_idx])
                    if has_uv:
                        out_uvs.append(raw.uvs[uv_idx])
                out_indices.append(out_i)

    if len(out_positions) > MAX_VERTEX_COUNT:
        raise ObjParseError(
            f"mesh has {len(out_positions)} unique vertices after welding, "
            f"which exceeds the 16-bit index buffer limit of "
            f"{MAX_VERTEX_COUNT} -- reduce mesh detail or split it into "
            f"multiple models")

    mesh = BuiltMesh()
    mesh.positions = out_positions
    mesh.normals = out_normals if has_normal else []
    mesh.uvs = out_uvs if has_uv else []
    mesh.indices = out_indices

    from model_format import FLAG_HAS_COLOR, FLAG_HAS_NORMAL, FLAG_HAS_UV
    flags = FLAG_HAS_COLOR
    if has_normal:
        flags |= FLAG_HAS_NORMAL
    if has_uv:
        flags |= FLAG_HAS_UV
    mesh.flags = flags

    xs = [p[0] for p in out_positions]
    ys = [p[1] for p in out_positions]
    zs = [p[2] for p in out_positions]
    mesh.bounds_min = (min(xs), min(ys), min(zs))
    mesh.bounds_max = (max(xs), max(ys), max(zs))

    return mesh
