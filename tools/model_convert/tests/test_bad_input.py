"""
test_bad_input.py -- feeds convert.py deliberately broken input and asserts
it fails loudly with a specific message, and touches the output path not
at all (no half-written file left behind). See convert.py's header comment
for the "parse everything into memory, write once" design this is
checking.

Run directly:  python tests/test_bad_input.py
"""
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import convert
from obj_parser import ObjParseError


class BadInputTest(unittest.TestCase):
    def setUp(self):
        self.tmpdir = tempfile.mkdtemp(prefix="d2m_badinput_")
        self.out_path = os.path.join(self.tmpdir, "out.d2m")

    def _write(self, name, text):
        path = os.path.join(self.tmpdir, name)
        with open(path, "w", encoding="utf-8") as f:
            f.write(text)
        return path

    def _assert_no_output_written(self):
        self.assertFalse(os.path.exists(self.out_path),
                          "convert() must not write an output file on failure")

    def test_missing_file(self):
        missing = os.path.join(self.tmpdir, "does_not_exist.obj")
        with self.assertRaises(FileNotFoundError) as ctx:
            convert.convert(missing, self.out_path)
        print(f"[missing_file] {ctx.exception}")
        self.assertIn("does_not_exist.obj", str(ctx.exception))
        self._assert_no_output_written()

    def test_no_mesh_points_only(self):
        path = self._write("points_only.obj", "v 0 0 0\nv 1 0 0\nv 0 1 0\n")
        with self.assertRaises(ObjParseError) as ctx:
            convert.convert(path, self.out_path)
        print(f"[no_mesh] {ctx.exception}")
        self.assertIn("no 'f'", str(ctx.exception))
        self._assert_no_output_written()

    def test_empty_file(self):
        path = self._write("empty.obj", "")
        with self.assertRaises(ObjParseError) as ctx:
            convert.convert(path, self.out_path)
        print(f"[empty_file] {ctx.exception}")
        self.assertIn("no 'v'", str(ctx.exception))
        self._assert_no_output_written()

    def test_concave_ngon(self):
        # A reflex ("dart") quadrilateral: going A(0,0) -> B(4,4) -> C(4,0)
        # -> D(1,1) -> back to A, D is pulled in toward the polygon's own
        # interior, so fanning from A produces triangle (A, C, D) whose
        # winding is the OPPOSITE of the polygon's own Newell normal
        # (verified numerically: cross(C-A, D-A) . polygon_normal < 0),
        # not merely degenerate -- this exercises the non-convex branch of
        # _triangulate_face specifically, distinct from a zero-area/
        # collinear-points failure.
        obj = (
            "v 0 0 0\n"
            "v 4 4 0\n"
            "v 4 0 0\n"
            "v 1 1 0\n"
            "f 1 2 3 4\n"
        )
        path = self._write("concave.obj", obj)
        with self.assertRaises(ObjParseError) as ctx:
            convert.convert(path, self.out_path)
        print(f"[concave_ngon] {ctx.exception}")
        self.assertIn("non-convex", str(ctx.exception))
        self._assert_no_output_written()

    def test_too_many_vertices_for_u16_index(self):
        # 21846 fully isolated triangles = 65538 unique vertices (no two
        # triangles share a position, so nothing welds away) -- one more
        # than the 16-bit index buffer's 65535-vertex ceiling.
        triangle_count = 21846
        lines = []
        for i in range(triangle_count):
            ox = i * 10.0
            base = i * 3
            lines.append(f"v {ox} 0 0")
            lines.append(f"v {ox + 1} 0 0")
            lines.append(f"v {ox} 1 0")
            lines.append(f"f {base + 1} {base + 2} {base + 3}")
        path = self._write("too_many_verts.obj", "\n".join(lines) + "\n")
        with self.assertRaises(ObjParseError) as ctx:
            convert.convert(path, self.out_path)
        print(f"[too_many_vertices] {ctx.exception}")
        self.assertIn("16-bit index buffer limit", str(ctx.exception))
        self._assert_no_output_written()


if __name__ == "__main__":
    unittest.main(verbosity=2)
