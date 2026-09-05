"""
test_roundtrip.py -- builds a known mesh (a flat-shaded unit cube: 8 corner
positions, 6 face normals, 4 UV corners -> 24 unique (pos,normal,uv)
vertices after welding, 12 triangles), runs it through convert.py, then
parses the produced binary back with plain struct.unpack (NOT by reusing
convert.py's own packing code) and asserts every field matches the source
mesh within tolerance. This is the "does the binary actually say what the
converter thinks it said" check -- see model_format.py for the layout.

Run directly:  python tests/test_roundtrip.py
"""
import os
import struct
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import convert
import model_format

CUBE_OBJ = """\
# unit cube, flat-shaded (one normal per face), centered at the origin
v -1 -1 -1
v  1 -1 -1
v  1  1 -1
v -1  1 -1
v -1 -1  1
v  1 -1  1
v  1  1  1
v -1  1  1

vn 0 0 -1
vn 0 0 1
vn -1 0 0
vn 1 0 0
vn 0 -1 0
vn 0 1 0

vt 0 0
vt 1 0
vt 1 1
vt 0 1

f 1/1/1 2/2/1 3/3/1 4/4/1
f 5/1/2 6/2/2 7/3/2 8/4/2
f 1/1/3 4/2/3 8/3/3 5/4/3
f 2/1/4 3/2/4 7/3/4 6/4/4
f 1/1/5 2/2/5 6/3/5 5/4/5
f 4/1/6 3/2/6 7/3/6 8/4/6
"""

EXPECTED_VERTEX_COUNT = 24   # 4 corners x 6 faces, each face has its own normal/uv
EXPECTED_TRIANGLE_COUNT = 12  # 2 per quad face x 6 faces
EXPECTED_INDEX_COUNT = 36
EXPECTED_STRIDE = 4 * model_format.ATTR_BYTES  # position, color, normal, uv


class RoundTripTest(unittest.TestCase):
    def setUp(self):
        self.tmpdir = tempfile.mkdtemp(prefix="d2m_roundtrip_")
        self.obj_path = os.path.join(self.tmpdir, "cube.obj")
        self.bin_path = os.path.join(self.tmpdir, "cube.d2m")
        with open(self.obj_path, "w", encoding="utf-8") as f:
            f.write(CUBE_OBJ)

    def test_roundtrip(self):
        mesh, total_bytes = convert.convert(self.obj_path, self.bin_path)

        self.assertEqual(mesh.vertex_count, EXPECTED_VERTEX_COUNT)
        self.assertEqual(mesh.triangle_count, EXPECTED_TRIANGLE_COUNT)
        self.assertEqual(len(mesh.indices), EXPECTED_INDEX_COUNT)

        with open(self.bin_path, "rb") as f:
            blob = f.read()

        self.assertEqual(len(blob), total_bytes)

        header = model_format.unpack_header(blob[:model_format.HEADER_SIZE])
        self.assertEqual(header["vertex_count"], EXPECTED_VERTEX_COUNT)
        self.assertEqual(header["index_count"], EXPECTED_INDEX_COUNT)
        self.assertEqual(header["vertex_stride"], EXPECTED_STRIDE)
        expected_flags = (model_format.FLAG_HAS_COLOR
                           | model_format.FLAG_HAS_NORMAL
                           | model_format.FLAG_HAS_UV)
        self.assertEqual(header["flags"], expected_flags)

        for a, b in zip(header["bounds_min"], (-1.0, -1.0, -1.0)):
            self.assertAlmostEqual(a, b, places=6)
        for a, b in zip(header["bounds_max"], (1.0, 1.0, 1.0)):
            self.assertAlmostEqual(a, b, places=6)

        vdata = blob[header["vertex_data_offset"]:
                     header["vertex_data_offset"] + EXPECTED_VERTEX_COUNT * EXPECTED_STRIDE]
        for i in range(EXPECTED_VERTEX_COUNT):
            base = i * EXPECTED_STRIDE
            px, py, pz, pw = struct.unpack_from("<4f", vdata, base)
            cr, cg, cb, ca = struct.unpack_from("<4f", vdata, base + 16)
            nx, ny, nz, nw = struct.unpack_from("<4f", vdata, base + 32)
            u, v, u2, u3 = struct.unpack_from("<4f", vdata, base + 48)

            self.assertAlmostEqual(pw, 1.0, places=6)
            self.assertEqual((px, py, pz), mesh.positions[i])

            self.assertEqual((cr, cg, cb, ca), (1.0, 1.0, 1.0, 1.0))

            self.assertAlmostEqual(nw, 0.0, places=6)
            self.assertEqual((nx, ny, nz), mesh.normals[i])
            # every synthesized normal is a unit axis vector -- confirms
            # welding did not average/blend distinct face normals together.
            self.assertAlmostEqual(nx * nx + ny * ny + nz * nz, 1.0, places=6)

            self.assertEqual((u, v), mesh.uvs[i])
            self.assertEqual((u2, u3), (0.0, 0.0))

        idata = blob[header["index_data_offset"]:
                     header["index_data_offset"] + EXPECTED_INDEX_COUNT * 2]
        indices = list(struct.unpack_from(f"<{EXPECTED_INDEX_COUNT}H", idata, 0))
        self.assertEqual(indices, mesh.indices)
        self.assertTrue(all(0 <= i < EXPECTED_VERTEX_COUNT for i in indices))

        print(f"[roundtrip] vertices={mesh.vertex_count} "
              f"triangles={mesh.triangle_count} "
              f"index_count={len(indices)} "
              f"stride={header['vertex_stride']}B "
              f"file_size={total_bytes}B "
              f"bounds_min={header['bounds_min']} bounds_max={header['bounds_max']}")


if __name__ == "__main__":
    unittest.main(verbosity=2)
