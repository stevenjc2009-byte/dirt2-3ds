#!/usr/bin/env python3
"""
convert.py -- offline OBJ -> DiRT2 model binary (".d2m") converter.

    python convert.py <input.obj> <output.d2m>

Runs entirely on the PC (this is not part of the 3DS build -- see
source/model/model.h/.c for the runtime loader that reads what this tool
writes). stdlib only, no third-party dependency: see obj_parser.py's
header comment for why OBJ was chosen over glTF for this project's current
needs, and model_format.py for the exact binary layout produced.

Never writes a partial/corrupt output file: parsing, triangulation,
welding and validation all happen in memory first (see obj_parser.py and
mesh_builder.py), and the header + all geometry are assembled into one
in-memory buffer before anything is opened for writing. Any failure along
the way exits non-zero with a specific message and touches the output
path not at all.
"""
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import model_format
from mesh_builder import build_mesh
from obj_parser import ObjParseError, parse_obj


def convert(input_path, output_path):
    if not os.path.isfile(input_path):
        raise FileNotFoundError(f"no such file: '{input_path}'")

    raw = parse_obj(input_path)
    mesh = build_mesh(raw)

    vertex_data = bytearray()
    for i in range(mesh.vertex_count):
        px, py, pz = mesh.positions[i]
        vertex_data += _pack4f(px, py, pz, 1.0)          # position, w = 1.0
        vertex_data += _pack4f(1.0, 1.0, 1.0, 1.0)       # color, synthesized white
        if mesh.flags & model_format.FLAG_HAS_NORMAL:
            nx, ny, nz = mesh.normals[i]
            vertex_data += _pack4f(nx, ny, nz, 0.0)
        if mesh.flags & model_format.FLAG_HAS_UV:
            u, v = mesh.uvs[i]
            vertex_data += _pack4f(u, v, 0.0, 0.0)

    index_data = bytearray()
    for idx in mesh.indices:
        index_data += struct.pack("<H", idx)

    vertex_data_offset = model_format.HEADER_SIZE
    index_data_offset = vertex_data_offset + len(vertex_data)

    header = model_format.pack_header(
        flags=mesh.flags,
        vertex_count=mesh.vertex_count,
        index_count=len(mesh.indices),
        vertex_data_offset=vertex_data_offset,
        index_data_offset=index_data_offset,
        bounds_min=mesh.bounds_min,
        bounds_max=mesh.bounds_max,
    )

    blob = bytes(header) + bytes(vertex_data) + bytes(index_data)

    out_dir = os.path.dirname(os.path.abspath(output_path))
    if out_dir and not os.path.isdir(out_dir):
        os.makedirs(out_dir, exist_ok=True)
    with open(output_path, "wb") as f:
        f.write(blob)

    return mesh, len(blob)


def _pack4f(a, b, c, d):
    return struct.pack("<4f", a, b, c, d)


def main(argv):
    if len(argv) != 3:
        print(f"usage: {argv[0]} <input.obj> <output.d2m>", file=sys.stderr)
        return 2

    input_path, output_path = argv[1], argv[2]
    try:
        mesh, total_bytes = convert(input_path, output_path)
    except FileNotFoundError as exc:
        print(f"convert.py: error: {exc}", file=sys.stderr)
        return 1
    except ObjParseError as exc:
        print(f"convert.py: error: {input_path}: {exc}", file=sys.stderr)
        return 1

    attrs = "+".join(model_format.attribute_order(mesh.flags))
    print(f"convert.py: {input_path} -> {output_path}")
    print(f"  vertices:   {mesh.vertex_count}")
    print(f"  triangles:  {mesh.triangle_count}")
    print(f"  attributes: {attrs} (stride {model_format.vertex_stride(mesh.flags)} bytes)")
    print(f"  bounds min: {mesh.bounds_min}")
    print(f"  bounds max: {mesh.bounds_max}")
    print(f"  output size: {total_bytes} bytes")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
