/*---------------------------------------------------------------------------------
 * model.c -- see model.h for the binary format this reads, the ownership
 * rules this follows (no shaderProgram_s of its own), and the two
 * integration steps (romfs init, a renderer.c wrapper) this still needs
 * from elsewhere before a loaded Model reaches the screen.
 *
 * HEADER FIELDS ARE PARSED, VERTEX/INDEX DATA IS NOT.
 *   The 64-byte header is read field-by-field with explicit little-endian
 *   byte assembly (read_u16le/read_u32le/read_f32le below) -- cheap, done
 *   once per model, and correct regardless of the host's own endianness.
 *   The vertex and index DATA, by contrast, is fread() straight into a
 *   linear-memory buffer with no per-element loop: both this converter's
 *   PC host and the 3DS's ARM11 are little-endian IEEE754, so the bytes on
 *   disk are already exactly what citro3d needs to read directly. See
 *   model.h's format comment for why this is true by construction, not by
 *   luck.
 *---------------------------------------------------------------------------------*/
#include "model/model.h"
#include "core/vecmath.h" /* vec3_make, for bounds_min/bounds_max below --
    plain C, no 3DS dependency, same as testground.c/renderer.c's own use
    of it. */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef __3DS__
#include <3ds.h>
#include <citro3d.h>
#endif

static uint16_t read_u16le(const uint8_t *p) {
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t read_u32le(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static float read_f32le(const uint8_t *p) {
    uint32_t bits = read_u32le(p);
    float value;
    memcpy(&value, &bits, sizeof(value)); /* reinterpret, not convert --
        IEEE754 bit pattern already matches a little-endian host's own
        `float` layout (both this converter's PC and the 3DS's ARM11 are
        little-endian), so this is a bit-for-bit reconstruction, not a
        numeric conversion. */
    return value;
}

uint32_t model_attribute_slot_count(uint16_t flags) {
    /* position + color are always present -- see model.h's
     * MODEL_FLAG_HAS_COLOR comment on why color is never optional in
     * version 1. This must compute the same slot count, in the same
     * fixed priority order (position, color, normal, uv), as
     * tools/model_convert/model_format.py's attribute_order(). */
    uint32_t slots = 2;
    if (flags & MODEL_FLAG_HAS_NORMAL) slots++;
    if (flags & MODEL_FLAG_HAS_UV) slots++;
    return slots;
}

static void model_zero(Model *m) {
    memset(m, 0, sizeof(*m));
}

static void *model_alloc(size_t bytes) {
#ifdef __3DS__
    /* GPU-readable buffers must be in linear memory -- see model.h's
     * header comment and this project's hard PICA200 constraints. */
    return linearAlloc(bytes);
#else
    return malloc(bytes);
#endif
}

static void model_free_buffer(void *p) {
    if (!p) return;
#ifdef __3DS__
    linearFree(p);
#else
    free(p);
#endif
}

bool model_load(Model *out, const char *path) {
    FILE *f = NULL;
    uint8_t header_bytes[MODEL_HEADER_SIZE];
    size_t got;
    uint16_t version, flags;
    uint32_t vertex_count, index_count, vertex_stride;
    uint32_t vertex_data_offset, index_data_offset;
    uint32_t reserved0, reserved1, reserved2;
    uint32_t expected_stride;
    size_t vertex_bytes, index_bytes;
    void *vbuf = NULL;
    void *ibuf = NULL;

    if (!out) return false;
    model_zero(out);

    if (!path) {
        fprintf(stderr, "model_load: NULL path\n");
        return false;
    }

    f = fopen(path, "rb");
    if (!f) {
        fprintf(stderr, "model_load: cannot open '%s'\n", path);
        return false;
    }

    got = fread(header_bytes, 1, MODEL_HEADER_SIZE, f);
    if (got != MODEL_HEADER_SIZE) {
        fprintf(stderr, "model_load: '%s': truncated header (%zu/%u bytes)\n",
                path, got, MODEL_HEADER_SIZE);
        fclose(f);
        return false;
    }

    if (header_bytes[0] != MODEL_MAGIC0 || header_bytes[1] != MODEL_MAGIC1 ||
        header_bytes[2] != MODEL_MAGIC2 || header_bytes[3] != MODEL_MAGIC3) {
        fprintf(stderr,
                "model_load: '%s': bad magic (got 0x%02x%02x%02x%02x, "
                "expected \"D2MB\") -- not a DiRT2 model file, or it is "
                "corrupt\n",
                path, header_bytes[0], header_bytes[1], header_bytes[2], header_bytes[3]);
        fclose(f);
        return false;
    }

    version = read_u16le(&header_bytes[4]);
    if (version != MODEL_VERSION) {
        fprintf(stderr,
                "model_load: '%s': unsupported version %u (this loader "
                "understands version %u) -- reconvert with the current "
                "tools/model_convert\n",
                path, (unsigned)version, (unsigned)MODEL_VERSION);
        fclose(f);
        return false;
    }

    flags = read_u16le(&header_bytes[6]);
    if (flags & ~(uint16_t)MODEL_FLAG_KNOWN_MASK) {
        fprintf(stderr,
                "model_load: '%s': unknown flag bits set (0x%04x, known "
                "mask 0x%04x) -- file is from an incompatible converter\n",
                path, (unsigned)flags, (unsigned)MODEL_FLAG_KNOWN_MASK);
        fclose(f);
        return false;
    }
    if (!(flags & MODEL_FLAG_HAS_COLOR)) {
        fprintf(stderr,
                "model_load: '%s': HAS_COLOR flag not set -- version 1 "
                "requires every model to carry a color attribute (see "
                "model.h)\n", path);
        fclose(f);
        return false;
    }

    vertex_count = read_u32le(&header_bytes[8]);
    index_count = read_u32le(&header_bytes[12]);
    vertex_stride = read_u32le(&header_bytes[16]);
    vertex_data_offset = read_u32le(&header_bytes[20]);
    index_data_offset = read_u32le(&header_bytes[24]);
    reserved0 = read_u32le(&header_bytes[52]);
    reserved1 = read_u32le(&header_bytes[56]);
    reserved2 = read_u32le(&header_bytes[60]);

    if (reserved0 != 0 || reserved1 != 0 || reserved2 != 0) {
        fprintf(stderr,
                "model_load: '%s': reserved header fields are non-zero "
                "(0x%x, 0x%x, 0x%x) -- file is corrupt or from an "
                "incompatible writer\n",
                path, (unsigned)reserved0, (unsigned)reserved1, (unsigned)reserved2);
        fclose(f);
        return false;
    }

    if (vertex_count == 0 || vertex_count > MODEL_MAX_VERTEX_COUNT) {
        fprintf(stderr,
                "model_load: '%s': vertex_count %u is out of range "
                "(1..%u)\n", path, (unsigned)vertex_count, (unsigned)MODEL_MAX_VERTEX_COUNT);
        fclose(f);
        return false;
    }
    if (index_count == 0 || (index_count % 3) != 0) {
        fprintf(stderr,
                "model_load: '%s': index_count %u is not a positive "
                "multiple of 3 (not a whole number of triangles)\n",
                path, (unsigned)index_count);
        fclose(f);
        return false;
    }

    expected_stride = model_attribute_slot_count(flags) * MODEL_ATTR_BYTES;
    if (vertex_stride != expected_stride) {
        fprintf(stderr,
                "model_load: '%s': vertex_stride %u does not match what "
                "flags 0x%04x implies (%u) -- file is corrupt\n",
                path, (unsigned)vertex_stride, (unsigned)flags, (unsigned)expected_stride);
        fclose(f);
        return false;
    }

    vertex_bytes = (size_t)vertex_count * (size_t)vertex_stride;
    index_bytes = (size_t)index_count * sizeof(uint16_t);

    vbuf = model_alloc(vertex_bytes);
    if (!vbuf) {
        fprintf(stderr, "model_load: '%s': out of memory allocating %zu "
                "bytes of vertex data\n", path, vertex_bytes);
        fclose(f);
        return false;
    }
    ibuf = model_alloc(index_bytes);
    if (!ibuf) {
        fprintf(stderr, "model_load: '%s': out of memory allocating %zu "
                "bytes of index data\n", path, index_bytes);
        model_free_buffer(vbuf);
        fclose(f);
        return false;
    }

    if (fseek(f, (long)vertex_data_offset, SEEK_SET) != 0) {
        fprintf(stderr, "model_load: '%s': cannot seek to vertex data "
                "offset %u\n", path, (unsigned)vertex_data_offset);
        model_free_buffer(vbuf);
        model_free_buffer(ibuf);
        fclose(f);
        return false;
    }
    got = fread(vbuf, 1, vertex_bytes, f);
    if (got != vertex_bytes) {
        fprintf(stderr, "model_load: '%s': truncated vertex data "
                "(%zu/%zu bytes)\n", path, got, vertex_bytes);
        model_free_buffer(vbuf);
        model_free_buffer(ibuf);
        fclose(f);
        return false;
    }

    if (fseek(f, (long)index_data_offset, SEEK_SET) != 0) {
        fprintf(stderr, "model_load: '%s': cannot seek to index data "
                "offset %u\n", path, (unsigned)index_data_offset);
        model_free_buffer(vbuf);
        model_free_buffer(ibuf);
        fclose(f);
        return false;
    }
    got = fread(ibuf, 1, index_bytes, f);
    if (got != index_bytes) {
        fprintf(stderr, "model_load: '%s': truncated index data "
                "(%zu/%zu bytes)\n", path, got, index_bytes);
        model_free_buffer(vbuf);
        model_free_buffer(ibuf);
        fclose(f);
        return false;
    }

    fclose(f);

    out->vertex_buffer = vbuf;
    out->index_buffer = ibuf;
    out->vertex_count = vertex_count;
    out->index_count = index_count;
    out->vertex_stride = vertex_stride;
    out->flags = flags;
    out->bounds_min = vec3_make(
        read_f32le(&header_bytes[28]), read_f32le(&header_bytes[32]),
        read_f32le(&header_bytes[36]));
    out->bounds_max = vec3_make(
        read_f32le(&header_bytes[40]), read_f32le(&header_bytes[44]),
        read_f32le(&header_bytes[48]));
    out->loaded = true;
    return true;
}

void model_unload(Model *m) {
    if (!m) return;
    model_free_buffer(m->vertex_buffer);
    model_free_buffer(m->index_buffer);
    model_zero(m);
}

#ifdef __3DS__
void model_draw(const Model *m, const C3D_Mtx *modelview, int uloc_modelview) {
    C3D_AttrInfo *attr;
    C3D_BufInfo *buf;
    C3D_TexEnv *env;
    uint64_t permutation;
    int slot;

    if (!m || !m->loaded || !modelview || uloc_modelview < 0) return;

    C3D_FVUnifMtx4x4(GPU_VERTEX_SHADER, uloc_modelview, modelview);

    /* Rebind this draw's own full pipeline state unconditionally, rather
     * than assuming whatever ran earlier in the frame left it correct --
     * same convention debugdraw.c's and renderer.c's own per-draw setup
     * already use (see their header comments). */
    attr = C3D_GetAttrInfo();
    AttrInfo_Init(attr);
    slot = 0;
    AttrInfo_AddLoader(attr, slot++, GPU_FLOAT, MODEL_ATTR_FLOATS); /* position */
    AttrInfo_AddLoader(attr, slot++, GPU_FLOAT, MODEL_ATTR_FLOATS); /* color */
    if (m->flags & MODEL_FLAG_HAS_NORMAL)
        AttrInfo_AddLoader(attr, slot++, GPU_FLOAT, MODEL_ATTR_FLOATS);
    if (m->flags & MODEL_FLAG_HAS_UV)
        AttrInfo_AddLoader(attr, slot++, GPU_FLOAT, MODEL_ATTR_FLOATS);

    /* Permutation nibbles (LSB-first) give the attribute id loaded at each
     * position, in the same order just added above -- 0,1,2,... since
     * this format's slots are always assigned in that fixed order. */
    permutation = 0;
    {
        int i;
        for (i = 0; i < slot; i++) permutation |= ((uint64_t)i) << (4 * i);
    }

    buf = C3D_GetBufInfo();
    BufInfo_Init(buf);
    BufInfo_Add(buf, m->vertex_buffer, m->vertex_stride, (int)slot, permutation);

    /* No texture stage configured -- see model.h's "TEXTURES: NOT HANDLED"
     * note. TEV 0 just passes the vertex color (dirt2.v.pica's outclr)
     * through untouched, same as debugdraw.c/renderer.c's own TexEnv
     * setup. */
    env = C3D_GetTexEnv(0);
    C3D_TexEnvInit(env);
    C3D_TexEnvSrc(env, C3D_Both, GPU_PRIMARY_COLOR, GPU_PRIMARY_COLOR, GPU_PRIMARY_COLOR);
    C3D_TexEnvFunc(env, C3D_Both, GPU_REPLACE);

    GSPGPU_FlushDataCache(m->vertex_buffer, m->vertex_count * m->vertex_stride);
    GSPGPU_FlushDataCache(m->index_buffer, m->index_count * (uint32_t)sizeof(uint16_t));

    C3D_DrawElements(GPU_TRIANGLES, (int)m->index_count, C3D_UNSIGNED_SHORT, m->index_buffer);
}
#endif
