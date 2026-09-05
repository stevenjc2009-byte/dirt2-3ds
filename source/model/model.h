/*---------------------------------------------------------------------------------
 * model.h -- runtime loader/drawer for the DiRT2 model binary format
 * (".d2m"), produced offline by tools/model_convert/convert.py from an
 * artist-authored OBJ export. See that tool's obj_parser.py header comment
 * for why OBJ (not glTF) is this pipeline's interchange format, and
 * mesh_builder.py for exactly how a mesh is triangulated/welded before it
 * ever reaches this binary layout.
 *
 * OWNER: model (this is a brand-new module; nothing else in the project
 * references it yet -- see this file's integration notes at the bottom for
 * the two small changes another pass needs to make elsewhere to actually
 * draw a loaded Model on screen).
 *
 * ===========================================================================
 * BINARY FORMAT LAYOUT (mirrors tools/model_convert/model_format.py
 * byte-for-byte -- if either changes, change both together).
 *
 * Whole file: little-endian throughout. The 3DS's ARM11 runs little-endian
 * and this converter runs on an x86/x64 PC, also little-endian, so no byte
 * swapping is ever needed anywhere in this pipeline -- worth stating
 * explicitly since it is exactly the kind of assumption that quietly stops
 * being true if this tool is ever run cross-endian.
 *
 * HEADER (64 bytes, at file offset 0):
 *   offset  size  type      field                description
 *   ------  ----  --------  -------------------  --------------------------
 *   0       4     char[4]   magic                "D2MB" ("DiRT2 Model
 *                                                 Binary"). Not NUL-terminated,
 *                                                 not a C string -- compared
 *                                                 as 4 raw bytes.
 *   4       2     u16       version              format version. This file
 *                                                 implements MODEL_VERSION
 *                                                 below; model_load rejects
 *                                                 anything else outright so a
 *                                                 stale asset from an older/
 *                                                 newer converter fails loudly
 *                                                 at load time instead of
 *                                                 being misinterpreted.
 *   6       2     u16       flags                bit 0 MODEL_FLAG_HAS_NORMAL
 *                                                 bit 1 MODEL_FLAG_HAS_UV
 *                                                 bit 2 MODEL_FLAG_HAS_COLOR
 *                                                   (version 1 ALWAYS sets
 *                                                   this -- see below)
 *                                                 bits 3-15 reserved, must be
 *                                                 0 (model_load rejects any
 *                                                 set reserved bit).
 *   8       4     u32       vertex_count         number of unique vertices
 *                                                 after welding.
 *   12      4     u32       index_count          number of indices
 *                                                 (3 * triangle_count).
 *   16      4     u32       vertex_stride        bytes per vertex. Always
 *                                                 recomputable from `flags`
 *                                                 (see MODEL_ATTR_BYTES
 *                                                 below) -- stored anyway so
 *                                                 model_load can cross-check
 *                                                 it against `flags` and
 *                                                 catch a corrupt file that
 *                                                 has consistent-looking
 *                                                 fields individually but
 *                                                 not together.
 *   20      4     u32       vertex_data_offset   byte offset from file start
 *                                                 to the first vertex.
 *   24      4     u32       index_data_offset    byte offset from file start
 *                                                 to the first index.
 *   28      4     f32       bounds_min.x
 *   32      4     f32       bounds_min.y
 *   36      4     f32       bounds_min.z         local-space AABB min, for
 *                                                 future culling -- computed
 *                                                 by the converter, not used
 *                                                 by model_load/model_draw
 *                                                 today (stored in the Model
 *                                                 struct for whoever wants
 *                                                 it).
 *   40      4     f32       bounds_max.x
 *   44      4     f32       bounds_max.y
 *   48      4     f32       bounds_max.z         local-space AABB max.
 *   52      4     u32       reserved0            must be 0.
 *   56      4     u32       reserved1            must be 0.
 *   60      4     u32       reserved2            must be 0.
 *
 * VERTEX DATA (at vertex_data_offset, vertex_count * vertex_stride bytes):
 *   Interleaved. EVERY attribute -- position, color, normal, uv -- is
 *   stored as exactly 4 x f32 (16 bytes), never 3 or 2, even when the
 *   logical quantity has fewer components (unused trailing components are
 *   zero-filled, except position's w which is always 1.0). This is
 *   deliberate, not wasteful-by-accident:
 *
 *     MEASURED FACT (this machine's prior 3DS work, not a guess): a
 *     3-component vertex attribute declared BEFORE a 4-component one
 *     FREEZES REAL PICA200 HARDWARE. Making every attribute uniformly
 *     4-component sidesteps the ordering hazard entirely -- there is no
 *     "3-before-4" for the loader or a future format change to ever get
 *     wrong, because there is no 3-wide attribute in this format, period.
 *     This exactly matches this project's existing convention: see
 *     source/render/dirt2.v.pica's header comment and
 *     source/render/debugdraw.c's DebugVertex struct, both of which made
 *     the same choice for the same reason before this module existed.
 *
 *   Attributes are packed CONTIGUOUSLY in this fixed priority order, each
 *   included only if its flag is set (position and color are never
 *   optional -- see MODEL_FLAG_HAS_COLOR below):
 *     1. position  (always)       : x, y, z, 1.0
 *     2. color     (always, v1)   : r, g, b, a  (see synthesis note below)
 *     3. normal    (if HAS_NORMAL): nx, ny, nz, 0.0
 *     4. uv        (if HAS_UV)    : u, v, 0.0, 0.0
 *   So a model with only position+color has an 8-float (32-byte) stride;
 *   adding normal makes it 12 floats (48 bytes); adding uv too makes it 16
 *   floats (64 bytes). model_attribute_slot_count() below computes this
 *   from `flags` the same way tools/model_convert/model_format.py's
 *   attribute_order() does -- keep the two in sync.
 *
 *   WHY COLOR IS NEVER OPTIONAL IN VERSION 1, AND WHY IT IS SYNTHESIZED:
 *     source/render/dirt2.v.pica -- the ONE vertex shader this whole
 *     process owns (see renderer.c's header comment on why there is
 *     exactly one shaderProgram_s for the process's entire lifetime) --
 *     hard-expects v0 = position, v1 = color, full stop; it has no
 *     conditional attribute logic and no other input. So every model this
 *     format describes must supply a color attribute at slot 1 or it will
 *     render with whatever garbage happens to be in that GPU input
 *     register. tools/model_convert/convert.py therefore ALWAYS emits a
 *     color attribute, synthesized as flat white (1,1,1,1) since OBJ has
 *     no standard per-vertex color and Blender's stock exporter does not
 *     write one. Real per-vertex tinting, or a lit shader that consumes
 *     the normal attribute, is future work once a second/richer shader
 *     exists -- this module does not and must not create one (see OWNERSHIP
 *     below).
 *
 *   TEXTURES: NOT HANDLED. UV data, when MODEL_FLAG_HAS_UV is set, is
 *     carried in the vertex buffer and nothing more. model_draw below does
 *     not bind a texture unit, does not configure a texture-sampling TEV
 *     stage, and this module does not touch GPU_TEXTURE0 at all. Loading,
 *     swizzling, or power-of-two validation of any texture image is
 *     entirely unimplemented -- UV is future-proofing for when a textured
 *     shader exists, not a working texture path today.
 *
 * INDEX DATA (at index_data_offset, index_count * 2 bytes):
 *   u16 (GPU_UNSIGNED_SHORT / C3D_UNSIGNED_SHORT), little-endian, one
 *   triangle-list index per entry (GPU_TRIANGLES, not a strip or fan).
 *   This caps a single model at 65535 unique vertices -- see
 *   MODEL_MAX_VERTEX_COUNT below; tools/model_convert's converter refuses
 *   to write a file that would exceed it, loudly, rather than silently
 *   truncating or wrapping indices.
 *
 * NO PER-VERTEX FIXUP AT LOAD TIME: model_load reads the 64-byte header
 * field-by-field (cheap, done once), but the vertex and index DATA is
 * copied as raw bytes straight into a linear-memory buffer and handed to
 * citro3d's BufInfo/DrawElements as-is -- there is no per-vertex loop
 * re-encoding floats or swapping bytes at load time, by construction (see
 * "whole file: little-endian" above).
 * ===========================================================================
 *
 * OWNERSHIP / THE ONE SHADER RULE.
 *   This module NEVER calls shaderProgramInit, never builds a DVLB, and
 *   never binds a program. renderer.c alone owns the process's single
 *   shaderProgram_s (see renderer.c's header comment on why creating a
 *   second one anywhere is a use-after-free that hard-crashes real
 *   hardware). model_draw instead takes an ALREADY-RESOLVED uniform
 *   location and an already-computed matrix to upload to it -- it assumes
 *   the caller's shader program is already bound (renderer.c binds its one
 *   program once at init and never unbinds it) and only touches transient
 *   per-draw GPU state (AttrInfo, BufInfo, TexEnv), the same pattern
 *   source/render/debugdraw.c and renderer.c's own renderer_draw_ground/
 *   renderer_draw_vehicle already use for their own per-draw geometry.
 *
 * INTEGRATION NOTES (not yet wired up -- reported, not applied; see the
 * task's "REPORT the exact change" rule. Nothing below is required for
 * this module to compile or for the round-trip/unit tests to pass; it is
 * required before a loaded Model actually appears on screen):
 *   1. source/main.c needs romfsInit() called once after gfxInitDefault()
 *      (and romfsExit() at shutdown) before any model_load call -- no
 *      romfs mount exists anywhere in this codebase yet. Without it,
 *      model_load's fopen("romfs:/...") calls fail (which model_load DOES
 *      already handle: it reports "cannot open" and returns false, it just
 *      never succeeds until romfsInit runs).
 *   2. renderer.c's `view` matrix (built as a LOCAL C3D_Mtx inside
 *      renderer_frame_begin) is not currently retained anywhere a later
 *      model_draw call could reach. Drawing a Model needs
 *      view-matrix * model-matrix uploaded to renderer.c's existing
 *      (currently static, unexported) s_uloc_modelview location. The
 *      smallest change: cache that `view` into a new static C3D_Mtx in
 *      renderer.c at the end of renderer_frame_begin, then add a small
 *      renderer_draw_model(const Model*, Vec3 position, Quat orientation)
 *      wrapper in renderer.c that builds a model matrix from
 *      position/orientation, multiplies it by the cached view, and calls
 *      model_draw(model, &combined, s_uloc_modelview). This keeps
 *      dirt2.v.pica and renderer.h's existing public shape untouched.
 *---------------------------------------------------------------------------------*/
#ifndef DIRT2_MODEL_MODEL_H
#define DIRT2_MODEL_MODEL_H

#include "core/types.h"
#include <stdint.h>

#ifdef __3DS__
#include <citro3d.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define MODEL_MAGIC0 'D'
#define MODEL_MAGIC1 '2'
#define MODEL_MAGIC2 'M'
#define MODEL_MAGIC3 'B'

#define MODEL_VERSION 1u

#define MODEL_FLAG_HAS_NORMAL (1u << 0)
#define MODEL_FLAG_HAS_UV     (1u << 1)
#define MODEL_FLAG_HAS_COLOR  (1u << 2) /* version 1 always sets this, see above */
#define MODEL_FLAG_KNOWN_MASK (MODEL_FLAG_HAS_NORMAL | MODEL_FLAG_HAS_UV | MODEL_FLAG_HAS_COLOR)

#define MODEL_HEADER_SIZE 64u

/* Every attribute is 4 x f32 -- see header comment on why. u32/u16 are
 * libctru typedefs (only in scope under __3DS__, via citro3d.h's own
 * includes); this header must stay includable without __3DS__ (see
 * debugdraw.h's precedent for the same reason), so plain uint32_t is used
 * throughout instead. */
#define MODEL_ATTR_FLOATS 4u
#define MODEL_ATTR_BYTES  (MODEL_ATTR_FLOATS * (uint32_t)sizeof(float)) /* 16 */

/* u16 index buffer ceiling -- see header comment. */
#define MODEL_MAX_VERTEX_COUNT 0xFFFFu

/* Runtime handle for one loaded model. Plain struct, not opaque -- every
 * field here is either a plain scalar or a pointer this module itself
 * allocated and frees in model_unload, so there is nothing to hide from a
 * caller that just wants to read e.g. bounds_min/bounds_max. */
typedef struct Model {
    void    *vertex_buffer;  /* interleaved vertex data, see layout above.
                               * linearAlloc'd (3DS) / malloc'd (non-3DS) by
                               * model_load; owned by this Model. */
    void    *index_buffer;   /* u16 triangle-list indices. Same allocator/
                               * ownership as vertex_buffer. */
    uint32_t vertex_count;
    uint32_t index_count;
    uint32_t vertex_stride;  /* bytes; see MODEL_ATTR_BYTES-derived layout */
    uint16_t flags;          /* MODEL_FLAG_* bits present in this model */
    Vec3     bounds_min;
    Vec3     bounds_max;
    bool     loaded;         /* false until model_load fully succeeds; a
                               * Model that failed to load is left fully
                               * zeroed (see model_load), never half-filled */
} Model;

/* Loads a converted .d2m file from `path` (e.g. "romfs:/models/car.d2m" --
 * this module does not hardcode the "romfs:/" prefix; that is the caller's
 * convention to apply, and the ordinary C stdio path works unchanged for a
 * plain filesystem path too, which is how this module's own tests exercise
 * it without romfs or citro3d).
 *
 * Validates the magic and version before touching anything else, and fails
 * (returns false, `*out` left fully zeroed) loudly and specifically -- see
 * model.c -- on: missing file, truncated header, bad magic, unsupported
 * version, unknown/reserved flag bits set, a vertex_stride inconsistent
 * with flags, an out-of-range vertex_count (over MODEL_MAX_VERTEX_COUNT),
 * or a truncated vertex/index data section. Never returns true with a
 * partially-populated Model. */
bool model_load(Model *out, const char *path);

/* Frees everything model_load allocated and zeroes *m. Safe to call on a
 * Model that failed to load (loaded == false) or was already unloaded --
 * both are no-ops. */
void model_unload(Model *m);

/* Returns the number of interleaved attribute slots this model's `flags`
 * imply (2..4 -- see the layout's fixed priority order above). Shared
 * logic between model_load's stride cross-check and model_draw's AttrInfo
 * setup so the two can never disagree about how many attributes a given
 * flags value means. */
uint32_t model_attribute_slot_count(uint16_t flags);

#ifdef __3DS__
/* Draws `m` with citro3d. Uploads `*modelview` (already the caller's
 * combined view * model transform -- see this file's OWNERSHIP section on
 * why this module never computes or owns a view matrix itself) to
 * `uloc_modelview`, an already-resolved uniform location from the
 * caller's own (single, process-wide) shaderProgram_s. Rebinds this
 * draw's own AttrInfo/BufInfo/TexEnv state unconditionally before issuing
 * one C3D_DrawElements call, matching the convention already established
 * by source/render/debugdraw.c and renderer.c's own per-draw setup (never
 * assumes an earlier draw call in the same frame left compatible state).
 * No-op if `m` is not loaded, `modelview` is NULL, or uloc_modelview < 0. */
void model_draw(const Model *m, const C3D_Mtx *modelview, int uloc_modelview);
#endif

#ifdef __cplusplus
}
#endif

#endif /* DIRT2_MODEL_MODEL_H */
