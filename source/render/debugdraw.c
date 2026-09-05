/*---------------------------------------------------------------------------------
 * debugdraw.c -- immediate-mode world-space lines/wire boxes/points (citro3d)
 * and screen-space text (citro2d). See debugdraw.h for the contract.
 *
 * NO LINE PRIMITIVE ON REAL PICA200 HARDWARE.
 *   GPU_Primitive_t (libctru's 3ds/gpu/enums.h) only offers GPU_TRIANGLES,
 *   GPU_TRIANGLE_STRIP, GPU_TRIANGLE_FAN and GPU_GEOMETRY_PRIM -- there is no
 *   line or point primitive mode, and neither libctru nor citro3d expose a
 *   wireframe fill mode. `3ds-project-folder/mc`'s highlight.c documents
 *   hitting the exact same wall for its selection-cage overlay. So every
 *   "line" here is two triangles (a flat quad extruded from the segment),
 *   and every "point" is three such quads forming a 3-axis cross -- see
 *   debugdraw_line's comment for the extrusion and its known limitation.
 *
 * BATCHING.
 *   All primitives queued between debugdraw_frame_begin/end land in ONE
 *   linear-memory vertex buffer (allocated once in debugdraw_init, cursor
 *   reset per frame, never reallocated -- reallocating per frame would
 *   fragment the linear heap, which cannot be defragmented, the same reason
 *   both sibling projects' per-frame batchers size once up front) and are
 *   drawn with exactly one C3D_DrawArrays call in debugdraw_frame_end, not
 *   one draw call per debugdraw_line/wire_box/point call. See
 *   MAX_DEBUG_VERTICES below for the sizing reasoning.
 *
 * Every citro3d/citro2d-specific line in this file is behind #ifdef __3DS__
 * so this still compiles on the WSL host build: the non-3DS bodies are
 * deliberate no-ops, not stubs left behind by accident.
 *---------------------------------------------------------------------------------*/
#include "render/debugdraw.h"
#include "render/renderer.h"
#include "core/vecmath.h"
#include <stdarg.h>
#include <stdio.h>
#include <math.h>

#ifdef __3DS__
#include <3ds.h>
#include <citro3d.h>
#include <citro2d.h>
/* No shader include here -- the shaderProgram_s itself is owned entirely by
 * renderer.c (see that file); debugdraw.c only ever draws with whatever
 * program renderer.c already bound, via the AttrInfo/BufInfo/TexEnv state
 * it sets right before its own C3D_DrawArrays call below. */
#endif

/* ---------------------------------------------------------------------
 * MAX_DEBUG_VERTICES sizing.
 *
 * Phase 1's actual debug-view load, per frame, once suspension/tyre debug
 * views land: 4 wheels x (suspension raycast + contact normal + tyre
 * slip-force vector) = 12 lines, one chassis wire box = 12 edges = 12
 * lines, a handful of point markers at 3 line-segments each (say 8 points
 * = 24 line-equivalents) -- roughly 48 lines/frame in realistic use, call
 * it 60 with room for whatever the suspension/tyre/vehicle agents add next.
 *
 * Each line becomes one 2-triangle quad = 6 vertices (see debugdraw_line),
 * so 60 lines/frame is 360 vertices. MAX_DEBUG_VERTICES is set to 4096 --
 * roughly 11x that estimate (682 line-equivalents) -- so a silently-dropped
 * primitive (debugdraw_line returns early past this budget, see below)
 * reads as "someone added a genuinely large amount of debug geometry in one
 * frame", not as an everyday occurrence, while the buffer itself
 * (4096 * sizeof(DebugVertex) = 4096 * 32 bytes = 128 KB, allocated ONCE in
 * linear memory at debugdraw_init, never per-frame) stays a small slice of
 * Old 3DS's 6 MB VRAM / roughly 80 MB app RAM budget.
 * ------------------------------------------------------------------- */
#define MAX_DEBUG_VERTICES 4096
#define VERTS_PER_LINE 6

/* Text queue sizing: Phase 1's per-wheel HUD readout is roughly 4 wheels x
 * 3 lines (compression, slip ratio, slip angle) plus speed/frame-time/etc,
 * call it 16 calls/frame in practice; 64 is 4x headroom. */
#define MAX_DEBUG_TEXT_CALLS 64

/* Half-width, in world units, of the flat ribbon debugdraw_line extrudes a
 * segment into. Small enough to read as "a line" at Phase 1's car-and-test-
 * track scale (see world/testground.h) without being so thin it vanishes
 * at a distance on a 400x240 screen. */
#define DEBUG_LINE_HALF_WIDTH 0.01f

#ifdef __3DS__
/* Both attributes 4-component float, matching dirt2.v.pica's v0/v1 -- see
 * that file's header comment on why (avoids the 3-before-4 attribute
 * ordering hazard entirely rather than relying on getting the order right). */
typedef struct DebugVertex {
    float pos[4]; /* x, y, z, 1.0 */
    float col[4]; /* r, g, b, a, already 0..1 */
} DebugVertex;

static DebugVertex *s_verts = NULL;
static int s_vert_count = 0;
static bool s_ready = false;

typedef struct QueuedText {
    C2D_Text text;
    float x, y;
    uint32_t rgba;
} QueuedText;

static C2D_TextBuf s_text_buf = NULL;
static QueuedText s_texts[MAX_DEBUG_TEXT_CALLS];
static int s_text_count = 0;
#endif

void debugdraw_init(void) {
#ifdef __3DS__
    if (s_ready) return;

    s_verts = (DebugVertex *)linearAlloc(MAX_DEBUG_VERTICES * sizeof(DebugVertex));
    s_text_buf = C2D_TextBufNew(4096);
    s_vert_count = 0;
    s_text_count = 0;

    s_ready = (s_verts != NULL) && (s_text_buf != NULL);
#endif
}

void debugdraw_shutdown(void) {
#ifdef __3DS__
    if (!s_ready) return;

    if (s_text_buf) {
        C2D_TextBufDelete(s_text_buf);
        s_text_buf = NULL;
    }
    if (s_verts) {
        linearFree(s_verts);
        s_verts = NULL;
    }
    s_ready = false;
#endif
}

void debugdraw_frame_begin(void) {
#ifdef __3DS__
    if (!s_ready) return;
    s_vert_count = 0;
    s_text_count = 0;
    C2D_TextBufClear(s_text_buf);
#endif
}

void debugdraw_frame_end(void) {
#ifdef __3DS__
    if (!s_ready) return;

    if (s_vert_count > 0) {
        /* Rebind this draw's own full pipeline state unconditionally
         * (attributes, buffer, TEV) rather than assuming whatever ran
         * earlier in the frame left it correct -- matches the convention
         * both sibling projects use for their own per-frame batchers. */
        C3D_AttrInfo *attr = C3D_GetAttrInfo();
        AttrInfo_Init(attr);
        AttrInfo_AddLoader(attr, 0, GPU_FLOAT, 4); /* position */
        AttrInfo_AddLoader(attr, 1, GPU_FLOAT, 4); /* colour */

        C3D_BufInfo *buf = C3D_GetBufInfo();
        BufInfo_Init(buf);
        BufInfo_Add(buf, s_verts, sizeof(DebugVertex), 2, 0x10);

        C3D_TexEnv *env = C3D_GetTexEnv(0);
        C3D_TexEnvInit(env);
        C3D_TexEnvSrc(env, C3D_Both, GPU_PRIMARY_COLOR, GPU_PRIMARY_COLOR, GPU_PRIMARY_COLOR);
        C3D_TexEnvFunc(env, C3D_Both, GPU_REPLACE);

        /* Flush only the byte range actually written this frame, not the
         * whole 128 KB buffer -- needless cache-flush cost otherwise. */
        GSPGPU_FlushDataCache(s_verts, (u32)(s_vert_count * sizeof(DebugVertex)));
        C3D_DrawArrays(GPU_TRIANGLES, 0, s_vert_count);
    }

    if (s_text_count > 0) {
        C3D_RenderTarget *target = renderer_get_target();
        if (target) {
            int i;
            /* C2D_Prepare() clobbers citro3d's attribute/buffer/TEV/depth
             * state (confirmed against model-making's main.c) -- safe here
             * because this is the LAST thing drawn this frame; the next
             * frame's debugdraw_frame_end call above re-establishes its own
             * full citro3d pipeline unconditionally before drawing lines. */
            C2D_Prepare();
            C2D_SceneBegin(target);
            for (i = 0; i < s_text_count; i++) {
                QueuedText *q = &s_texts[i];
                /* debugdraw.h's rgba is 0xRRGGBBAA; citro2d's C2D_Color32
                 * packs the OPPOSITE byte order (alpha highest) -- convert
                 * via their own macro rather than guessing shifts. */
                u8 r = (u8)((q->rgba >> 24) & 0xFF);
                u8 g = (u8)((q->rgba >> 16) & 0xFF);
                u8 b = (u8)((q->rgba >> 8) & 0xFF);
                u8 a = (u8)(q->rgba & 0xFF);
                C2D_DrawText(&q->text, C2D_WithColor, q->x, q->y, 0.5f, 0.5f, 0.5f,
                             C2D_Color32(r, g, b, a));
            }
            C2D_Flush();
        }
    }
#endif
}

void debugdraw_line(Vec3 from, Vec3 to, uint32_t rgba) {
#ifdef __3DS__
    if (!s_ready) return;
    if (s_vert_count + VERTS_PER_LINE > MAX_DEBUG_VERTICES) return; /* over
        budget for this frame -- see MAX_DEBUG_VERTICES' sizing comment; a
        debug overlay that silently stops drawing past its budget is far
        better than one that overruns its buffer. */

    Vec3 dir = vec3_sub(to, from);
    f32 len = vec3_length(dir);
    if (len < 1e-6f) return; /* degenerate: no direction to extrude along */
    dir = vec3_scale(dir, 1.0f / len);

    /* Extrude a flat world-space ribbon perpendicular to the segment,
     * rather than a true camera-facing billboard: debugdraw.c is
     * deliberately decoupled from the Camera (renderer.c alone owns the
     * view/projection uniforms; debugdraw.c only ever sees world-space
     * points, see this file's header and debugdraw.h's OWNER note), and
     * the PICA200 has no native line primitive at all (see this file's
     * header). A ribbon perpendicular to a fixed reference axis is simple,
     * deterministic and good enough for a tool whose entire job is showing
     * magnitude/direction, not a pixel-perfect line. Known limitation: a
     * line viewed exactly edge-on (parallel to the current view direction)
     * can look thinner than others, or briefly degenerate -- acceptable for
     * a debug overlay, would not be for real vehicle/world geometry. */
    Vec3 reference = (fabsf(dir.y) > 0.99f) ? vec3_make(1.0f, 0.0f, 0.0f)
                                             : vec3_make(0.0f, 1.0f, 0.0f);
    Vec3 side = vec3_normalize(vec3_cross(dir, reference));
    Vec3 offset = vec3_scale(side, DEBUG_LINE_HALF_WIDTH);

    Vec3 a0 = vec3_sub(from, offset);
    Vec3 a1 = vec3_add(from, offset);
    Vec3 b0 = vec3_sub(to, offset);
    Vec3 b1 = vec3_add(to, offset);

    float r = ((rgba >> 24) & 0xFF) / 255.0f;
    float g = ((rgba >> 16) & 0xFF) / 255.0f;
    float b = ((rgba >> 8) & 0xFF) / 255.0f;
    float al = (rgba & 0xFF) / 255.0f;

    DebugVertex va = { { a0.x, a0.y, a0.z, 1.0f }, { r, g, b, al } };
    DebugVertex vb = { { a1.x, a1.y, a1.z, 1.0f }, { r, g, b, al } };
    DebugVertex vc = { { b0.x, b0.y, b0.z, 1.0f }, { r, g, b, al } };
    DebugVertex vd = { { b1.x, b1.y, b1.z, 1.0f }, { r, g, b, al } };

    /* Two triangles covering the quad a0-a1-b1-b0. Winding does not matter
     * -- renderer.c sets GPU_CULL_NONE for exactly this reason. */
    DebugVertex *v = &s_verts[s_vert_count];
    v[0] = va; v[1] = vb; v[2] = vc;
    v[3] = vb; v[4] = vd; v[5] = vc;
    s_vert_count += VERTS_PER_LINE;
#else
    (void)from; (void)to; (void)rgba;
#endif
}

void debugdraw_wire_box(Vec3 center, Vec3 half_extents, uint32_t rgba) {
    Vec3 c[8];
    int i;
    for (i = 0; i < 8; i++) {
        c[i] = vec3_make(
            center.x + ((i & 1) ? half_extents.x : -half_extents.x),
            center.y + ((i & 2) ? half_extents.y : -half_extents.y),
            center.z + ((i & 4) ? half_extents.z : -half_extents.z));
    }
    /* 12 edges: two vertices connect iff they differ in exactly one of the
     * x/y/z bits used to build c[] above. */
    debugdraw_line(c[0], c[1], rgba); /* vary x */
    debugdraw_line(c[2], c[3], rgba);
    debugdraw_line(c[4], c[5], rgba);
    debugdraw_line(c[6], c[7], rgba);

    debugdraw_line(c[0], c[2], rgba); /* vary y */
    debugdraw_line(c[1], c[3], rgba);
    debugdraw_line(c[4], c[6], rgba);
    debugdraw_line(c[5], c[7], rgba);

    debugdraw_line(c[0], c[4], rgba); /* vary z */
    debugdraw_line(c[1], c[5], rgba);
    debugdraw_line(c[2], c[6], rgba);
    debugdraw_line(c[3], c[7], rgba);
}

void debugdraw_point(Vec3 position, f32 size, uint32_t rgba) {
    /* A 3-axis cross rather than a sphere -- three debugdraw_line calls,
     * cheap and immediately readable as "a point" from any angle despite
     * debugdraw_line's ribbons not being true billboards. `size` is the
     * half-length of each arm, matching debugdraw_wire_box's
     * half_extents convention. */
    Vec3 dx = vec3_make(size, 0.0f, 0.0f);
    Vec3 dy = vec3_make(0.0f, size, 0.0f);
    Vec3 dz = vec3_make(0.0f, 0.0f, size);
    debugdraw_line(vec3_sub(position, dx), vec3_add(position, dx), rgba);
    debugdraw_line(vec3_sub(position, dy), vec3_add(position, dy), rgba);
    debugdraw_line(vec3_sub(position, dz), vec3_add(position, dz), rgba);
}

void debugdraw_text(f32 screen_x, f32 screen_y, uint32_t rgba, const char *fmt, ...) {
    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

#ifdef __3DS__
    if (!s_ready) return;
    if (s_text_count >= MAX_DEBUG_TEXT_CALLS) return; /* see sizing comment
        above -- silently dropped past budget, same policy as debugdraw_line. */

    QueuedText *q = &s_texts[s_text_count];
    C2D_TextParse(&q->text, s_text_buf, buf);
    C2D_TextOptimize(&q->text);
    q->x = screen_x;
    q->y = screen_y;
    q->rgba = rgba;
    s_text_count++;
#else
    (void)screen_x; (void)screen_y; (void)rgba; (void)buf;
#endif
}
