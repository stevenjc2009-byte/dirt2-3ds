/*---------------------------------------------------------------------------------
 * hud.c -- bottom-screen instrument panel. See hud.h for the contract.
 *
 * THE RENDER TARGET IS OWNED HERE, NOT BY renderer.c. renderer.c creates and
 * owns the TOP screen target and nothing else; this file creates the BOTTOM
 * one. Splitting them that way means the bottom screen can be dropped or
 * replaced without touching the 3D path at all, and neither file has to know
 * how the other's screen is configured.
 *
 * DRAWING ORDER, AND WHY IT MATTERS. hud_draw runs inside renderer.c's
 * C3D_FrameBegin/C3D_FrameEnd bracket, after every top-screen draw. It calls
 * C2D_SceneBegin, which internally does C3D_FrameDrawOn(bottom) -- so from
 * that point on the frame is drawing to the bottom screen and STAYS there.
 * Anything that wanted the top screen after this would silently land here.
 * That is why hud.h says to call this last, and why main.c does.
 *
 * The clear uses C2D_TargetClear rather than C3D_RenderTargetClear: the top
 * screen's clear happens in renderer_frame_begin, before any drawing, where a
 * plain C3D_RenderTargetClear is correct. This one happens mid-frame, after
 * the top screen has already been drawn, and C2D_TargetClear is the call that
 * splits the command buffer first so the clear does not eat the drawing that
 * came before it in the same frame.
 *
 * NO C2D_Prepare CALL HERE, DELIBERATELY -- see hud_draw's own comment.
 *
 * TEXT BUFFER SIZING. One C2D_TextBuf of 1024 glyphs, allocated once in
 * hud_init, cleared (not reallocated) at the top of every hud_draw. The panel
 * draws roughly 25 short strings a frame, well under 400 glyphs; 1024 is
 * comfortable headroom without being a meaningful slice of the Old 3DS's
 * memory budget. Same "allocate once, reset the cursor per frame" policy as
 * debugdraw.c's vertex buffer and for the same reason: the linear heap cannot
 * be defragmented, so per-frame allocation is not an option.
 *
 * Every citro2d/citro3d line is behind #ifdef __3DS__ so this compiles on the
 * WSL host build as deliberate no-ops.
 *---------------------------------------------------------------------------------*/
#include "render/hud.h"

#include <stdio.h>
#include <stdarg.h>
#include <math.h>

#ifdef __3DS__
#include <3ds.h>
#include <citro3d.h>
#include <citro2d.h>

/* Same transfer flags renderer.c uses for the top screen -- see that file. */
#define HUD_DISPLAY_TRANSFER_FLAGS \
    (GX_TRANSFER_FLIP_VERT(0) | GX_TRANSFER_OUT_TILED(0) | GX_TRANSFER_RAW_COPY(0) | \
     GX_TRANSFER_IN_FORMAT(GX_TRANSFER_FMT_RGBA8) | GX_TRANSFER_OUT_FORMAT(GX_TRANSFER_FMT_RGB8) | \
     GX_TRANSFER_SCALING(GX_TRANSFER_SCALE_NO))

/* The bottom panel is physically 320x240. Same convention as renderer.c's
 * TOP_SCREEN_WIDTH/HEIGHT comment: C3D_RenderTargetCreate takes
 * (height, width), so the argument order below looks transposed and is not. */
#define BOTTOM_SCREEN_WIDTH  320
#define BOTTOM_SCREEN_HEIGHT 240

#define HUD_TEXT_BUF_GLYPHS 1024

static C3D_RenderTarget *s_target = NULL;
static C2D_TextBuf s_text_buf = NULL;
static bool s_ready = false;

/* ---- palette ----------------------------------------------------------
 * Dark panel, warm readout. Deliberately low-contrast backgrounds and high-
 * contrast text: the bottom screen is glanced at, not read, so the numbers
 * have to pop off it without the panel itself drawing the eye away from the
 * road on the top screen. */
#define CLR_PANEL     C2D_Color32(0x14, 0x16, 0x1A, 0xFF)
#define CLR_BAND      C2D_Color32(0x1F, 0x23, 0x2A, 0xFF)
#define CLR_SLOT      C2D_Color32(0x2A, 0x2F, 0x38, 0xFF)
#define CLR_TEXT      C2D_Color32(0xE8, 0xE8, 0xE2, 0xFF)
#define CLR_DIM       C2D_Color32(0x8A, 0x92, 0x9E, 0xFF)
#define CLR_ACCENT    C2D_Color32(0xF0, 0xC4, 0x20, 0xFF)  /* the DiRT2 yellow */
#define CLR_GO        C2D_Color32(0x4C, 0xD1, 0x64, 0xFF)
#define CLR_STOP      C2D_Color32(0xE0, 0x4C, 0x3C, 0xFF)
#define CLR_AIR       C2D_Color32(0x50, 0x58, 0x64, 0xFF)

/* Draws one string. The C2D_Text is a stack local: C2D_TextParse copies the
 * glyph data into s_text_buf, and C2D_DrawText only needs the handle to
 * survive until the draw call returns, so nothing here outlives the frame. */
static void hud_text(f32 x, f32 y, f32 scale, u32 colour, const char *fmt, ...) {
    char line[96];
    C2D_Text text;
    va_list args;

    va_start(args, fmt);
    vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);

    C2D_TextParse(&text, s_text_buf, line);
    C2D_TextOptimize(&text);
    C2D_DrawText(&text, C2D_WithColor, x, y, 0.5f, scale, scale, colour);
}

/* Seconds -> "M:SS.hh", the format every racing game uses, because "83.4 s" is
 * not a lap time anyone can compare against a memory of one.
 *
 * A negative value is lap.h's LAP_NO_TIME sentinel meaning "no lap finished
 * yet" and renders as dashes. Tested with < 0.0f rather than == LAP_NO_TIME:
 * comparing a float against a sentinel for exact equality is how a sentinel
 * that has been through one arithmetic operation stops matching. */
static void hud_format_time(char *out, size_t out_size, f32 seconds) {
    int minutes, whole, hundredths;

    if (seconds < 0.0f) {
        snprintf(out, out_size, "--:--.--");
        return;
    }
    /* Truncate, do not round: a lap shown as 1:20.00 that was actually
     * 1:19.996 reads as a whole hundredth slower than it was, and rounding UP
     * across the minute boundary would print 1:59.100 -> 2:00.00 for a time
     * that never reached two minutes. */
    whole = (int)seconds;
    hundredths = (int)((seconds - (f32)whole) * 100.0f);

    /* EVERY field is clamped, and not for cosmetic reasons. gcc's
     * -Werror=format-truncation reasons about the value ranges it can prove,
     * and it cannot prove anything about an int derived from a float: it
     * assumed all three conversions could be eleven digits and reported
     * "output between 8 and 26 bytes into a destination of size 16".
     *
     * Widening the buffer is the wrong fix -- it silences the diagnostic
     * without making the range true. Clamping is what actually makes the
     * bound hold: minutes and hundredths become two digits each and
     * whole % 60 becomes 0..59, so the result is provably 8 bytes.
     *
     * 5999 seconds is 99:59. A lap longer than that is not a lap. */
    if (whole < 0) whole = 0;
    if (whole > 5999) whole = 5999;
    if (hundredths < 0) hundredths = 0;
    if (hundredths > 99) hundredths = 99;
    minutes = whole / 60;

    snprintf(out, out_size, "%d:%02d.%02d", minutes, whole % 60, hundredths);
}

/* A horizontal fill bar: slot, then `fraction` of it filled from the left.
 * Clamped here rather than at every call site -- an out-of-range fraction is
 * a bug in the caller's data, and a bar that draws outside its own slot hides
 * that bug behind a rendering artefact instead of showing it as a full bar. */
static void hud_bar(f32 x, f32 y, f32 w, f32 h, f32 fraction, u32 colour) {
    if (!(fraction > 0.0f)) fraction = 0.0f;   /* also catches NaN */
    if (fraction > 1.0f) fraction = 1.0f;
    C2D_DrawRectSolid(x, y, 0.0f, w, h, CLR_SLOT);
    if (fraction > 0.0f) {
        C2D_DrawRectSolid(x, y, 0.0f, w * fraction, h, colour);
    }
}

/* A bar that fills outward from its own centre, for a signed value in
 * [-1, +1]. Steering is the only thing this is used for, and a left-filling
 * bar for left lock is the whole point: it reads the way the stick moves. */
static void hud_bar_centred(f32 x, f32 y, f32 w, f32 h, f32 value, u32 colour) {
    f32 half = w * 0.5f;
    f32 mag;
    if (!(value > -1.0f)) value = -1.0f;       /* NaN lands here too */
    if (value > 1.0f) value = 1.0f;
    mag = fabsf(value) * half;

    C2D_DrawRectSolid(x, y, 0.0f, w, h, CLR_SLOT);
    if (value >= 0.0f) {
        C2D_DrawRectSolid(x + half, y, 0.0f, mag, h, colour);
    } else {
        C2D_DrawRectSolid(x + half - mag, y, 0.0f, mag, h, colour);
    }
    /* Centre tick, drawn last so it stays visible over the fill. */
    C2D_DrawRectSolid(x + half - 1.0f, y - 1.0f, 0.0f, 2.0f, h + 2.0f, CLR_TEXT);
}
#endif /* __3DS__ */

void hud_init(void) {
#ifdef __3DS__
    if (s_ready) return;

    s_target = C3D_RenderTargetCreate(BOTTOM_SCREEN_HEIGHT, BOTTOM_SCREEN_WIDTH,
                                       GPU_RB_RGBA8, GPU_RB_DEPTH24_STENCIL8);
    if (s_target) {
        C3D_RenderTargetSetOutput(s_target, GFX_BOTTOM, GFX_LEFT,
                                   HUD_DISPLAY_TRANSFER_FLAGS);
    }
    s_text_buf = C2D_TextBufNew(HUD_TEXT_BUF_GLYPHS);

    /* Both or neither: a target with no text buffer would draw a blank panel
     * every frame and look like a rendering bug rather than a failed alloc. */
    s_ready = (s_target != NULL) && (s_text_buf != NULL);
#endif
}

void hud_shutdown(void) {
#ifdef __3DS__
    if (!s_ready) return;
    if (s_text_buf) {
        C2D_TextBufDelete(s_text_buf);
        s_text_buf = NULL;
    }
    if (s_target) {
        C3D_RenderTargetDelete(s_target);
        s_target = NULL;
    }
    s_ready = false;
#endif
}

void hud_draw(const HudStats *stats) {
#ifdef __3DS__
    static const char *WHEEL_LABEL[4] = { "FL", "FR", "RL", "RR" };
    f32 rev_fraction, speed_kmh;
    int i;

    if (!s_ready || !stats) return;

    /* NO C2D_Prepare() here. debugdraw_frame_end already called it for the
     * top screen's text, and calling it again is not free: it rebinds
     * citro2d's shader program and resets the depth test, which is exactly
     * the state clobbering that caused v0.1's total 3D blackout. Nothing
     * draws in 3D after this point in the frame, so leaving citro2d's state
     * as debugdraw left it is both correct and cheaper.
     *
     * The coupling is real and worth naming: if debugdraw ever stops drawing
     * text -- and therefore stops calling C2D_Prepare -- this panel goes
     * blank, because citro2d's program would never get bound. main.c always
     * draws top-screen text, so today it always does. */
    /* RESET THE GLYPH CURSOR EVERY FRAME. C2D_TextParse appends into the
     * buffer and never reclaims anything on its own; without this the 1024
     * glyphs are consumed in about forty frames, after which every parse
     * returns NULL and C2D_DrawText draws nothing at all.
     *
     * That failure is silent and it does not look like a text bug: the panel's
     * rectangles and bars keep rendering perfectly, so the screen reads as a
     * layout that was drawn without any labels. Measured, not reasoned -- the
     * first build of this file was missing this line, and the ten-second
     * capture showed every bar in place and not one character of text. */
    C2D_TextBufClear(s_text_buf);

    C2D_TargetClear(s_target, CLR_PANEL);
    C2D_SceneBegin(s_target);

    speed_kmh = stats->speed_ms * 3.6f;

    /* ---- header band ---- */
    C2D_DrawRectSolid(0.0f, 0.0f, 0.0f, 320.0f, 18.0f, CLR_BAND);
    hud_text(6.0f, 2.0f, 0.50f, CLR_ACCENT, "DiRT2");
    hud_text(52.0f, 3.0f, 0.42f, CLR_DIM, "TELEMETRY");
    hud_text(130.0f, 2.0f, 0.50f, CLR_TEXT, "LAP %d", stats->lap_count + 1);
    hud_text(196.0f, 3.0f, 0.42f, CLR_DIM, "%4.1f fps  %4.1f ms",
             (double)stats->fps, (double)stats->frame_ms);

    /* ---- speed, the one number worth reading at a glance ---- */
    hud_text(8.0f, 22.0f, 1.35f, CLR_TEXT, "%3.0f", (double)speed_kmh);
    hud_text(96.0f, 46.0f, 0.50f, CLR_ACCENT, "km/h");
    hud_text(96.0f, 26.0f, 0.44f, CLR_DIM, "%5.1f m/s", (double)stats->speed_ms);

    /* ---- revs ----
     * Scaled from idle, not from zero: an engine that never drops below idle
     * would otherwise show a bar that never empties, which reads as broken. */
    rev_fraction = 0.0f;
    if (stats->max_rpm > stats->idle_rpm) {
        rev_fraction = (stats->engine_rpm - stats->idle_rpm) /
                       (stats->max_rpm - stats->idle_rpm);
    }
    hud_text(176.0f, 24.0f, 0.44f, CLR_DIM, "rpm");
    hud_text(206.0f, 22.0f, 0.55f, CLR_TEXT, "%5.0f", (double)stats->engine_rpm);
    /* Redline colour above 85% of the usable band -- the same threshold the
     * bar's own red segment marks, so the number and the bar agree. */
    hud_bar(176.0f, 46.0f, 136.0f, 10.0f, rev_fraction,
            (rev_fraction > 0.85f) ? CLR_STOP : CLR_ACCENT);
    C2D_DrawRectSolid(176.0f + 136.0f * 0.85f, 44.0f, 0.0f, 1.0f, 14.0f, CLR_STOP);

    /* ---- driver inputs ---- */
    hud_text(8.0f, 64.0f, 0.42f, CLR_DIM, "THR");
    hud_bar(44.0f, 65.0f, 120.0f, 8.0f, stats->throttle, CLR_GO);
    hud_text(8.0f, 78.0f, 0.42f, CLR_DIM, "BRK");
    hud_bar(44.0f, 79.0f, 120.0f, 8.0f, stats->brake, CLR_STOP);
    hud_text(8.0f, 92.0f, 0.42f, CLR_DIM, "STR");
    hud_bar_centred(44.0f, 93.0f, 120.0f, 8.0f, stats->steer, CLR_ACCENT);

    hud_text(176.0f, 64.0f, 0.42f, CLR_DIM, "handbrake");
    hud_text(250.0f, 64.0f, 0.42f,
             stats->handbrake ? CLR_STOP : CLR_AIR,
             "%s", stats->handbrake ? "ON" : "off");
    hud_text(176.0f, 78.0f, 0.42f, CLR_DIM, "steps/frame");
    hud_text(258.0f, 78.0f, 0.42f, CLR_TEXT, "%lu",
             (unsigned long)stats->physics_steps);
    hud_text(176.0f, 92.0f, 0.42f, CLR_DIM, "pos");
    hud_text(200.0f, 92.0f, 0.42f, CLR_TEXT, "%+.0f %+.0f %+.0f",
             (double)stats->position.x, (double)stats->position.y,
             (double)stats->position.z);

    /* ---- per-wheel suspension ----
     * Laid out as the car is: front row above rear row, left column left.
     * A wheel in the air is drawn in the dim "air" colour rather than being
     * hidden, because a wheel that stops reporting is indistinguishable from
     * one that is merely unloaded, and those are very different problems. */
    C2D_DrawRectSolid(0.0f, 108.0f, 0.0f, 320.0f, 16.0f, CLR_BAND);
    hud_text(6.0f, 110.0f, 0.44f, CLR_DIM, "SUSPENSION");
    hud_text(112.0f, 110.0f, 0.40f, CLR_DIM, "compression / load");

    for (i = 0; i < 4; i++) {
        const HudWheel *w = &stats->wheels[i];
        /* WheelIndex order is FL, FR, RL, RR -- two per row, front row first,
         * which is why the column is i&1 and the row is i>>1. */
        f32 x = 8.0f + (f32)(i & 1) * 156.0f;
        f32 y = 130.0f + (f32)(i >> 1) * 46.0f;
        f32 fraction = 0.0f;

        if (w->max_travel > 0.0f) fraction = w->compression / w->max_travel;

        hud_text(x, y, 0.46f, w->grounded ? CLR_TEXT : CLR_AIR, "%s", WHEEL_LABEL[i]);
        hud_text(x + 26.0f, y + 2.0f, 0.38f, CLR_DIM, "%.0f N", (double)w->normal_load);
        if (!w->grounded) hud_text(x + 90.0f, y + 2.0f, 0.38f, CLR_AIR, "AIR");
        hud_bar(x, y + 18.0f, 140.0f, 8.0f, fraction,
                w->grounded ? CLR_GO : CLR_AIR);
        hud_text(x, y + 28.0f, 0.36f, CLR_DIM, "%.3f m", (double)w->compression);
    }

    /* ---- race strip, bottom of the panel ----
     * The band's own background doubles as the lap progress bar: it fills from
     * the left as the car goes round. That is why there is no separate bar
     * widget here -- a 320-wide strip that is already the full width of the
     * screen is the best progress bar available, and spending 8 more pixels of
     * a 240-pixel screen on a second one would be worse, not better.
     *
     * OFF-COURSE is shown by turning the whole strip red rather than by adding
     * a label. The strip is in peripheral vision while the driver looks at the
     * top screen, and a colour change is the only thing peripheral vision
     * actually resolves; a word saying "OFF" would never be read. */
    C2D_DrawRectSolid(0.0f, 224.0f, 0.0f, 320.0f, 16.0f,
                      stats->on_track ? CLR_BAND : CLR_STOP);
    if (stats->on_track) {
        f32 p = stats->lap_progress;
        if (!(p > 0.0f)) p = 0.0f;
        if (p > 1.0f) p = 1.0f;
        C2D_DrawRectSolid(0.0f, 224.0f, 0.0f, 320.0f * p, 16.0f, CLR_SLOT);
    }
    {
        char cur[16], last[16], best[16];
        hud_format_time(cur, sizeof(cur), stats->current_lap_time);
        hud_format_time(last, sizeof(last), stats->last_lap_time);
        hud_format_time(best, sizeof(best), stats->best_lap_time);
        hud_text(6.0f, 226.0f, 0.40f, CLR_TEXT, "%s", cur);
        hud_text(96.0f, 226.0f, 0.40f, CLR_DIM, "last %s", last);
        hud_text(206.0f, 226.0f, 0.40f, CLR_ACCENT, "best %s", best);
    }

    C2D_Flush();
#else
    (void)stats;
#endif
}
