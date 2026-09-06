/*---------------------------------------------------------------------------------
 * pausemenu.c -- implementation. See pausemenu.h for the API contract, the
 * call-order main.c is expected to follow, and why the state machine and the
 * drawing are split the way they are.
 *
 * KEY BIT MIRRORING, AND WHY THIS FILE DOES NOT INCLUDE 3ds.h FOR THEM.
 *   pausemenu_update() (the state machine) must stay host-compilable -- see
 *   pausemenu.h's HOST BUILD note -- so it cannot #include <3ds.h> for
 *   KEY_A/KEY_B/KEY_SELECT/KEY_DUP/KEY_DDOWN. Instead the five bit values
 *   this file actually needs are mirrored locally as PM_KEY_* below, copied
 *   from libctru's own 3ds/services/hid.h (confirmed against the installed
 *   devkitPro copy at the time this was written: KEY_A=BIT(0), KEY_B=BIT(1),
 *   KEY_SELECT=BIT(2), KEY_DUP=BIT(6), KEY_DDOWN=BIT(7)) so the exact same
 *   state-machine code runs, bit-for-bit, on real hardware and in this
 *   module's host probe -- not a hand-copied stand-in that could silently
 *   drift from what the console actually reports. These are long-stable
 *   libctru ABI constants; if they were ever renumbered every KEY_* consumer
 *   in this codebase would need the same update, not just this file.
 *
 *   D-PAD ONLY, NOT KEY_UP/KEY_DOWN. libctru's KEY_UP/KEY_DOWN are
 *   KEY_DUP|KEY_CPAD_UP and KEY_DDOWN|KEY_CPAD_DOWN -- Circle Pad included.
 *   steve's own words for this feature only ever mention D-pad navigation,
 *   so this deliberately tests the bare KEY_DUP/KEY_DDOWN bits rather than
 *   the combined masks; adding Circle Pad menu navigation would be a scope
 *   addition nobody asked for.
 *
 * Every citro2d/citro3d call in this file is behind #ifdef __3DS__, same
 * convention as hud.c/debugdraw.c, so pausemenu_update and this file's other
 * host-visible logic compile on the WSL host build as real, tested code, and
 * the two draw functions compile to deliberate no-ops.
 *---------------------------------------------------------------------------------*/
#include "ui/pausemenu.h"

#include "render/renderer.h"   /* renderer_get_target() -- top screen        */
#include "render/hud.h"        /* hud_get_target() -- bottom screen; see
                                 * pausemenu.h's note on why this module must
                                 * NOT create its own bottom-screen target    */

#include <string.h>
#include <stdio.h>

#ifdef __3DS__
#include <3ds.h>
#include <citro3d.h>
#include <citro2d.h>
#include <stdarg.h>
#endif

/* See file header. */
#define PM_KEY_A       (1u << 0)
#define PM_KEY_B       (1u << 1)
#define PM_KEY_SELECT  (1u << 2)
#define PM_KEY_DUP     (1u << 6)
#define PM_KEY_DDOWN   (1u << 7)

#ifdef __3DS__
/* Declared here, ahead of pausemenu_init/pausemenu_shutdown below, rather
 * than down in the DRAWING section where they are also used -- both the
 * init/shutdown pair AND the draw functions need these, and C has no
 * forward declaration for a file-static variable, so whichever use comes
 * first in the file has to be the declaration.
 *
 * Sized well above this menu's actual per-frame text-call count (at most
 * ~8 strings on the busiest screen, the root list) -- same "comfortable
 * headroom, not a meaningful slice of memory" policy as hud.c's
 * HUD_TEXT_BUF_GLYPHS. Two buffers, one per screen, for the same reason
 * hud.c and debugdraw.c each own exactly one buffer for the one screen they
 * draw to -- text queued for one screen has no business sharing a cursor
 * with the other. */
#define PAUSEMENU_TEXT_BUF_GLYPHS_TOP    128
#define PAUSEMENU_TEXT_BUF_GLYPHS_BOTTOM 256

static C2D_TextBuf s_text_buf_top = NULL;
static C2D_TextBuf s_text_buf_bottom = NULL;
static bool s_draw_ready = false;
#endif

/* =========================================================================
 * STATE MACHINE -- pure logic, host-compilable, no drawing call anywhere
 * below this point until the #ifdef __3DS__ drawing section further down.
 * ========================================================================= */

void pausemenu_init(PauseMenu *menu) {
    if (menu) {
        memset(menu, 0, sizeof(*menu));
        menu->state = PAUSE_MENU_CLOSED;
        /* menu->open, root_cursor, quit_requested, update_result and
         * update_qr.size are all correctly zero/false from the memset --
         * QrCode.size == 0 is qr.h's own "do not draw" value, and an
         * all-zero UpdateResult has no meaning until update_check() fills
         * it, which is exactly the state before any check has ever run. */
    }
#ifdef __3DS__
    if (!s_draw_ready) {
        s_text_buf_top = C2D_TextBufNew(PAUSEMENU_TEXT_BUF_GLYPHS_TOP);
        s_text_buf_bottom = C2D_TextBufNew(PAUSEMENU_TEXT_BUF_GLYPHS_BOTTOM);
        /* Both or neither -- see hud_init's identical reasoning: a menu that
         * draws panels/highlights but never any text would look like a
         * rendering bug, not a failed allocation. */
        s_draw_ready = (s_text_buf_top != NULL) && (s_text_buf_bottom != NULL);
    }
#endif
}

void pausemenu_shutdown(PauseMenu *menu) {
    (void)menu; /* caller-owned memory -- see pausemenu.h; nothing to free here */
#ifdef __3DS__
    if (s_text_buf_top) {
        C2D_TextBufDelete(s_text_buf_top);
        s_text_buf_top = NULL;
    }
    if (s_text_buf_bottom) {
        C2D_TextBufDelete(s_text_buf_bottom);
        s_text_buf_bottom = NULL;
    }
    s_draw_ready = false;
#endif
}

void pausemenu_update(PauseMenu *menu, uint32_t keys_down) {
    if (!menu) return;

    if (!menu->open) {
        if (keys_down & PM_KEY_SELECT) {
            menu->open = true;
            menu->state = PAUSE_MENU_ROOT;
            menu->root_cursor = PAUSE_ROOT_RESUME;
        }
        return;
    }

    switch (menu->state) {
    case PAUSE_MENU_ROOT:
        /* SELECT closes unconditionally, checked first so it always wins
         * over a simultaneous D-pad/A press in the same input sample. */
        if (keys_down & PM_KEY_SELECT) {
            menu->open = false;
            menu->state = PAUSE_MENU_CLOSED;
            break;
        }
        if (keys_down & PM_KEY_DDOWN) {
            menu->root_cursor = (menu->root_cursor + 1) % PAUSE_ROOT_ITEM_COUNT;
        }
        if (keys_down & PM_KEY_DUP) {
            menu->root_cursor = (menu->root_cursor + PAUSE_ROOT_ITEM_COUNT - 1)
                                 % PAUSE_ROOT_ITEM_COUNT;
        }
        if (keys_down & PM_KEY_A) {
            switch (menu->root_cursor) {
            case PAUSE_ROOT_RESUME:
                menu->open = false;
                menu->state = PAUSE_MENU_CLOSED;
                break;
            case PAUSE_ROOT_OPTIONS:
                menu->state = PAUSE_MENU_OPTIONS;
                break;
            case PAUSE_ROOT_QUIT:
                menu->quit_requested = true;
                menu->open = false;
                menu->state = PAUSE_MENU_CLOSED;
                break;
            default:
                break; /* PAUSE_ROOT_ITEM_COUNT is not a real item */
            }
        }
        break;

    case PAUSE_MENU_OPTIONS:
        if (keys_down & PM_KEY_B) {
            menu->state = PAUSE_MENU_ROOT;
            break;
        }
        if (keys_down & PM_KEY_A) {
            /* Arms the checking state; see pausemenu.h's
             * PAUSE_MENU_UPDATE_CHECKING comment for why update_check() is
             * NOT called here, in this same call. */
            menu->state = PAUSE_MENU_UPDATE_CHECKING;
        }
        break;

    case PAUSE_MENU_UPDATE_CHECKING:
        /* Reaching this CASE at all (as opposed to the OPTIONS case just
         * having set the state to this value) can only happen on a call
         * that is strictly later than the one that armed it -- and per
         * pausemenu.h's call-order contract, this module's draw functions
         * run once, in full, between every pair of pausemenu_update() calls.
         * So control reaching this line already proves at least one whole
         * frame rendered the "Checking for updates..." screen. That is what
         * update_check()'s own contract (update.h's "BLOCKING IS
         * DELIBERATE") requires before the blocking call is made, and it is
         * satisfied here with no extra "has a frame drawn yet" flag needed --
         * the state value itself IS that flag. No input is read in this
         * state; whatever was in keys_down this call is irrelevant. */
        update_check(&menu->update_result);
        /* Built unconditionally, success or failure: on failure
         * release_url is the empty string (update_check zeroes `out` up
         * front and only fills release_url on UPDATE_OK), and
         * qr_encode_url("") returns false with qr->size left at 0 -- qr.h's
         * own "do not draw" value -- so the failure path needs no special
         * casing here at all. */
        qr_encode_url(&menu->update_qr, menu->update_result.release_url);
        menu->state = PAUSE_MENU_UPDATE_RESULT;
        break;

    case PAUSE_MENU_UPDATE_RESULT:
        if (keys_down & PM_KEY_B) {
            menu->state = PAUSE_MENU_OPTIONS;
        }
        break;

    case PAUSE_MENU_CLOSED:
    default:
        break; /* unreachable: menu->open is false in this state, handled above */
    }
}

bool pausemenu_is_open(const PauseMenu *menu) {
    return menu && menu->open;
}

bool pausemenu_take_quit_request(PauseMenu *menu) {
    bool requested;
    if (!menu) return false;
    requested = menu->quit_requested;
    menu->quit_requested = false;
    return requested;
}

/* =========================================================================
 * DRAWING -- citro2d, top and bottom screen. Everything below is behind
 * #ifdef __3DS__; the two public draw functions have a trivial (void)menu
 * no-op body on host, matching hud.c/debugdraw.c's convention.
 * ========================================================================= */

#ifdef __3DS__

/* s_text_buf_top / s_text_buf_bottom / s_draw_ready are declared earlier in
 * this file, ahead of pausemenu_init/pausemenu_shutdown -- see that
 * declaration's comment for why.
 *
 * Palette intentionally copies hud.c's values verbatim (see render/hud.c) so
 * the pause menu reads as drawn by the same hand, on the same day, as the
 * rest of this project's UI -- not a second, slightly-off dark theme. */
#define CLR_OVERLAY   C2D_Color32(0x00, 0x00, 0x00, 0xB0)
#define CLR_PANEL     C2D_Color32(0x14, 0x16, 0x1A, 0xFF)
#define CLR_BAND      C2D_Color32(0x1F, 0x23, 0x2A, 0xFF)
#define CLR_SLOT      C2D_Color32(0x2A, 0x2F, 0x38, 0xFF)
#define CLR_TEXT      C2D_Color32(0xE8, 0xE8, 0xE2, 0xFF)
#define CLR_DIM       C2D_Color32(0x8A, 0x92, 0x9E, 0xFF)
#define CLR_ACCENT    C2D_Color32(0xF0, 0xC4, 0x20, 0xFF)
#define CLR_STOP      C2D_Color32(0xE0, 0x4C, 0x3C, 0xFF)
#define CLR_QR_LIGHT  C2D_Color32(0xFF, 0xFF, 0xFF, 0xFF)
#define CLR_QR_DARK   C2D_Color32(0x00, 0x00, 0x00, 0xFF)

/* Plain screen-pixel space, top-left origin -- same convention debugdraw.c's
 * top-screen text and hud.c's bottom-screen drawing both already use.
 * Values are the 3DS's real screen resolutions, not the transposed
 * (height, width) argument order C3D_RenderTargetCreate happens to take
 * (see hud.c's comment on that transpose -- it does not affect these). */
#define TOP_SCREEN_W    400.0f
#define TOP_SCREEN_H    240.0f
#define BOTTOM_SCREEN_W 320.0f
#define BOTTOM_SCREEN_H 240.0f

static const char *ROOT_LABELS[PAUSE_ROOT_ITEM_COUNT] = { "Resume", "Options", "Quit" };

/* Draws one string, same idiom as hud.c's hud_text/debugdraw.c's queued
 * text: the C2D_Text is a stack local, C2D_TextParse copies glyph data into
 * `buf`, nothing here outlives the draw call. Takes the target buffer
 * explicitly (unlike hud_text, which only ever has one buffer to use) since
 * this module owns two. */
static void pm_text(C2D_TextBuf buf, f32 x, f32 y, f32 scale, u32 colour,
                     const char *fmt, ...) {
    char line[96];
    C2D_Text text;
    va_list args;

    va_start(args, fmt);
    vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);

    C2D_TextParse(&text, buf, line);
    C2D_TextOptimize(&text);
    C2D_DrawText(&text, C2D_WithColor, x, y, 0.5f, scale, scale, colour);
}

void pausemenu_draw_top(const PauseMenu *menu) {
    C3D_RenderTarget *target;
    int i;
    const f32 panel_x = 100.0f, panel_y = 30.0f, panel_w = 200.0f, panel_h = 180.0f;

    if (!s_draw_ready) return;

    /* RESET THE GLYPH CURSOR EVERY FRAME, BEFORE ANY EARLY RETURN BELOW.
     * hud.c's own header comment documents exactly this failure mode:
     * skipping this (or letting an early return skip it) silently exhausts
     * the buffer after ~40 draws -- not this frame's 40, EVER, because a
     * skipped clear never gives the slots back -- after which every later
     * C2D_TextParse returns NULL and text vanishes while rectangles keep
     * drawing, with no error at all. Placed here, first, so a future edit
     * that adds another early-return below this line cannot reintroduce
     * that bug by accident. */
    C2D_TextBufClear(s_text_buf_top);

    if (!menu || !menu->open) return;

    target = renderer_get_target();
    if (!target) return;

    /* No C2D_Prepare() here -- deliberately, same reasoning as hud_draw's
     * identical comment in render/hud.c. main.c's frame loop calls
     * debugdraw_text() unconditionally every frame (dt/steps/speed/lap
     * readouts), so debugdraw_frame_end() has already called C2D_Prepare()
     * for the top screen this frame by the time this function runs -- see
     * pausemenu.h's call-order note, which places this call after
     * debugdraw_frame_end(). Calling C2D_Prepare() again here would rebind
     * citro2d's shader program and reset the depth test for no benefit,
     * exactly the state-clobbering hud.c's comment warns about. Re-selecting
     * the target with C2D_SceneBegin is still needed and is cheap/idempotent
     * -- it does not depend on which target happened to be active last. */
    C2D_SceneBegin(target);

    /* Darken the frozen 3D scene behind the menu so the panel reads as
     * modal, not as an overlay competing with the road for attention. */
    C2D_DrawRectSolid(0.0f, 0.0f, 0.0f, TOP_SCREEN_W, TOP_SCREEN_H, CLR_OVERLAY);

    C2D_DrawRectSolid(panel_x, panel_y, 0.0f, panel_w, 18.0f, CLR_BAND);
    C2D_DrawRectSolid(panel_x, panel_y + 18.0f, 0.0f, panel_w, panel_h - 18.0f, CLR_PANEL);

    switch (menu->state) {
    case PAUSE_MENU_ROOT:
        pm_text(s_text_buf_top, panel_x + 8.0f, panel_y + 2.0f, 0.5f, CLR_ACCENT, "PAUSED");
        for (i = 0; i < PAUSE_ROOT_ITEM_COUNT; i++) {
            f32 iy = panel_y + 30.0f + (f32)i * 24.0f;
            bool sel = (i == menu->root_cursor);
            if (sel) {
                C2D_DrawRectSolid(panel_x + 4.0f, iy - 2.0f, 0.0f, panel_w - 8.0f, 20.0f, CLR_SLOT);
            }
            pm_text(s_text_buf_top, panel_x + 16.0f, iy, 0.5f,
                    sel ? CLR_ACCENT : CLR_TEXT, "%s", ROOT_LABELS[i]);
        }
        pm_text(s_text_buf_top, panel_x + 8.0f, panel_y + panel_h - 16.0f, 0.38f, CLR_DIM,
                "A select   SELECT close");
        break;

    case PAUSE_MENU_OPTIONS:
        pm_text(s_text_buf_top, panel_x + 8.0f, panel_y + 2.0f, 0.5f, CLR_ACCENT, "OPTIONS");
        C2D_DrawRectSolid(panel_x + 4.0f, panel_y + 28.0f, 0.0f, panel_w - 8.0f, 20.0f, CLR_SLOT);
        pm_text(s_text_buf_top, panel_x + 16.0f, panel_y + 30.0f, 0.5f, CLR_ACCENT, "Update");
        pm_text(s_text_buf_top, panel_x + 8.0f, panel_y + panel_h - 16.0f, 0.38f, CLR_DIM,
                "A check   B back");
        break;

    case PAUSE_MENU_UPDATE_CHECKING:
        pm_text(s_text_buf_top, panel_x + 8.0f, panel_y + 2.0f, 0.5f, CLR_ACCENT, "OPTIONS");
        pm_text(s_text_buf_top, panel_x + 16.0f, panel_y + 60.0f, 0.46f, CLR_TEXT,
                "Checking for updates...");
        break;

    case PAUSE_MENU_UPDATE_RESULT:
        pm_text(s_text_buf_top, panel_x + 8.0f, panel_y + 2.0f, 0.5f, CLR_ACCENT, "OPTIONS");
        pm_text(s_text_buf_top, panel_x + 16.0f, panel_y + 60.0f, 0.44f, CLR_TEXT,
                "Update check done");
        pm_text(s_text_buf_top, panel_x + 16.0f, panel_y + 84.0f, 0.40f, CLR_DIM,
                "see bottom screen");
        pm_text(s_text_buf_top, panel_x + 8.0f, panel_y + panel_h - 16.0f, 0.38f, CLR_DIM,
                "B back");
        break;

    case PAUSE_MENU_CLOSED:
    default:
        break; /* unreachable: guarded by the !menu->open return above */
    }

    C2D_Flush();
}

void pausemenu_draw_bottom(const PauseMenu *menu) {
    C3D_RenderTarget *target;

    if (!s_draw_ready) return;

    /* Same placement rule as pausemenu_draw_top -- cleared first, before any
     * early return, so this buffer can never silently exhaust either. */
    C2D_TextBufClear(s_text_buf_bottom);

    if (!menu || !menu->open) return;

    target = hud_get_target();
    if (!target) return;

    /* hud_draw's own target -- this module never creates a second one for
     * the bottom screen. Same C2D_TargetClear-not-C3D_RenderTargetClear
     * reasoning as hud.c's file header: this runs mid-frame, after the top
     * screen already drew, so the clear must split the command buffer
     * first rather than eating prior drawing. */
    C2D_TargetClear(target, CLR_PANEL);
    C2D_SceneBegin(target);

    if (menu->state != PAUSE_MENU_UPDATE_RESULT) {
        /* Every other open state: a plain "PAUSED" panel, not stale
         * telemetry and not a stale old update result -- a bottom screen
         * that kept showing the LAST check's result while the player is
         * back at the root menu would misreport what "Update" is about to
         * do if pressed again. */
        pm_text(s_text_buf_bottom, 8.0f, 8.0f, 0.5f, CLR_DIM, "PAUSED");
        C2D_Flush();
        return;
    }

    {
        const UpdateResult *r = &menu->update_result;

        pm_text(s_text_buf_bottom, 8.0f, 6.0f, 0.5f, CLR_ACCENT, "UPDATE CHECK");
        pm_text(s_text_buf_bottom, 8.0f, 30.0f, 0.42f, CLR_DIM, "current");
        pm_text(s_text_buf_bottom, 8.0f, 44.0f, 0.46f, CLR_TEXT, "%s", r->current_version);

        if (r->status == UPDATE_OK) {
            pm_text(s_text_buf_bottom, 8.0f, 70.0f, 0.42f, CLR_DIM, "latest");
            pm_text(s_text_buf_bottom, 8.0f, 84.0f, 0.46f,
                    r->update_available ? CLR_ACCENT : CLR_TEXT, "%s", r->latest_version);
            pm_text(s_text_buf_bottom, 8.0f, 112.0f, 0.42f,
                    r->update_available ? CLR_ACCENT : CLR_DIM, "%s",
                    r->update_available ? "update available" : "up to date");
        } else {
            pm_text(s_text_buf_bottom, 8.0f, 70.0f, 0.42f, CLR_STOP, "check failed");
        }

        /* `detail` (update.h) is always filled: empty on the success path,
         * a specific diagnostic on any failure. Shown as up to three fixed-
         * width lines rather than word-wrapped -- this is a short diagnostic
         * string (curl error text, an HTTP status line), not prose, so a
         * hard character cut is simple and cannot misbehave on a string
         * with no convenient word breaks (e.g. a raw echoed URL). */
        if (r->detail[0] != '\0') {
            char line1[24], line2[24], line3[24];
            size_t len = strlen(r->detail);
            snprintf(line1, sizeof(line1), "%.23s", r->detail);
            snprintf(line2, sizeof(line2), "%.23s", (len > 23) ? r->detail + 23 : "");
            snprintf(line3, sizeof(line3), "%.23s", (len > 46) ? r->detail + 46 : "");
            pm_text(s_text_buf_bottom, 8.0f, 140.0f, 0.36f, CLR_DIM, "%s", line1);
            if (line2[0]) pm_text(s_text_buf_bottom, 8.0f, 154.0f, 0.36f, CLR_DIM, "%s", line2);
            if (line3[0]) pm_text(s_text_buf_bottom, 8.0f, 168.0f, 0.36f, CLR_DIM, "%s", line3);
        }

        pm_text(s_text_buf_bottom, 8.0f, 226.0f, 0.40f, CLR_DIM, "B back");
    }

    /* ---- QR code, right side of the bottom screen ----
     * INTEGER PIXELS PER MODULE, ALWAYS. A fractional scale draws some
     * module columns/rows one screen pixel wider than their neighbours,
     * which is exactly the kind of uneven module width a phone camera's
     * decoder fails to lock onto -- see pausemenu.h/this project's QR
     * requirement. `scale` below is computed with integer division for
     * exactly this reason, never as a float ratio.
     *
     * QUIET ZONE >= 4 MODULES. A QR code with no light margin around it does
     * not reliably scan -- decoders use the quiet zone to find the code's
     * edges in the first place. 4 modules is the standard minimum quoted for
     * QR Model 2 and is what `quiet` below reserves on every side. */
    if (menu->update_qr.size > 0) {
        const QrCode *qr = &menu->update_qr;
        const f32 box_x = 156.0f, box_y = 30.0f, box_w = 156.0f, box_h = 196.0f;
        const int quiet = 4;
        const int total_modules = qr->size + quiet * 2;
        const f32 box_min = (box_w < box_h) ? box_w : box_h;
        int scale = (int)(box_min / (f32)total_modules); /* integer division: the
            largest whole number of pixels-per-module that still fits the box */
        f32 px_size, origin_x, origin_y;
        int mx, my;

        if (scale < 1) scale = 1; /* degrade to the smallest legal scale rather
            than draw nothing -- this project's URLs never actually reach a
            size that forces this (see qr.h's character-budget note), so this
            is a safety net, not the expected path; a 1px/module code that
            overflows the box slightly still beats a blank screen. */

        px_size = (f32)(total_modules * scale);
        origin_x = box_x + (box_w - px_size) * 0.5f; /* centred in the box */
        origin_y = box_y + (box_h - px_size) * 0.5f;

        /* One rect covers the quiet zone AND the light modules -- both are
         * the same colour, so there is no reason to draw the margin and the
         * light squares separately. */
        C2D_DrawRectSolid(origin_x, origin_y, 0.0f, px_size, px_size, CLR_QR_LIGHT);

        for (my = 0; my < qr->size; my++) {
            for (mx = 0; mx < qr->size; mx++) {
                if (qr->modules[my * QR_MAX_MODULES + mx]) {
                    C2D_DrawRectSolid(
                        origin_x + (f32)(quiet + mx) * (f32)scale,
                        origin_y + (f32)(quiet + my) * (f32)scale,
                        0.0f, (f32)scale, (f32)scale, CLR_QR_DARK);
                }
            }
        }
    } else {
        /* size == 0: either the check has not completed with UPDATE_OK yet,
         * or qr_encode_url() itself failed (qr.h's contract) -- either way,
         * qr.h says this must not be drawn. */
        pm_text(s_text_buf_bottom, 164.0f, 100.0f, 0.40f, CLR_DIM, "no QR code");
    }

    C2D_Flush();
}

#else /* !__3DS__ */

void pausemenu_draw_top(const PauseMenu *menu) {
    (void)menu;
}

void pausemenu_draw_bottom(const PauseMenu *menu) {
    (void)menu;
}

#endif /* __3DS__ */
