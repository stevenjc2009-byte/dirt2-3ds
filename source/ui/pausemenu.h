/*---------------------------------------------------------------------------------
 * pausemenu.h -- SELECT-button pause menu: Resume / Options / Quit, with
 * Options holding a single "Update" item that runs update.h's check-only
 * updater and shows the result.
 *
 * OWNER: ui. Splits cleanly along the same line every other ui/render module
 * in this project does: pausemenu_update() is pure state-machine logic (no
 * citro2d/citro3d call anywhere in it, host-compilable, unit-testable) and
 * pausemenu_draw_top()/pausemenu_draw_bottom() are the citro2d presentation
 * of whatever that state machine currently holds. main.c drives all three;
 * this header does not call itself.
 *
 * SCOPE, PER STEVE'S OWN WORDS: "so when I press select, it will open up a
 * pause menu with resume options and quit, and then it has options, update,
 * and then I check for updates and stuff." That is the whole feature. The
 * updater itself is CHECK-AND-REPORT ONLY (see update.h) -- this menu shows
 * update.h's UpdateResult, it does not add a download or install path on
 * top of it.
 *
 * CALL ORDER main.c IS EXPECTED TO FOLLOW (documented here the same way
 * hud.h documents hud_draw's ordering constraints):
 *
 *   startup:
 *     renderer_init(); hud_init(); ... ; pausemenu_init(&menu);
 *       -- pausemenu_init() allocates this module's own citro2d text buffers
 *       and therefore MUST run after renderer_init() (C2D_Init), same
 *       ordering rule as hud_init()/debugdraw_init().
 *
 *   every render frame, BEFORE the physics/input step for that frame:
 *     pausemenu_update(&menu, (uint32_t)hidKeysDown());
 *       -- exactly ONCE per rendered frame. See PauseMenuState's
 *       PAUSE_MENU_UPDATE_CHECKING comment below for why "exactly once per
 *       frame" is load-bearing, not a style preference: it is what turns a
 *       plain switch/case into "wait one whole frame before the blocking
 *       call", with no extra flag needed.
 *     if (pausemenu_is_open(&menu)) { skip/freeze the physics step and
 *       input_update() for this frame -- a paused game should not keep
 *       simulating underneath the menu. }
 *     if (pausemenu_take_quit_request(&menu)) { break out of the frame loop. }
 *
 *   inside the renderer_frame_begin/renderer_frame_end bracket, AFTER every
 *   top-screen 3D draw call and AFTER debugdraw_frame_end() (same
 *   C2D_Prepare-coupling reasoning as hud_draw -- see pausemenu_draw_top()'s
 *   own comment in pausemenu.c):
 *     pausemenu_draw_top(&menu);
 *
 *   for the bottom screen, call EXACTLY ONE of the following per frame, never
 *   both -- both clear+SceneBegin the SAME hud_get_target() target, so
 *   calling both is not unsafe but wastes a clear and whichever runs second
 *   wins:
 *     if (pausemenu_is_open(&menu)) pausemenu_draw_bottom(&menu);
 *     else                          hud_draw(&hud_stats);
 *
 *   shutdown, reverse of init order (same reasoning as hud_shutdown's own
 *   note -- this module's text buffers must go before C2D_Fini):
 *     pausemenu_shutdown(&menu); ... ; hud_shutdown(); renderer_shutdown();
 *
 * WHY THE PauseMenu STRUCT IS CALLER-OWNED, UNLIKE hud.c/debugdraw.c'S
 * STATE. Those two modules own 100% of their own state as file-static
 * globals because nothing outside them ever needs to read or reset it. A
 * PauseMenu is different: main.c needs to poll pausemenu_is_open() to decide
 * whether to freeze physics and pausemenu_take_quit_request() to decide
 * whether to exit, every frame, so the struct has to live somewhere main.c
 * can hold a pointer to. This module's own GPU resources (the two citro2d
 * text buffers) stay file-static internals for exactly the reason hud.c's
 * text buffer does -- nothing outside this file has any business touching
 * them, so they are not in the struct at all.
 *
 * NO DYNAMIC ALLOCATION. PauseMenu is plain fixed-size data (an UpdateResult
 * and a QrCode, both fixed-size structs -- QrCode is ~2.4 KB, see qr.h) that
 * main.c can put on the stack or in .bss; the only heap-ish allocation this
 * module does at all is the two small citro2d text buffers in pausemenu_init,
 * matching hud_init/debugdraw_init's own one-time-allocation policy.
 *
 * HOST BUILD. Every citro2d/citro3d call in pausemenu.c is behind
 * #ifdef __3DS__, same convention as hud.c/debugdraw.c, so pausemenu_update
 * (the actual state machine) and this header compile and are directly
 * testable on the WSL host build -- see this module's own verification
 * notes. pausemenu_draw_top/bottom compile to deliberate no-ops on host.
 *---------------------------------------------------------------------------------*/
#ifndef DIRT2_UI_PAUSEMENU_H
#define DIRT2_UI_PAUSEMENU_H

#include <stdint.h>
#include <stdbool.h>

#include "update/update.h"  /* UpdateResult -- update.h has zero 3DS
                              * dependency itself, so pulling it in here does
                              * not cost this header its host-compilability. */
#include "ui/qr.h"           /* QrCode -- same reasoning, qr.h/qr.c are pure C. */

#ifdef __cplusplus
extern "C" {
#endif

/* The menu's current screen. A flat state machine, not a stack, because the
 * whole menu is only three screens deep (root -> options -> update) and a
 * generic push/pop stack would be more machinery than three fixed levels
 * need -- see coding-style's "simplest thing that works". */
typedef enum PauseMenuState {
    PAUSE_MENU_CLOSED = 0,          /* menu not showing; game runs normally  */
    PAUSE_MENU_ROOT,                 /* Resume / Options / Quit               */
    PAUSE_MENU_OPTIONS,              /* Update                                */

    /* ARMED THE INSTANT Update is chosen, and left showing for AT LEAST one
     * full rendered frame before anything blocking happens -- this is the
     * whole point of this being its own state rather than update_check()
     * being called straight out of the OPTIONS case.
     *
     * update.h's update_check() is a deliberate, possibly multi-second
     * blocking call, and its contract explicitly assumes the caller has
     * ALREADY drawn a "Checking for updates..." frame before calling it (see
     * update.h's "BLOCKING IS DELIBERATE" comment) -- otherwise the console
     * appears to freeze with no explanation.
     *
     * pausemenu_update() is documented above to be called exactly once per
     * rendered frame, and pausemenu_draw_top()/pausemenu_draw_bottom() are
     * called once per frame straight after it, before the next frame's
     * pausemenu_update() call. That ordering is what makes a plain
     * switch-case sufficient with no extra "have we drawn a frame yet" flag:
     *   frame N:   OPTIONS case sees A pressed, sets state to this value,
     *              returns. This frame's draw calls run next and show
     *              "Checking for updates..." because the state is already
     *              this value by the time they run.
     *   frame N+1: pausemenu_update() is called again. Landing in THIS case
     *              at all is only possible because frame N's draw already
     *              happened in between -- so it is safe to call
     *              update_check() (and build the QR code from its result)
     *              right here, synchronously, and move to
     *              PAUSE_MENU_UPDATE_RESULT before this frame's own draw
     *              calls run.
     * See pausemenu.c's pausemenu_update() for the actual switch, and this
     * module's host probe for a test that asserts update_check() has NOT
     * run by the end of frame N and HAS run by the end of frame N+1. */
    PAUSE_MENU_UPDATE_CHECKING,

    PAUSE_MENU_UPDATE_RESULT,        /* result shown; B returns to Options    */
} PauseMenuState;

/* Root-level items, in the order they are shown and cycled. steve's own
 * words for this feature ("resume options and quit") give the order. */
typedef enum PauseRootItem {
    PAUSE_ROOT_RESUME = 0,
    PAUSE_ROOT_OPTIONS,
    PAUSE_ROOT_QUIT,
    PAUSE_ROOT_ITEM_COUNT
} PauseRootItem;

/* See this header's file comment for why this is caller-owned rather than a
 * module-internal static like hud.c's/debugdraw.c's state. */
typedef struct PauseMenu {
    bool open;                 /* true for every state except PAUSE_MENU_CLOSED --
                                 * kept as its own field (rather than derived
                                 * from `state != PAUSE_MENU_CLOSED` at every
                                 * call site) so pausemenu_is_open() is a
                                 * trivial, obviously-correct accessor. */
    PauseMenuState state;
    int root_cursor;           /* a PauseRootItem value; which root item is
                                 * highlighted while state == PAUSE_MENU_ROOT */
    bool quit_requested;       /* latched true by choosing Quit at the root;
                                 * drained (read-and-clear) by
                                 * pausemenu_take_quit_request() so main.c
                                 * never has to remember to reset it itself */

    /* Meaningful only once `state` has reached PAUSE_MENU_UPDATE_RESULT at
     * least once (i.e. after a completed update_check()) -- both are
     * zeroed/size==0 by pausemenu_init() until then, which is itself a safe
     * "nothing to show" value for both structs' own drawing/reading rules
     * (UpdateResult has no defined meaning before update_check() fills it;
     * QrCode.size == 0 means "do not draw", per qr.h). */
    UpdateResult update_result;
    QrCode update_qr;          /* built from update_result.release_url the
                                 * moment the check completes -- see
                                 * pausemenu.c's PAUSE_MENU_UPDATE_CHECKING
                                 * case. Not rebuilt on every draw call: QR
                                 * encoding is real work (Reed-Solomon), and
                                 * the result never changes between draws of
                                 * the same completed check. */
} PauseMenu;

/* Resets `menu` to the closed state and (on __3DS__) allocates this module's
 * own citro2d text buffers. MUST be called after renderer_init() -- see this
 * header's call-order note above. Safe to call once at startup; calling it
 * again re-zeroes `menu` (closing the menu, discarding any in-progress
 * update result) but does not leak or re-allocate the text buffers if they
 * are already up, same "both or neither, and idempotent" policy hud_init
 * uses. `menu` may be NULL to allocate only the module's own draw resources
 * without touching a caller's struct (not the expected use, but harmless). */
void pausemenu_init(PauseMenu *menu);

/* Frees this module's own citro2d text buffers (no-op on host or if never
 * initialized). Does not touch `menu` itself -- it is caller-owned memory
 * (stack or .bss), not something this module allocated. `menu` is accepted
 * only for signature symmetry with pausemenu_init and is otherwise unused. */
void pausemenu_shutdown(PauseMenu *menu);

/* Advances the state machine by exactly one frame's worth of input. Pure
 * logic -- no drawing, no citro2d/citro3d call, fully host-compilable and
 * unit-testable (see this module's verification notes). Call exactly once
 * per rendered frame; see PAUSE_MENU_UPDATE_CHECKING's comment above for why
 * "exactly once per frame" is a real constraint, not a suggestion.
 *
 * `keys_down` is a plain "just pressed this frame" button bitmask -- pass
 * (uint32_t)hidKeysDown() on real hardware. uint32_t rather than libctru's
 * own `u32` for the same reason debugdraw.h uses uint32_t for its rgba
 * parameter: so this header has zero dependency on 3ds.h and stays
 * includable, unmodified, from the host build. pausemenu.c interprets only
 * KEY_A / KEY_B / KEY_SELECT / KEY_DUP / KEY_DDOWN's bit positions, mirrored
 * internally as plain hex constants for the same reason -- see pausemenu.c's
 * header comment for exactly which bits and why that mirroring is safe.
 *
 * Behaviour summary (see pausemenu.c for the full switch):
 *   closed:            SELECT opens the menu at the root.
 *   root:              D-pad up/down move the highlight; A activates the
 *                       highlighted item (Resume closes the menu, Options
 *                       goes to the options screen, Quit latches
 *                       quit_requested and closes the menu); SELECT closes
 *                       the menu unconditionally. B does nothing here --
 *                       there is no level above the root to go back to;
 *                       SELECT is the root's own exit, per steve's original
 *                       "select opens/closes the pause menu" framing.
 *   options:           A runs the update check (enters the checking state);
 *                       B returns to the root.
 *   update (checking):  no input is read -- see the PAUSE_MENU_UPDATE_CHECKING
 *                       comment; this state's ONLY job is to let one frame
 *                       render before the blocking check runs on the next
 *                       call.
 *   update (result):    B returns to the options screen. */
void pausemenu_update(PauseMenu *menu, uint32_t keys_down);

/* True whenever the menu is showing (any state except PAUSE_MENU_CLOSED).
 * main.c polls this once per frame to decide whether to freeze physics/input
 * for that frame and whether to call pausemenu_draw_bottom() instead of
 * hud_draw() -- see this header's call-order note above. NULL-safe (returns
 * false). */
bool pausemenu_is_open(const PauseMenu *menu);

/* Read-and-clear: returns whatever quit_requested currently holds, then
 * clears it back to false. main.c calls this once per frame and breaks its
 * frame loop on a true result -- structured as take-not-peek specifically so
 * main.c cannot forget to reset the flag itself and act on a stale Quit
 * choice a second time. NULL-safe (returns false). */
bool pausemenu_take_quit_request(PauseMenu *menu);

/* Draws the menu itself (title, list, highlighted item) on the TOP screen,
 * OVER whatever 3D scene main.c already drew this frame -- it does NOT clear
 * the top target, only overlays a translucent panel on it. A no-op if
 * !pausemenu_is_open(menu) (still clears this module's own top-screen text
 * buffer first regardless -- see pausemenu.c's C2D_TextBufClear placement
 * note) or on the host build.
 *
 * MUST be called inside a renderer_frame_begin/renderer_frame_end bracket,
 * targeting the SAME top-screen render target renderer_get_target() and
 * debugdraw.h's text already used this frame -- see this header's call-order
 * note above for exactly where in main.c's frame this belongs and why it
 * depends on debugdraw_frame_end() having already run this frame. */
void pausemenu_draw_top(const PauseMenu *menu);

/* Draws the update-check detail (current vs latest version, the status/error
 * text, "B back") and, once a completed check produced a release URL, a
 * scannable QR code of it -- on the BOTTOM screen. Uses hud_get_target()
 * (does NOT create a second render target for the bottom screen -- see
 * hud.h's note on why that would be a bug) and, like hud_draw(), clears and
 * takes over that target for the rest of the frame.
 *
 * Draws only when menu->state == PAUSE_MENU_UPDATE_RESULT; for every other
 * open state it clears the target to a plain panel with just "PAUSED" (so
 * the bottom screen does not show stale telemetry, or a stale old update
 * result, while the player is elsewhere in the menu) and returns. A no-op
 * (still clears this module's own bottom-screen text buffer first -- same
 * reasoning as pausemenu_draw_top) if !pausemenu_is_open(menu) or on host.
 *
 * MUST be called inside a renderer_frame_begin/renderer_frame_end bracket.
 * Call this INSTEAD OF hud_draw() for any frame where pausemenu_is_open()
 * is true -- see this header's call-order note above. */
void pausemenu_draw_bottom(const PauseMenu *menu);

#ifdef __cplusplus
}
#endif

#endif /* DIRT2_UI_PAUSEMENU_H */
