/*
 * ui.c - OLED + rotary encoder interface.
 *
 *   line 0  state, frequency, duty          RUN  20.00kHz D10.0%
 *   line 1  primary rail V and I            Vp 12.03V  I 0.512A
 *   line 2  power and input voltage         P  6.16W  Vin 30.1V
 *   line 3  menu item  (inverted = editing) > Vset       12.00 V
 *
 *   turn          scroll menu / change value while editing
 *   short press   start / stop editing the item (or run "Save")
 *   long press    output ON/OFF; in FAULT it clears the fault
 */
#include "ui.h"
#include "gfx.h"
#include "xfmt.h"
#include "control.h"
#include "settings.h"
#include "config.h"
#include "clock.h"
#include "ssd1306.h"

enum { IT_VSET, IT_FREQ, IT_DUTY, IT_ILIM, IT_TRIP, IT_SS, IT_SAVE, IT_COUNT };
static const char *const k_item_name[IT_COUNT] = {
    "Vset", "Freq", "Duty", "I limit", "OCP trip", "Softstart", "Save",
};

static uint8_t  s_item;
static bool     s_editing;
static uint32_t s_last_input_ms;
static char     s_msg[GFX_COLS + 1];
static uint32_t s_msg_until;

static bool     s_oled;
static uint8_t  s_page = GFX_PAGES;
static uint32_t s_last_frame, s_last_retry;

void ui_message(const char *msg, uint32_t now)
{
    xsnprintf(s_msg, sizeof s_msg, "%s", msg);
    s_msg_until = now + 1500u;
}

static int32_t clamp32(int32_t v, int32_t lo, int32_t hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static uint32_t freq_step(uint32_t f)
{
    if (f < 2000u)  return 10u;
    if (f < 10000u) return 50u;
    if (f < 50000u) return 100u;
    return 500u;
}

static void adjust(int steps, bool fast)
{
    int32_t m = fast ? 10 : 1;
    switch (s_item) {
    case IT_VSET:
        g_set.vset_mv = (uint16_t)clamp32(g_set.vset_mv + steps * 50 * m, VSET_MIN_MV, VSET_MAX_MV);
        break;
    case IT_FREQ:
        g_set.freq_hz = (uint32_t)clamp32((int32_t)g_set.freq_hz + steps * (int32_t)freq_step(g_set.freq_hz) * m,
                                          FREQ_MIN_HZ, FREQ_MAX_HZ);
        break;
    case IT_DUTY:
        g_set.duty_x10 = (uint16_t)clamp32(g_set.duty_x10 + steps * 5 * m, 0, DUTY_MAX_X10);
        break;
    case IT_ILIM:
        g_set.ilim_ma = (uint16_t)clamp32(g_set.ilim_ma + steps * 10 * m, ILIM_MIN_MA, ILIM_MAX_MA);
        break;
    case IT_TRIP:
        g_set.trip_ma = (uint16_t)clamp32(g_set.trip_ma + steps * 50 * m, TRIP_MIN_MA, TRIP_MAX_MA);
        (void)control_apply_trip();
        break;
    case IT_SS:
        g_set.ss_ms = (uint16_t)clamp32(g_set.ss_ms + steps * 100 * m, 0, SS_MAX_MS);
        break;
    default:
        break;
    }
}

static void item_value(char *buf, uint32_t n)
{
    switch (s_item) {
    case IT_VSET: xsnprintf(buf, n, "%.2k V", g_set.vset_mv); break;
    case IT_FREQ:
        if (g_set.freq_hz < 100000u) xsnprintf(buf, n, "%.2k kHz", g_set.freq_hz);
        else                         xsnprintf(buf, n, "%.1k kHz", g_set.freq_hz);
        break;
    case IT_DUTY: xsnprintf(buf, n, "%.1k %%", g_set.duty_x10 * 100); break;
    case IT_ILIM: xsnprintf(buf, n, "%.3k A", g_set.ilim_ma); break;
    case IT_TRIP: xsnprintf(buf, n, "%.2k A", g_set.trip_ma); break;
    case IT_SS:   xsnprintf(buf, n, "%.1k s", g_set.ss_ms); break;
    case IT_SAVE: xsnprintf(buf, n, "press"); break;
    default:      buf[0] = '\0'; break;
    }
}

void ui_handle(int steps, bool fast, btn_event_t ev, uint32_t now)
{
    if (steps || ev != BTN_NONE) s_last_input_ms = now;
    if (s_editing && (now - s_last_input_ms) > UI_EDIT_TIMEOUT_MS) s_editing = false;

    if (steps) {
        if (s_editing) {
            adjust(steps, fast);
        } else {
            int32_t it = ((int32_t)s_item + steps) % IT_COUNT;
            if (it < 0) it += IT_COUNT;
            s_item = (uint8_t)it;
        }
    }

    if (ev == BTN_SHORT) {
        if (s_item == IT_SAVE) {
            if (control_active())         ui_message("Turn output off first", now);
            else if (settings_save())     ui_message("Settings saved", now);
            else                          ui_message("Save FAILED", now);
        } else {
            s_editing = !s_editing;
        }
    } else if (ev == BTN_LONG) {
        s_editing = false;
        if (g_st.state == ST_FAULT) {
            ui_message(control_clear_fault() ? "Fault cleared" : "Still faulted", now);
        } else if (control_active()) {
            control_stop();
            ui_message("Output OFF", now);
        } else {
            const char *why = control_start();
            ui_message(why ? why : "Output ON", now);
        }
    }
}

void ui_render(uint32_t now)
{
    gfx_clear();

    /* line 0: state / frequency / duty */
    char fbuf[12];
    if (g_set.freq_hz < 100000u) xsnprintf(fbuf, sizeof fbuf, "%.2kkHz", g_set.freq_hz);
    else                         xsnprintf(fbuf, sizeof fbuf, "%.1kkHz", g_set.freq_hz);

    if (g_st.state == ST_FAULT) {
        gfx_printf(0, "FAULT:%-5s hold=clr", control_fault_name(g_st.fault));
        gfx_invert_line(0);
    } else {
        const char *st = control_state_name(g_st.state);
        if (g_st.state == ST_RUN && g_st.cc_active) st = "CC";
        if (control_active() && g_st.vin_mv < VIN_UVLO_MV) st = "UVLO";
        uint32_t duty = control_active() ? g_st.duty_now_x10 : g_set.duty_x10;
        gfx_printf(0, "%-4s %8s D%.1k%%", st, fbuf, (int32_t)duty * 100);
    }

    /* lines 1-2: measurements */
    if (g_st.ina_valid) {
        gfx_printf(1, "Vp%6.2kV  I%6.3kA", g_st.vbus_mv, g_st.i_ma);
        gfx_printf(2, "P%6.2kW  Vin%5.1kV", g_st.p_mw, g_st.vin_mv);
    } else {
        gfx_printf(1, "Vp  --.--V  I --.---A");
        gfx_printf(2, "P  --.--W  Vin%5.1kV", g_st.vin_mv);
    }

    /* line 3: message, or the current menu item */
    if (s_msg[0] && (int32_t)(s_msg_until - now) > 0) {
        gfx_printf(3, "%s", s_msg);
    } else {
        s_msg[0] = '\0';
        char val[12];
        item_value(val, sizeof val);
        gfx_printf(3, "%c %-9s%10s", s_editing ? '*' : '>', k_item_name[s_item], val);
        if (s_editing) gfx_invert_line(3);
    }
}

void ui_init(bool oled_present)
{
    s_oled = oled_present;
    s_item = IT_VSET;
}

/* One 128-byte page is sent per call, so a frame never blocks the main
 * loop for more than ~7 ms. The OLED isn't fitted yet on early boards:
 * if it is missing, retry every 2 s so it can be plugged in later. */
void ui_tick(uint32_t now)
{
    bool fast;
    int steps = input_enc_steps(&fast);
    btn_event_t ev = input_take_button();
    ui_handle(steps, fast, ev, now);

    if (!s_oled) {
        if (now - s_last_retry > 2000u) {
            s_last_retry = now;
            s_oled = ssd1306_init();
            s_page = GFX_PAGES;
        }
        return;
    }

    if (s_page < GFX_PAGES) {
        if (!ssd1306_flush_page(s_page)) { s_oled = false; return; }
        s_page++;
    } else if (now - s_last_frame >= UI_FRAME_MS) {
        s_last_frame = now;
        ui_render(now);
        s_page = 0;
    }
}
