/*
 * cli.c - serial command interface on J1 (115200 8N1).
 * Type "help" for the list. Lines end with CR or LF.
 */
#include "cli.h"
#include "uart.h"
#include "xfmt.h"
#include "control.h"
#include "settings.h"
#include "config.h"
#include "clock.h"
#include "pwm.h"
#include "soft_i2c.h"
#include "conv.h"
#include <string.h>

static char     s_line[64];
static uint8_t  s_len;
static uint32_t s_stream_ms, s_last_stream;

static const char *const k_help =
    "Commands:\n"
    "  on | off             enable / disable output (soft start on enable)\n"
    "  clear                clear a latched fault\n"
    "  status               measurements and state\n"
    "  get                  show settings\n"
    "  vset <V>             primary rail, 6.5-24.5 V      e.g. vset 18.5\n"
    "  freq <Hz|k>          switching frequency            e.g. freq 25k\n"
    "  duty <%>             duty cycle, 0-50 %             e.g. duty 12.5\n"
    "  ilim <A>             average current limit (CC)     e.g. ilim 1.2\n"
    "  trip <A>             hardware OCP peak threshold    e.g. trip 2.5\n"
    "  ss <ms>              soft start time                e.g. ss 1500\n"
    "  save | load | defaults   settings in flash (output must be off to save)\n"
    "  stream <ms>          CSV telemetry every <ms>, 0 = stop\n"
    "  i2cscan              list devices on the I2C bus\n";

static void print_settings(void)
{
    uart_printf("vset %.2k V  freq %u Hz (actual %u)  duty %.1k %%\n",
                g_set.vset_mv, g_set.freq_hz, pwm_actual_freq(), g_set.duty_x10 * 100);
    uart_printf("ilim %.3k A  trip %.2k A  ss %u ms\n",
                g_set.ilim_ma, g_set.trip_ma, g_set.ss_ms);
}

static void print_status(void)
{
    uart_printf("state %s  fault %s  trips %u\n", control_state_name(g_st.state),
                control_fault_name(g_st.fault), g_st.trip_count);
    if (g_st.ina_valid)
        uart_printf("rail %.3k V  I %.3k A  P %.2k W\n", g_st.vbus_mv, g_st.i_ma, g_st.p_mw);
    else
        uart_puts("rail/current: no INA226 data\n");
    uart_printf("vin %.2k V  buck I(CM) %.2k A  vdd %.3k V\n", g_st.vin_mv, g_st.icm_ma, g_st.vdd_mv);
    uart_printf("cmd %.3k V  trim %d mV  duty %.1k %%  CC %s\n", g_st.vcmd_mv, g_st.vtrim_mv,
                (int32_t)g_st.duty_now_x10 * 100, g_st.cc_active ? "yes" : "no");
    uart_printf("DAC vset code %u (%.2k V)  DAC trip code %u (%.2k A)  i2c errors %u\n",
                g_st.dac_vset_code, conv_code_to_vout(g_st.dac_vset_code, VDD_NOMINAL_MV),
                g_st.dac_trip_code, conv_code_to_trip(g_st.dac_trip_code, VDD_NOMINAL_MV),
                i2c_error_count());
    if (g_st.vin_mv < VIN_UVLO_MV)
        uart_puts("note: VIN below LM5117 UVLO (~17.5 V) - buck will not switch\n");
}

static bool set_param(const char *name, const char *arg)
{
    int32_t v;
    if (!strcmp(name, "vset")) {
        if (!parse_fixed(arg, 3, &v) || v < VSET_MIN_MV || v > VSET_MAX_MV) return false;
        g_set.vset_mv = (uint16_t)v;
    } else if (!strcmp(name, "freq")) {
        size_t n = strlen(arg);
        bool k = n && (arg[n - 1] == 'k' || arg[n - 1] == 'K');
        if (!parse_fixed(arg, k ? 3 : 0, &v) || v < FREQ_MIN_HZ || v > FREQ_MAX_HZ) return false;
        g_set.freq_hz = (uint32_t)v;
    } else if (!strcmp(name, "duty")) {
        if (!parse_fixed(arg, 1, &v) || v < 0 || v > DUTY_MAX_X10) return false;
        g_set.duty_x10 = (uint16_t)v;
    } else if (!strcmp(name, "ilim")) {
        if (!parse_fixed(arg, 3, &v) || v < ILIM_MIN_MA || v > ILIM_MAX_MA) return false;
        g_set.ilim_ma = (uint16_t)v;
    } else if (!strcmp(name, "trip")) {
        if (!parse_fixed(arg, 3, &v) || v < TRIP_MIN_MA || v > TRIP_MAX_MA) return false;
        g_set.trip_ma = (uint16_t)v;
        (void)control_apply_trip();
    } else if (!strcmp(name, "ss")) {
        if (!parse_fixed(arg, 0, &v) || v < 0 || v > SS_MAX_MS) return false;
        g_set.ss_ms = (uint16_t)v;
    } else {
        return false;
    }
    return true;
}

static void i2c_scan(void)
{
    uart_puts("I2C:");
    for (uint8_t a = 0x08; a < 0x78; a++) {
        if (i2c_probe(a)) uart_printf(" 0x%x", a);
    }
    uart_puts("  (expect 0x3c OLED, 0x40 INA226, 0x60/0x61 DACs)\n");
}

static void run(char *line)
{
    char *cmd = line;
    while (*cmd == ' ') cmd++;
    char *arg = cmd;
    while (*arg && *arg != ' ') arg++;
    if (*arg) { *arg++ = '\0'; while (*arg == ' ') arg++; }
    for (char *p = cmd; *p; p++) if (*p >= 'A' && *p <= 'Z') *p = (char)(*p + 32);
    if (!*cmd) return;

    if (!strcmp(cmd, "help") || !strcmp(cmd, "?")) {
        uart_puts(k_help);
    } else if (!strcmp(cmd, "on")) {
        const char *why = control_start();
        uart_printf("%s\n", why ? why : "output ON");
    } else if (!strcmp(cmd, "off")) {
        control_stop();
        uart_puts("output OFF\n");
    } else if (!strcmp(cmd, "clear")) {
        uart_puts(control_clear_fault() ? "fault cleared\n" : "fault still present\n");
    } else if (!strcmp(cmd, "status")) {
        print_status();
    } else if (!strcmp(cmd, "get")) {
        print_settings();
    } else if (!strcmp(cmd, "save")) {
        if (control_active()) uart_puts("turn the output off first\n");
        else uart_puts(settings_save() ? "saved\n" : "save FAILED\n");
    } else if (!strcmp(cmd, "load")) {
        uart_puts(settings_load() ? "loaded\n" : "no valid settings, defaults loaded\n");
        (void)control_apply_trip();
        print_settings();
    } else if (!strcmp(cmd, "defaults")) {
        settings_defaults();
        (void)control_apply_trip();
        print_settings();
    } else if (!strcmp(cmd, "stream")) {
        int32_t v = 0;
        if (*arg && (!parse_fixed(arg, 0, &v) || v < 0)) { uart_puts("bad value\n"); return; }
        s_stream_ms = (v > 0 && v < 50) ? 50u : (uint32_t)v;
        if (s_stream_ms)
            uart_puts("t_ms,state,fault,freq_hz,duty_pct,vset_v,vcmd_v,vbus_v,i_a,p_w,vin_v,icm_a,cc,trips\n");
    } else if (!strcmp(cmd, "i2cscan")) {
        if (control_active()) uart_puts("turn the output off first\n");
        else i2c_scan();
    } else if (*arg && set_param(cmd, arg)) {
        print_settings();
    } else if (!strcmp(cmd, "set")) {
        /* "set vset 12" form */
        char *a2 = arg;
        while (*a2 && *a2 != ' ') a2++;
        if (*a2) { *a2++ = '\0'; while (*a2 == ' ') a2++; }
        if (set_param(arg, a2)) print_settings();
        else uart_puts("bad parameter or value out of range\n");
    } else {
        uart_puts("? unknown command or value out of range - type help\n");
    }
}

void cli_init(void)
{
    s_len = 0;
}

void cli_poll(void)
{
    int c;
    while ((c = uart_getc()) >= 0) {
        if (c == '\r' || c == '\n') {
            if (s_len) {
                uart_puts("\n");
                s_line[s_len] = '\0';
                run(s_line);
                s_len = 0;
            }
            uart_puts("> ");
        } else if (c == 0x08 || c == 0x7F) {
            if (s_len) { s_len--; uart_puts("\b \b"); }
        } else if (c >= 0x20 && c < 0x7F && s_len < sizeof(s_line) - 1u) {
            s_line[s_len++] = (char)c;
            uart_putc((char)c);
        }
    }
}

void cli_tick(uint32_t now)
{
    if (!s_stream_ms || (now - s_last_stream) < s_stream_ms) return;
    s_last_stream = now;
    uart_printf("%u,%s,%s,%u,%.1k,%.3k,%.3k,%.3k,%.3k,%.3k,%.2k,%.2k,%u,%u\n",
                now, control_state_name(g_st.state), control_fault_name(g_st.fault),
                g_set.freq_hz, (int32_t)g_st.duty_now_x10 * 100, g_set.vset_mv, g_st.vcmd_mv,
                g_st.vbus_mv, g_st.i_ma, g_st.p_mw, g_st.vin_mv, g_st.icm_ma,
                g_st.cc_active ? 1u : 0u, g_st.trip_count);
}
