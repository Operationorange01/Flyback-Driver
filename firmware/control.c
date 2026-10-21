/*
 * control.c - power stage state machine.
 *
 *   OFF ──start──> STARTING ──ramp done──> RUN
 *    ^                 |                     |
 *    |                 └──── any fault ──────┴──> FAULT ──clear──> OFF
 *    └────────────────────── stop ──────────────────┘
 *
 * Every CTRL_PERIOD_MS:
 *  - poll the INA226 (new sample every ~35 ms) and the ADC
 *  - soft start: duty and rail both ramp from minimum over g_set.ss_ms
 *  - constant-current limiter: pulls the rail command down while the
 *    measured average primary current exceeds g_set.ilim_ma
 *  - voltage trim: slow integral correction so the measured rail equals
 *    the setpoint (absorbs resistor and reference tolerances)
 *  - over-voltage and sensor-timeout protection
 *
 * The hardware OCP (comparator + latch + TIM1 break) does not depend on
 * any of this code.
 */
#include "control.h"
#include "settings.h"
#include "board.h"
#include "clock.h"
#include "config.h"
#include "conv.h"
#include "devices.h"
#include "adc.h"
#include "pwm.h"
#include "uart.h"

ctrl_status_t g_st;

static volatile bool s_hw_trip;
static uint32_t s_t_start, s_last_ina_ms, s_tick;
static int32_t  s_v_cc, s_v_slewed;
static uint32_t s_duty_slewed;
static uint32_t s_pwm_freq_applied = 0xFFFFFFFFu, s_pwm_duty_applied = 0xFFFFFFFFu;
static uint8_t  s_ovp_cnt, s_ovp_abs_cnt, s_cc_fail_cnt, s_dac_err_cnt;

#define VSET_CODE_OFF 4095u   /* highest code = lowest possible rail */

static int32_t vdd_for_dac(void)
{
#if USE_VREFINT_FOR_VDD
    return g_st.vdd_mv;
#else
    return VDD_NOMINAL_MV;
#endif
}

const char *control_state_name(ctrl_state_t s)
{
    static const char *const n[] = { "OFF", "SOFT", "RUN", "FAULT" };
    return n[s];
}

const char *control_fault_name(fault_t f)
{
    static const char *const n[] = { "none", "OCP", "OVP", "I2C", "CLIM", "BOOT" };
    return n[f];
}

bool control_active(void)
{
    return g_st.state == ST_STARTING || g_st.state == ST_RUN;
}

/* Called from TIM1_BRK_IRQHandler */
void control_hw_trip_isr(void)
{
    s_hw_trip = true;
#if TRIP_DISABLES_BUCK
    buck_enable(false);
#endif
}

static bool dac_vset(uint16_t code)
{
    if (code == g_st.dac_vset_code) return true;
    if (!mcp4725_write(I2C_ADDR_DAC_VSET, code)) return false;
    g_st.dac_vset_code = code;
    return true;
}

static void apply_pwm(uint32_t freq, uint32_t duty)
{
    if (freq != s_pwm_freq_applied || duty != s_pwm_duty_applied) {
        pwm_set(freq, duty);
        s_pwm_freq_applied = freq;
        s_pwm_duty_applied = duty;
    }
}

static void outputs_off(void)
{
    pwm_disarm();
    s_pwm_duty_applied = 0;
    buck_enable(false);
    g_st.duty_now_x10 = 0;
    g_st.cc_active = false;
    g_st.vtrim_mv = 0;
    /* forced write: park the rail command at its minimum */
    g_st.dac_vset_code = 0xFFFF;
    (void)dac_vset(VSET_CODE_OFF);
    g_st.vcmd_mv = conv_code_to_vout(VSET_CODE_OFF, vdd_for_dac());
}

static void enter_fault(fault_t f)
{
    outputs_off();
    if (g_st.state != ST_FAULT || g_st.fault == FLT_NONE) g_st.fault = f;
    g_st.state = ST_FAULT;
    uart_printf("\n!! FAULT: %s  (Vp=%.2kV I=%.3kA)\n",
                control_fault_name(f), g_st.vbus_mv, g_st.i_ma);
}

bool control_apply_trip(void)
{
    uint16_t code = conv_trip_to_code(g_set.trip_ma, vdd_for_dac());
    if (g_st.dev_dac_t && mcp4725_write(I2C_ADDR_DAC_TRIP, code)) {
        g_st.dac_trip_code = code;
        return true;
    }
    if (control_active()) enter_fault(FLT_COMM);
    return false;
}

/* Clears the 74HC SR latch and the TIM1 break flag. Only succeeds when
 * the comparator no longer sees over-current. */
static bool hw_latch_clear(void)
{
    pwm_disarm();
    if (fault_detect_active()) return false;
    fault_latch_reset_pulse();
    TIM1->SR = ~TIM_SR_BIF;
    if (TIM1->SR & TIM_SR_BIF) return false;
    TIM1->DIER |= TIM_DIER_BIE;
    s_hw_trip = false;
    return true;
}

static bool probe_devices(void)
{
    uint16_t dac, ee;
    g_st.dev_ina   = ina226_init();
    g_st.dev_dac_v = mcp4725_read(I2C_ADDR_DAC_VSET, &dac, &ee);
    if (g_st.dev_dac_v && ee != VSET_CODE_OFF) {
        /* Make the DAC's power-on value the LOWEST rail voltage, so the
         * board is safe even before the MCU runs. Written once. */
        uart_puts("Setting U3 EEPROM power-on code to minimum rail\n");
        g_st.dev_dac_v = mcp4725_write_eeprom(I2C_ADDR_DAC_VSET, VSET_CODE_OFF);
    }
    g_st.dev_dac_t = mcp4725_read(I2C_ADDR_DAC_TRIP, &dac, &ee);
    return g_st.dev_ina && g_st.dev_dac_v && g_st.dev_dac_t;
}

void control_init(void)
{
    adc_values_t a;
    adc_update(&a);
    g_st.vdd_mv = a.vdd_mv;
    g_st.vin_mv = a.vin_mv;
    g_st.icm_ma = a.icm_ma;

    bool ok = probe_devices();
    outputs_off();
    (void)control_apply_trip();
    bool latch_ok = hw_latch_clear();

    uart_printf("INA226 %s, DAC Vset %s, DAC trip %s, OCP latch %s\n",
                g_st.dev_ina ? "ok" : "MISSING", g_st.dev_dac_v ? "ok" : "MISSING",
                g_st.dev_dac_t ? "ok" : "MISSING", latch_ok ? "clear" : "STUCK");

    if (!ok)            { g_st.state = ST_FAULT; g_st.fault = FLT_BOOT; }
    else if (!latch_ok) { g_st.state = ST_FAULT; g_st.fault = FLT_HW_OCP; }
    else                { g_st.state = ST_OFF;   g_st.fault = FLT_NONE; }
    s_last_ina_ms = millis();
}

const char *control_start(void)
{
    if (g_st.state == ST_FAULT) return "Fault: clear first";
    if (control_active())       return 0;
    if (!g_st.dev_ina || !g_st.dev_dac_v || !g_st.dev_dac_t) return "I2C device missing";

    settings_clamp();
    g_st.dac_vset_code = 0xFFFF;
    if (!dac_vset(VSET_CODE_OFF)) return "Vset DAC no response";
    if (!control_apply_trip())   return "Trip DAC no response";
    if (!hw_latch_clear())       return "OCP comparator high";

    s_v_cc        = g_set.vset_mv;
    s_v_slewed    = VSET_MIN_MV;
    s_duty_slewed = 0;
    s_ovp_cnt = s_ovp_abs_cnt = s_cc_fail_cnt = s_dac_err_cnt = 0;
    g_st.vtrim_mv = 0;
    g_st.cc_active = false;

    apply_pwm(g_set.freq_hz, 0);
    buck_enable(true);
    if (!pwm_arm()) { outputs_off(); return "Break input active"; }

    s_t_start = millis();
    s_last_ina_ms = s_t_start;
    g_st.state = ST_STARTING;
    return 0;
}

void control_stop(void)
{
    outputs_off();
    if (g_st.state != ST_FAULT) g_st.state = ST_OFF;
}

bool control_clear_fault(void)
{
    if (g_st.state != ST_FAULT) return true;
    if (g_st.fault == FLT_BOOT || g_st.fault == FLT_COMM) {
        if (!probe_devices()) return false;
    }
    outputs_off();
    if (!control_apply_trip()) return false;
    if (!hw_latch_clear()) return false;
    g_st.state = ST_OFF;
    g_st.fault = FLT_NONE;
    return true;
}

static int32_t move_toward(int32_t x, int32_t target, int32_t step)
{
    if (x < target) return (target - x > step) ? x + step : target;
    if (x > target) return (x - target > step) ? x - step : target;
    return x;
}

static void update_leds(uint32_t now)
{
    bool blink_slow = (now / 500u) & 1u;
    bool blink_fast = (now / 125u) & 1u;
    switch (g_st.state) {
    case ST_OFF:      led_set(LED_STATUS, false);      break;
    case ST_STARTING: led_set(LED_STATUS, blink_fast); break;
    case ST_RUN:      led_set(LED_STATUS, true);       break;
    case ST_FAULT:    led_set(LED_STATUS, false);      break;
    }
    /* Fault LED: solid for a latched fault, blinking for missing devices */
    if (g_st.state == ST_FAULT)
        led_set(LED_FAULT, g_st.fault == FLT_BOOT ? blink_slow : true);
    else
        led_set(LED_FAULT, false);
    led_set(LED_CURLIM, g_st.cc_active);
}

/* Runs on each new INA226 sample (~every 35 ms) while active */
static void on_new_sample(void)
{
    int32_t vset = g_set.vset_mv;
    int32_t i = g_st.i_ma, ilim = g_set.ilim_ma;

    /* constant-current limiter */
    if (i > ilim) {
        int32_t drop = (i - ilim) * CC_GAIN_MV_PER_MA;
        if (drop > CC_MAX_STEP_MV) drop = CC_MAX_STEP_MV;
        s_v_cc -= drop;
    } else {
        s_v_cc += CC_RECOVER_MV;
    }
    if (s_v_cc > vset) s_v_cc = vset;
    if (s_v_cc < VSET_MIN_MV) s_v_cc = VSET_MIN_MV;

    if (s_v_cc <= VSET_MIN_MV && i > ilim + ilim / 10) {
        if (++s_cc_fail_cnt >= CC_FAIL_SAMPLES) { enter_fault(FLT_CC); return; }
    } else {
        s_cc_fail_cnt = 0;
    }

    /* over-voltage */
    if (g_st.vbus_mv > VOUT_ABS_MAX_MV) {
        if (++s_ovp_abs_cnt >= 2) { enter_fault(FLT_OVP); return; }
    } else {
        s_ovp_abs_cnt = 0;
    }
    if (g_st.vbus_mv > s_v_slewed + OVP_MARGIN_MV) {
        if (++s_ovp_cnt >= OVP_SAMPLES) { enter_fault(FLT_OVP); return; }
    } else {
        s_ovp_cnt = 0;
    }

#if VTRIM_ENABLE
    /* integral trim, only in steady CV operation */
    int32_t err = s_v_slewed - g_st.vbus_mv;
    if (g_st.state == ST_RUN && !g_st.cc_active && s_v_slewed == vset &&
        err > -VTRIM_WINDOW_MV && err < VTRIM_WINDOW_MV) {
        g_st.vtrim_mv += err / 4;
        if (g_st.vtrim_mv >  VTRIM_MAX_MV) g_st.vtrim_mv =  VTRIM_MAX_MV;
        if (g_st.vtrim_mv < -VTRIM_MAX_MV) g_st.vtrim_mv = -VTRIM_MAX_MV;
    }
#endif
}

void control_tick(void)
{
    uint32_t now = millis();
    s_tick++;

    /* 1. hardware over-current latched by the break interrupt */
    if (s_hw_trip) {
        s_hw_trip = false;
        g_st.trip_count++;
        if (g_st.state != ST_FAULT) enter_fault(FLT_HW_OCP);
    }

    /* 2. sensors */
    bool fresh = false;
    if (g_st.dev_ina) {
        ina226_sample_t s;
        if (ina226_poll(&s, &fresh)) {
            if (fresh) {
                g_st.vbus_mv = s.vbus_mv;
                g_st.i_ma    = s.i_ma;
                g_st.p_mw    = s.p_mw;
                g_st.ina_valid = true;
                s_last_ina_ms = now;
            }
        }
    }
    if ((now - s_last_ina_ms) > SENSOR_TIMEOUT_MS) g_st.ina_valid = false;

    if ((s_tick & 1u) == 0u) {
        adc_values_t a;
        adc_update(&a);
        g_st.vdd_mv = a.vdd_mv;
        g_st.vin_mv = a.vin_mv;
        g_st.icm_ma = a.icm_ma;
    }

    if (!control_active()) {
        apply_pwm(g_set.freq_hz, 0);
        update_leds(now);
        return;
    }

    if (!g_st.ina_valid) { enter_fault(FLT_COMM); update_leds(now); return; }

    /* 3. soft start ramp */
    int32_t  v_ramp;
    uint32_t duty_target;
    if (g_st.state == ST_STARTING) {
        uint32_t el = now - s_t_start;
        uint32_t frac = (g_set.ss_ms == 0u || el >= g_set.ss_ms) ? 1000u : (el * 1000u) / g_set.ss_ms;
        v_ramp = VSET_MIN_MV + (int32_t)(((int64_t)(g_set.vset_mv - VSET_MIN_MV) * frac) / 1000);
        duty_target = (g_set.duty_x10 * frac) / 1000u;
        if (frac >= 1000u) g_st.state = ST_RUN;
    } else {
        v_ramp = g_set.vset_mv;
        duty_target = g_set.duty_x10;
    }

    /* 4. limiter, OVP, trim on each fresh measurement */
    if (fresh) {
        on_new_sample();
        if (g_st.state == ST_FAULT) { update_leds(now); return; }
    }

    /* 5. rail command = min(ramp, CC limit), slew limited */
    int32_t target = v_ramp < s_v_cc ? v_ramp : s_v_cc;
    g_st.cc_active = s_v_cc < v_ramp;
    s_v_slewed = move_toward(s_v_slewed, target, VSLEW_MV_PER_TICK);
    g_st.vcmd_mv = s_v_slewed;

    uint16_t code = conv_vout_to_code(s_v_slewed + g_st.vtrim_mv, vdd_for_dac());
    if (!dac_vset(code)) {
        if (++s_dac_err_cnt >= 3) { enter_fault(FLT_COMM); update_leds(now); return; }
    } else {
        s_dac_err_cnt = 0;
    }

    /* 6. PWM, duty slew limited */
    s_duty_slewed = (uint32_t)move_toward((int32_t)s_duty_slewed, (int32_t)duty_target, DUTY_SLEW_PER_TICK);
    g_st.duty_now_x10 = s_duty_slewed;
    apply_pwm(g_set.freq_hz, s_duty_slewed);

    update_leds(now);
}
