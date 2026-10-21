#include "conv.h"
#include "config.h"

static int32_t round_f(float x) { return (int32_t)(x < 0 ? x - 0.5f : x + 0.5f); }

static uint16_t clamp_code(int32_t c)
{
    if (c < 0) return 0;
    if (c > 4095) return 4095;
    return (uint16_t)c;
}

/* Node equation at FB (held at 0.8 V by the LM5117):
 *   (Vout - 0.8)/R11 = 0.8/R12 + (0.8 - Vdac)/R27
 * MCP4725: Vdac = VDD * code / 4096 */
uint16_t conv_vout_to_code(int32_t vout_mv, int32_t vdd_mv)
{
    float vout = (float)vout_mv / 1000.0f;
    float vdac = FB_VREF_V - R27_OHM * ((vout - FB_VREF_V) / R11_OHM - FB_VREF_V / R12_OHM);
    float code = vdac * 4096.0f / ((float)vdd_mv / 1000.0f);
    return clamp_code(round_f(code));
}

int32_t conv_code_to_vout(uint16_t code, int32_t vdd_mv)
{
    float vdac = (float)vdd_mv / 1000.0f * (float)code / 4096.0f;
    float vout = FB_VREF_V + R11_OHM * (FB_VREF_V / R12_OHM + (FB_VREF_V - vdac) / R27_OHM);
    return round_f(vout * 1000.0f);
}

/* Comparator reference = Vdac * R32/(R31+R32); trips at I*RSENSE above it */
uint16_t conv_trip_to_code(int32_t i_ma, int32_t vdd_mv)
{
    float vref = (float)i_ma / 1000.0f * RSENSE_OHM;
    float vdac = vref * (R31_OHM + R32_OHM) / R32_OHM;
    return clamp_code(round_f(vdac * 4096.0f / ((float)vdd_mv / 1000.0f)));
}

int32_t conv_code_to_trip(uint16_t code, int32_t vdd_mv)
{
    float vdac = (float)vdd_mv / 1000.0f * (float)code / 4096.0f;
    float vref = vdac * R32_OHM / (R31_OHM + R32_OHM);
    return round_f(vref / RSENSE_OHM * 1000.0f);
}

int32_t conv_adc_to_mv(uint32_t raw, int32_t vdd_mv)
{
    return (int32_t)((raw * (uint32_t)vdd_mv + 2048u) / 4096u);
}

/* PA4 node: (VIN - V)/R17 + Ihyst = V/R18, Ihyst = 20 uA only when V > 1.25 V */
int32_t conv_vin_from_pin(int32_t pin_mv)
{
    float v = (float)pin_mv / 1000.0f;
    float vin = v * (1.0f + R17_OHM / R18_OHM);
    if (v > UVLO_THRESH_V) vin -= UVLO_HYST_A * R17_OHM;
    if (vin < 0) vin = 0;
    return round_f(vin * 1000.0f);
}

int32_t conv_icm_from_pin(int32_t pin_mv)
{
    return round_f((float)pin_mv / (LM5117_RS_OHM * LM5117_AS));
}

/* Internal reference is typically 1.20 V */
int32_t conv_vdd_from_vrefint(uint32_t raw)
{
    if (raw == 0) return VDD_NOMINAL_MV;
    return (int32_t)((1200UL * 4096UL + raw / 2u) / raw);
}

/* Shunt LSB 2.5 uV across 0.033 ohm -> 75.76 uA per count */
int32_t conv_ina_shunt_to_ma(int16_t raw)
{
    float i = (float)raw * 2.5e-6f / RSENSE_OHM;
    return round_f(i * 1000.0f);
}

int32_t conv_ina_bus_to_mv(uint16_t raw)
{
    return (int32_t)(((uint32_t)raw * 5u + 2u) / 4u);   /* 1.25 mV/LSB */
}

pwm_timing_t conv_pwm_timing(uint32_t timer_clk, uint32_t freq_hz, uint32_t duty_x10, uint32_t duty_max_x10)
{
    pwm_timing_t t;
    if (freq_hz < FREQ_MIN_HZ) freq_hz = FREQ_MIN_HZ;
    if (freq_hz > FREQ_MAX_HZ) freq_hz = FREQ_MAX_HZ;
    if (duty_x10 > duty_max_x10) duty_x10 = duty_max_x10;

    uint32_t ticks = (timer_clk + freq_hz / 2u) / freq_hz;
    t.psc = (ticks - 1u) / 65536u;
    uint32_t period = (ticks + (t.psc + 1u) / 2u) / (t.psc + 1u);
    if (period > 65536u) period = 65536u;
    t.arr = period - 1u;
    t.ccr = (period * duty_x10 + 500u) / 1000u;
    /* rounding must never push past the duty ceiling */
    uint32_t ccr_max = (period * duty_max_x10) / 1000u;
    if (t.ccr > ccr_max) t.ccr = ccr_max;
    return t;
}
