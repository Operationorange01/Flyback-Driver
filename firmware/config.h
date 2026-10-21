/*
 * config.h - every tunable and every hardware constant in one place.
 *
 * Hardware values come from "Flyback driver netlist v2" (KiCad 10) and BOM.csv.
 * If you change a resistor on the board, change it here too.
 */
#ifndef CONFIG_H
#define CONFIG_H

#define FW_VERSION          "1.0.0"

/* ------------------------------------------------------------------ */
/* Clock: there is no crystal on the board (PD0/PD1 unconnected), so   */
/* the MCU runs from HSI/2 x 16 = 64 MHz (the HSI maximum on F103).    */
/* ------------------------------------------------------------------ */
#define SYSCLK_HZ           64000000UL

/* ------------------------------------------------------------------ */
/* Supply rail                                                         */
/* "3.3V" net is made by TPS62130 with R33 = 300k / R34 = 100k and     */
/* Vfb = 0.8 V  ->  0.8 * (1 + 300/100) = 3.20 V.  Both MCP4725 DACs   */
/* use VDD as their reference, so DAC math uses this value.            */
/* ------------------------------------------------------------------ */
#define VDD_NOMINAL_MV      3200
#define USE_VREFINT_FOR_VDD 0       /* 1 = use measured VDD (internal ref, +-3 %) */

/* ------------------------------------------------------------------ */
/* LM5117 primary-rail setpoint network                                */
/*   PRIMARY_RAIL --R11-- FB --R12-- SGND,   DAC U3 --R27-- FB          */
/*   Vout = 0.8 + R11 * (0.8/R12 + (0.8 - Vdac)/R27)                   */
/* Higher DAC code = LOWER output voltage.                             */
/* ------------------------------------------------------------------ */
#define FB_VREF_V           0.8f
#define R11_OHM             100000.0f
#define R12_OHM             4120.0f   /* BOM part CRCW08054K12 (schematic says 4.07k) */
#define R27_OHM             17400.0f

/* ------------------------------------------------------------------ */
/* Hardware over-current trip (LM393 U9A + 74HC latch U11)             */
/*   CUTOFF_VREF = Vdac(U10) * R32 / (R31 + R32)                       */
/*   trips when I_switch * RSENSE > CUTOFF_VREF                        */
/* ------------------------------------------------------------------ */
#define R31_OHM             33000.0f
#define R32_OHM             1000.0f
#define RSENSE_OHM          0.033f    /* CUR_SEN_R1, also the INA226 shunt */

/* ------------------------------------------------------------------ */
/* VIN sense on PA4 (shared with LM5117 UVLO pin)                      */
/*   VIN_PROT --R17-- UVLO/PA4 --R18-- SGND                            */
/* LM5117 sources 20 uA into this node once UVLO > 1.25 V.             */
/* ------------------------------------------------------------------ */
#define R17_OHM             130000.0f
#define R18_OHM             10000.0f
#define UVLO_HYST_A         20e-6f
#define UVLO_THRESH_V       1.25f
#define VIN_UVLO_MV         17500     /* 1.25 V * (1 + R17/R18): buck will not start below this */

/* LM5117 current monitor: CM -> R28/C27 -> PA5.  Vcm ~= Iout*RS*AS.    */
/* Only valid in continuous conduction; used for telemetry only.       */
#define LM5117_RS_OHM       0.015f    /* R19 */
#define LM5117_AS           10.0f

/* ------------------------------------------------------------------ */
/* I2C bus (PB6 = SCL, PB7 = SDA, bit-banged, 4.7k pull-ups)           */
/* ------------------------------------------------------------------ */
#define I2C_ADDR_DAC_VSET   0x60      /* U3  MCP4725A0, A0 = SGND -> sets PRIMARY_RAIL */
#define I2C_ADDR_DAC_TRIP   0x61      /* U10 MCP4725A0, A0 = 3V3  -> sets OCP threshold */
#define I2C_ADDR_INA226     0x40      /* U5, A0 = A1 = SGND */
#define I2C_ADDR_OLED       0x3C      /* U4 SSD1306 128x32 */
#define I2C_SPEED_HZ        200000UL

/* INA226: AVG = 16, VBUSCT = VSHCT = 1.1 ms, continuous shunt+bus.    */
/* One result every ~35 ms.  The switch current is pulsed, the INA226  */
/* integrates it, so the reading is the average primary current.      */
#define INA226_CONFIG       0x4527

/* ------------------------------------------------------------------ */
/* Operating limits (user settings are clamped to these)               */
/* ------------------------------------------------------------------ */
#define VSET_MIN_MV         6500      /* hardware floor is ~6.4 V at VDD = 3.2 V */
#define VSET_MAX_MV         24500     /* hardware ceiling is ~24.8 V */
#define VOUT_ABS_MAX_MV     27000     /* OVP: immediate fault above this */
#define OVP_MARGIN_MV       3000      /* OVP: fault if rail stays this far above command */
#define OVP_SAMPLES         10        /* ... for this many INA226 samples (~350 ms) */

#define FREQ_MIN_HZ         500
#define FREQ_MAX_HZ         150000
#define DUTY_MAX_X10        500       /* 50.0 % hard ceiling on on-time (core reset) */

#define ILIM_MIN_MA         50
#define ILIM_MAX_MA         2400      /* INA226 full scale is 81.92 mV / 0.033 = 2.48 A */
#define TRIP_MIN_MA         100
#define TRIP_MAX_MA         2800      /* DAC full scale gives ~2.85 A */
#define SS_MAX_MS           10000

/* Factory defaults */
#define DEF_FREQ_HZ         20000
#define DEF_DUTY_X10        100       /* 10.0 % */
#define DEF_VSET_MV         12000
#define DEF_ILIM_MA         1000
#define DEF_TRIP_MA         2000
#define DEF_SS_MS           1000

/* ------------------------------------------------------------------ */
/* Control loop                                                        */
/* ------------------------------------------------------------------ */
#define CTRL_PERIOD_MS      5
#define CC_GAIN_MV_PER_MA   5         /* rail drop per mA over the limit, per sample */
#define CC_MAX_STEP_MV      2000
#define CC_RECOVER_MV       100       /* rail recovery per sample when under limit */
#define CC_FAIL_SAMPLES     15        /* at min voltage and still >110% limit -> fault */
#define VTRIM_ENABLE        1         /* trim DAC so the INA226-measured rail matches Vset */
#define VTRIM_MAX_MV        1000      /* covers ~4 % gain error at 24 V */
#define VTRIM_WINDOW_MV     1000
#define VSLEW_MV_PER_TICK   100       /* 20 V/s max rail slew */
#define DUTY_SLEW_PER_TICK  5         /* 0.5 % per 5 ms = 100 %/s */
#define SENSOR_TIMEOUT_MS   300
#define TRIP_DISABLES_BUCK  1         /* a hardware OCP trip also shuts the LM5117 off */

/* ------------------------------------------------------------------ */
/* User interface                                                      */
/* ------------------------------------------------------------------ */
#define ENC_COUNTS_PER_DETENT 2       /* EC11E15244B2: 15 pulses / 30 detents. Use 4 if one click = two steps */
#define ENC_REVERSE         0         /* 1 if clockwise turns values down */
#define ENC_FAST_MS         40        /* detents closer than this count x10 */
#define BTN_DEBOUNCE_MS     20
#define BTN_LONG_MS         800
#define UI_FRAME_MS         100
#define UI_EDIT_TIMEOUT_MS  10000

#define UART_BAUD           115200

#endif
