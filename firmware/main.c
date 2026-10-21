/*
 * Flyback Driver firmware - STM32F103C8T6
 *
 * Bare-metal (CMSIS register headers only). See README.md for the
 * hardware notes, the UI and the serial command set.
 */
#include "board.h"
#include "clock.h"
#include "config.h"
#include "uart.h"
#include "adc.h"
#include "pwm.h"
#include "input.h"
#include "soft_i2c.h"
#include "ssd1306.h"
#include "settings.h"
#include "control.h"
#include "ui.h"
#include "cli.h"

static const char *reset_cause(void)
{
    uint32_t csr = RCC->CSR;
    RCC->CSR |= RCC_CSR_RMVF;
    if (csr & RCC_CSR_IWDGRSTF) return "WATCHDOG";
    if (csr & RCC_CSR_SFTRSTF)  return "software";
    if (csr & RCC_CSR_PORRSTF)  return "power-on";
    if (csr & RCC_CSR_PINRSTF)  return "reset pin";
    return "other";
}

int main(void)
{
    /* BUCK_DIS was already driven high in SystemInit(). */
    clock_init();
    board_init();
    uart_init(UART_BAUD);
    uart_printf("\n\nFlyback Driver fw %s  (reset: %s)\n", FW_VERSION, reset_cause());

    adc_init();
    pwm_init();          /* timer running, MOE = 0, gate held low */
    input_init();
    i2c_init();

    bool loaded = settings_load();
    uart_puts(loaded ? "settings loaded from flash\n" : "using default settings\n");

    bool oled = ssd1306_init();
    uart_printf("OLED %s\n", oled ? "ok" : "not found (optional)");

    control_init();      /* checks devices, parks DACs, clears the OCP latch */
    iwdg_init();
    ui_init(oled);
    cli_init();
    uart_puts("type 'help' for commands\n> ");

    uint32_t t_ctrl = millis();
    for (;;) {
        iwdg_kick();
        cli_poll();

        uint32_t now = millis();
        if ((int32_t)(now - t_ctrl) >= CTRL_PERIOD_MS) {
            t_ctrl += CTRL_PERIOD_MS;
            if ((int32_t)(now - t_ctrl) > 50) t_ctrl = now;   /* don't try to catch up */
            control_tick();
        }

        ui_tick(now);
        cli_tick(now);
    }
}
