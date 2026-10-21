#include "board.h"
#include "clock.h"
#include "config.h"

void gpio_cfg(GPIO_TypeDef *port, uint32_t pin, uint32_t cfg)
{
    volatile uint32_t *reg = (pin < 8u) ? &port->CRL : &port->CRH;
    uint32_t sh = (pin & 7u) * 4u;
    *reg = (*reg & ~(0xFu << sh)) | (cfg << sh);
}

/* Called from SystemInit before anything else. The LM5117 VCCDIS pin has
 * an external pull-up, so the buck is already off; this keeps it that way
 * once the pin becomes an output. */
void board_early_safe(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    GPIOB->BSRR = 1u << PB_BUCK_DIS;           /* level first, then direction */
    gpio_cfg(GPIOB, PB_BUCK_DIS, GPIO_OUT_PP_2M);
}

static void pull_down(GPIO_TypeDef *port, uint32_t pin)
{
    port->BRR = 1u << pin;
    gpio_cfg(port, pin, GPIO_IN_PULL);
}

void board_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN |
                    RCC_APB2ENR_IOPCEN | RCC_APB2ENR_AFIOEN;

    /* SWD only; frees PA15/PB3/PB4 (unused anyway). SWJ_CFG is write-only. */
    AFIO->MAPR = AFIO_MAPR_SWJ_CFG_JTAGDISABLE;

    /* Encoder and button have external 10k pull-ups */
    gpio_cfg(GPIOA, PA_ENC_A,  GPIO_IN_FLOAT);
    gpio_cfg(GPIOA, PA_ENC_B,  GPIO_IN_FLOAT);
    gpio_cfg(GPIOA, PA_ENC_SW, GPIO_IN_FLOAT);

    gpio_cfg(GPIOA, PA_VIN_SENSE, GPIO_ANALOG);
    gpio_cfg(GPIOA, PA_ICM_SENSE, GPIO_ANALOG);

    /* PWM pin stays an input (R26 pulls the driver input low) until
     * pwm_init() has the timer in a safe state. */
    gpio_cfg(GPIOA, PA_PWM, GPIO_IN_FLOAT);

    gpio_cfg(GPIOA, PA_UART_TX, GPIO_AF_PP_2M);
    GPIOA->BSRR = 1u << PA_UART_RX;
    gpio_cfg(GPIOA, PA_UART_RX, GPIO_IN_PULL);

    /* Latch reset line: released (high via R30), driven open-drain */
    GPIOA->BSRR = 1u << PA_FAULT_STOP;
    gpio_cfg(GPIOA, PA_FAULT_STOP, GPIO_OUT_OD_2M);

    /* Unused port A pins */
    pull_down(GPIOA, 3);  pull_down(GPIOA, 6);  pull_down(GPIOA, 7);
    pull_down(GPIOA, 11); pull_down(GPIOA, 15);

    /* LEDs */
    GPIOB->BRR = (1u << PB_LED_STATUS) | (1u << PB_LED_FAULT) | (1u << PB_LED_CURLIM);
    gpio_cfg(GPIOB, PB_LED_STATUS, GPIO_OUT_PP_2M);
    gpio_cfg(GPIOB, PB_LED_FAULT,  GPIO_OUT_PP_2M);
    gpio_cfg(GPIOB, PB_LED_CURLIM, GPIO_OUT_PP_2M);

    /* Comparator output: PB12 is TIM1_BKIN. PB13 is wired to the same net,
     * so it must never be driven (a low would mask a real trip). */
    gpio_cfg(GPIOB, PB_FAULT_DET,  GPIO_IN_FLOAT);
    gpio_cfg(GPIOB, PB_FAULT_DET2, GPIO_IN_FLOAT);

    /* Unused port B / C pins */
    pull_down(GPIOB, 1); pull_down(GPIOB, 2); pull_down(GPIOB, 3);
    pull_down(GPIOB, 4); pull_down(GPIOB, 5); pull_down(GPIOB, 8);
    pull_down(GPIOB, 9); pull_down(GPIOB, 15);
    pull_down(GPIOC, 13); pull_down(GPIOC, 14); pull_down(GPIOC, 15);

    /* Freeze the watchdog and TIM1 when a debugger halts the core. For
     * TIM1 this also forces the outputs off (as if MOE were cleared), so
     * hitting a breakpoint never leaves the MOSFET on. */
    DBGMCU->CR |= DBGMCU_CR_DBG_IWDG_STOP | DBGMCU_CR_DBG_TIM1_STOP;
}

void board_emergency_off(void)
{
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM1->CCR1 = 0;
    GPIOB->BSRR = 1u << PB_BUCK_DIS;
    GPIOB->BSRR = 1u << PB_LED_FAULT;
}

void led_set(led_t led, bool on)
{
    static const uint8_t pin[] = { PB_LED_STATUS, PB_LED_FAULT, PB_LED_CURLIM };
    uint32_t bit = 1u << pin[led];
    GPIOB->BSRR = on ? bit : (bit << 16);
}

void buck_enable(bool on)
{
    /* VCCDIS low = LM5117 internal VCC regulator on = buck running */
    if (on) GPIOB->BRR = 1u << PB_BUCK_DIS;
    else    GPIOB->BSRR = 1u << PB_BUCK_DIS;
}

bool buck_is_enabled(void)
{
    return (GPIOB->ODR & (1u << PB_BUCK_DIS)) == 0u;
}

bool fault_detect_active(void)
{
    return (GPIOB->IDR & (1u << PB_FAULT_DET)) != 0u;
}

/* FAULT_STOP is the R input of the NAND SR latch: a low pulse clears the
 * latch, provided the comparator is no longer reporting over-current. */
void fault_latch_reset_pulse(void)
{
    GPIOA->BRR = 1u << PA_FAULT_STOP;
    delay_us(20);
    GPIOA->BSRR = 1u << PA_FAULT_STOP;
    delay_us(5);
}

void iwdg_init(void)
{
    IWDG->KR  = 0x5555;     /* unlock PR/RLR */
    IWDG->PR  = 3;          /* LSI / 32 -> ~1.25 kHz */
    IWDG->RLR = 625;        /* ~500 ms (LSI is 30-60 kHz, so 330-670 ms) */
    IWDG->KR  = 0xAAAA;
    IWDG->KR  = 0xCCCC;     /* start */
}

void iwdg_kick(void)
{
    IWDG->KR = 0xAAAA;
}
