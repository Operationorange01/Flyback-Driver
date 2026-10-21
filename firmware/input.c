/*
 * input.c - rotary encoder on TIM2 (hardware quadrature decoding with the
 * maximum input filter) and the encoder push button on PA2.
 */
#include "input.h"
#include "board.h"
#include "clock.h"
#include "config.h"

static uint16_t s_last_cnt;
static int32_t  s_acc;
static uint32_t s_last_step_ms;

static volatile uint8_t  s_event;
static uint8_t  s_stable, s_debounce;
static uint16_t s_held_ms;
static bool     s_long_fired;

void input_init(void)
{
    RCC->APB1ENR  |= RCC_APB1ENR_TIM2EN;
    TIM2->CR1   = (2u << TIM_CR1_CKD_Pos);        /* tDTS = 4 x tCK */
    TIM2->SMCR  = (3u << TIM_SMCR_SMS_Pos);       /* encoder mode 3, x4 */
    TIM2->CCMR1 = (1u << TIM_CCMR1_CC1S_Pos) | (1u << TIM_CCMR1_CC2S_Pos) |
                  (0xFu << TIM_CCMR1_IC1F_Pos) | (0xFu << TIM_CCMR1_IC2F_Pos);
    TIM2->CCER  = 0;
    TIM2->ARR   = 0xFFFF;
    TIM2->CNT   = 0;
    TIM2->CR1  |= TIM_CR1_CEN;
    s_last_cnt  = 0;
}

int input_enc_steps(bool *fast)
{
    uint16_t cnt = (uint16_t)TIM2->CNT;
    int16_t delta = (int16_t)(cnt - s_last_cnt);
    s_last_cnt = cnt;
#if ENC_REVERSE
    delta = (int16_t)-delta;
#endif
    s_acc += delta;
    int steps = (int)(s_acc / ENC_COUNTS_PER_DETENT);
    s_acc -= steps * ENC_COUNTS_PER_DETENT;

    *fast = false;
    if (steps) {
        uint32_t now = millis();
        *fast = (now - s_last_step_ms) < ENC_FAST_MS;
        s_last_step_ms = now;
    }
    return steps;
}

void input_sample_1ms(void)
{
    uint8_t raw = (GPIOA->IDR & (1u << PA_ENC_SW)) ? 0u : 1u;   /* pressed = low */

    if (raw != s_stable) {
        if (++s_debounce >= BTN_DEBOUNCE_MS) {
            s_stable = raw;
            s_debounce = 0;
            if (s_stable) {
                s_held_ms = 0;
                s_long_fired = false;
            } else if (!s_long_fired) {
                s_event = BTN_SHORT;
            }
        }
    } else {
        s_debounce = 0;
    }

    if (s_stable && !s_long_fired) {
        if (++s_held_ms >= BTN_LONG_MS) {
            s_long_fired = true;
            s_event = BTN_LONG;
        }
    }
}

btn_event_t input_take_button(void)
{
    __disable_irq();
    btn_event_t e = (btn_event_t)s_event;
    s_event = BTN_NONE;
    __enable_irq();
    return e;
}
