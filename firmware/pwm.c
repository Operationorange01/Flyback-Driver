/*
 * pwm.c - TIM1 channel 1 on PA8 drives the UCC27511 / IRFP460.
 *
 * Three independent ways the gate is turned off on over-current:
 *  1. LM393 + 74HC latch pulls UCC27511 IN- high (pure hardware, ~100 ns)
 *  2. the same comparator output is TIM1_BKIN (PB12): the timer clears
 *     MOE asynchronously, no CPU involved
 *  3. the break interrupt tells the firmware, which latches a fault
 * MOE is never set again automatically (AOE = 0).
 */
#include "pwm.h"
#include "board.h"
#include "config.h"
#include "conv.h"

static uint32_t s_freq = DEF_FREQ_HZ;
static uint32_t s_duty = 0;

void pwm_init(void)
{
    RCC->APB2ENR  |= RCC_APB2ENR_TIM1EN;
    RCC->APB2RSTR |= RCC_APB2RSTR_TIM1RST;
    RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM1RST;

    TIM1->CR1   = TIM_CR1_ARPE;                         /* edge aligned, up-count */
    TIM1->CR2   = 0;                                    /* OIS1 = 0: idle level low */
    TIM1->CCMR1 = (6u << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;  /* PWM mode 1 */
    TIM1->CCR1  = 0;
    TIM1->CCER  = TIM_CCER_CC1E;                        /* active high */

    /* MOE = 0 for now. OSSI/OSSR = 1: output actively driven low when off.
     * BKE + BKP: break on PB12 HIGH. LOCK level 1 freezes BKE/BKP/OIS/AOE. */
    TIM1->BDTR  = TIM_BDTR_OSSR | TIM_BDTR_OSSI | TIM_BDTR_BKE | TIM_BDTR_BKP |
                  (1u << TIM_BDTR_LOCK_Pos);

    pwm_set(DEF_FREQ_HZ, 0);
    TIM1->EGR = TIM_EGR_UG;            /* load the preloaded values */
    TIM1->SR  = 0;

    NVIC_SetPriority(TIM1_BRK_IRQn, 0);
    NVIC_EnableIRQ(TIM1_BRK_IRQn);
    TIM1->DIER = TIM_DIER_BIE;

    TIM1->CR1 |= TIM_CR1_CEN;

    /* Only now hand PA8 to the timer; it is driving low already. */
    gpio_cfg(GPIOA, PA_PWM, GPIO_AF_PP_50M);
}

void pwm_set(uint32_t freq_hz, uint32_t duty_x10)
{
    pwm_timing_t t = conv_pwm_timing(SYSCLK_HZ, freq_hz, duty_x10, DUTY_MAX_X10);
    s_freq = freq_hz;
    s_duty = duty_x10;

    /* PSC, ARR and CCR1 are all preloaded. Block update events while
     * writing so the three values take effect together on the next
     * overflow - no runt or stretched pulse. */
    TIM1->CR1 |= TIM_CR1_UDIS;
    TIM1->PSC  = t.psc;
    TIM1->ARR  = t.arr;
    TIM1->CCR1 = t.ccr;
    TIM1->CR1 &= ~TIM_CR1_UDIS;
}

uint32_t pwm_actual_freq(void)
{
    uint32_t div = (TIM1->PSC + 1u) * (TIM1->ARR + 1u);
    return div ? SYSCLK_HZ / div : 0;
}

bool pwm_arm(void)
{
    TIM1->SR = ~TIM_SR_BIF;              /* rc_w0: clears only BIF */
    if (TIM1->SR & TIM_SR_BIF) return false;   /* break input still high */
    TIM1->DIER |= TIM_DIER_BIE;
    TIM1->BDTR |= TIM_BDTR_MOE;
    return true;
}

void pwm_disarm(void)
{
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    pwm_set(s_freq, 0);
}

bool pwm_is_armed(void)
{
    return (TIM1->BDTR & TIM_BDTR_MOE) != 0u;
}

void TIM1_BRK_IRQHandler(void)
{
    /* The break input can stay high for a while; disable the interrupt so
     * it can't storm. pwm_arm() re-enables it. MOE is already cleared by
     * hardware at this point. */
    TIM1->DIER &= ~TIM_DIER_BIE;
    TIM1->SR = ~TIM_SR_BIF;
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    control_hw_trip_isr();
}
