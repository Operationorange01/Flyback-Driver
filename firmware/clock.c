#include "clock.h"
#include "board.h"
#include "config.h"
#include "input.h"

uint32_t SystemCoreClock = 8000000UL;
static volatile uint32_t s_ms;

/* Called by the reset handler before main(). */
void SystemInit(void)
{
    board_early_safe();
}

void clock_init(void)
{
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY)) { }

    /* 64 MHz needs two flash wait states */
    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2;

    /* PLLSRC = HSI/2, x16 = 64 MHz. APB1 = 32 MHz (max 36), APB2 = 64 MHz,
     * ADC = 64/6 = 10.7 MHz (max 14). */
    RCC->CFGR = RCC_CFGR_PLLMULL16 | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_ADCPRE_DIV6;
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) { }
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) { }
    SystemCoreClock = SYSCLK_HZ;

    SysTick_Config(SYSCLK_HZ / 1000u);
    NVIC_SetPriority(SysTick_IRQn, 3);

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void SysTick_Handler(void)
{
    s_ms++;
    input_sample_1ms();
}

uint32_t millis(void)
{
    return s_ms;
}

void delay_cycles(uint32_t cycles)
{
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles) { }
}

void delay_us(uint32_t us)
{
    delay_cycles(us * (SYSCLK_HZ / 1000000UL));
}

void delay_ms(uint32_t ms)
{
    while (ms--) {
        delay_us(1000);
        iwdg_kick();
    }
}
