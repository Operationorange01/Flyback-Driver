/*
 * startup.c - vector table and reset handler for STM32F103x8/xB.
 * Any unexpected interrupt or CPU fault puts the power stage in a safe
 * state and waits for the watchdog to reset the MCU.
 */
#include <stdint.h>
#include "board.h"

extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _estack;
extern void SystemInit(void);
extern int main(void);

void Reset_Handler(void);
void Default_Handler(void);

#define WEAK_ALIAS __attribute__((weak, alias("Default_Handler")))
void NMI_Handler(void)               WEAK_ALIAS;
void HardFault_Handler(void)         WEAK_ALIAS;
void MemManage_Handler(void)         WEAK_ALIAS;
void BusFault_Handler(void)          WEAK_ALIAS;
void UsageFault_Handler(void)        WEAK_ALIAS;
void SVC_Handler(void)               WEAK_ALIAS;
void DebugMon_Handler(void)          WEAK_ALIAS;
void PendSV_Handler(void)            WEAK_ALIAS;
void SysTick_Handler(void)           WEAK_ALIAS;
void TIM1_BRK_IRQHandler(void)       WEAK_ALIAS;
void USART1_IRQHandler(void)         WEAK_ALIAS;

typedef void (*vector_t)(void);

__attribute__((section(".isr_vector"), used))
const vector_t g_vectors[16 + 43] = {
    (vector_t)(uintptr_t)&_estack,
    Reset_Handler,
    NMI_Handler,
    HardFault_Handler,
    MemManage_Handler,
    BusFault_Handler,
    UsageFault_Handler,
    0, 0, 0, 0,
    SVC_Handler,
    DebugMon_Handler,
    0,
    PendSV_Handler,
    SysTick_Handler,
    /* External interrupts 0..42 (RM0008 table 63). Only TIM1_BRK (24) and
     * USART1 (37) are used; anything else unexpected lands in
     * Default_Handler and shuts the power stage down. */
#define D Default_Handler
    D, D, D, D, D, D, D, D, D, D,          /*  0.. 9 */
    D, D, D, D, D, D, D, D, D, D,          /* 10..19 */
    D, D, D, D,                            /* 20..23 */
    TIM1_BRK_IRQHandler,                   /* 24 */
    D, D, D, D, D, D, D, D, D, D, D, D,    /* 25..36 */
    USART1_IRQHandler,                     /* 37 */
    D, D, D, D, D,                         /* 38..42 */
#undef D
};

void Reset_Handler(void)
{
    uint32_t *src = &_sidata;
    uint32_t *dst = &_sdata;
    while (dst < &_edata) *dst++ = *src++;
    for (dst = &_sbss; dst < &_ebss; ) *dst++ = 0;

    SystemInit();
    main();
    for (;;) { }
}

void Default_Handler(void)
{
    board_emergency_off();
    for (;;) { }   /* the independent watchdog resets the MCU */
}
