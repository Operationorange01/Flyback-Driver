/*
 * uart.c - USART1 on J1 (PA9 TX, PA10 RX), interrupt driven ring buffers.
 * Output never blocks the control loop: if the TX buffer is full,
 * characters are dropped.
 */
#include "uart.h"
#include "board.h"
#include "config.h"
#include "xfmt.h"
#include <stdarg.h>

#define TX_SIZE 1024u
#define RX_SIZE 128u

static volatile uint8_t  s_tx[TX_SIZE];
static volatile uint16_t s_tx_head, s_tx_tail;
static volatile uint8_t  s_rx[RX_SIZE];
static volatile uint16_t s_rx_head, s_rx_tail;

void uart_init(uint32_t baud)
{
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    USART1->BRR = (SYSCLK_HZ + baud / 2u) / baud;    /* APB2 = SYSCLK */
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
    NVIC_SetPriority(USART1_IRQn, 2);
    NVIC_EnableIRQ(USART1_IRQn);
    USART1->CR1 |= USART_CR1_UE;
}

void USART1_IRQHandler(void)
{
    uint32_t sr = USART1->SR;
    if (sr & (USART_SR_RXNE | USART_SR_ORE)) {
        uint8_t b = (uint8_t)USART1->DR;              /* also clears ORE */
        uint16_t next = (uint16_t)((s_rx_head + 1u) % RX_SIZE);
        if (next != s_rx_tail) { s_rx[s_rx_head] = b; s_rx_head = next; }
    }
    if ((USART1->CR1 & USART_CR1_TXEIE) && (sr & USART_SR_TXE)) {
        if (s_tx_tail != s_tx_head) {
            USART1->DR = s_tx[s_tx_tail];
            s_tx_tail = (uint16_t)((s_tx_tail + 1u) % TX_SIZE);
        } else {
            USART1->CR1 &= ~USART_CR1_TXEIE;
        }
    }
}

void uart_putc(char c)
{
    uint16_t next = (uint16_t)((s_tx_head + 1u) % TX_SIZE);
    if (next == s_tx_tail) return;               /* full: drop */
    s_tx[s_tx_head] = (uint8_t)c;
    s_tx_head = next;
    USART1->CR1 |= USART_CR1_TXEIE;
}

void uart_puts(const char *s)
{
    while (*s) {
        if (*s == '\n') uart_putc('\r');
        uart_putc(*s++);
    }
}

void uart_printf(const char *fmt, ...)
{
    char buf[160];
    va_list ap;
    va_start(ap, fmt);
    xvsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    uart_puts(buf);
}

int uart_getc(void)
{
    if (s_rx_tail == s_rx_head) return -1;
    uint8_t b = s_rx[s_rx_tail];
    s_rx_tail = (uint16_t)((s_rx_tail + 1u) % RX_SIZE);
    return b;
}

void uart_flush(void)
{
    while (s_tx_tail != s_tx_head) { iwdg_kick(); }
    while (!(USART1->SR & USART_SR_TC)) { }
}
