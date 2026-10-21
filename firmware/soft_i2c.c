#include "soft_i2c.h"
#include "board.h"
#include "clock.h"
#include "config.h"

#define SCL_BIT   (1u << PB_I2C_SCL)
#define SDA_BIT   (1u << PB_I2C_SDA)
#define HALF_CYC  ((SYSCLK_HZ / I2C_SPEED_HZ) / 2u)
#define STRETCH_TIMEOUT_CYC (SYSCLK_HZ / 1000u)   /* 1 ms */

static uint32_t s_errors;

static void dly(void)     { delay_cycles(HALF_CYC); }
static void sda_hi(void)  { GPIOB->BSRR = SDA_BIT; }
static void sda_lo(void)  { GPIOB->BRR  = SDA_BIT; }
static void scl_lo(void)  { GPIOB->BRR  = SCL_BIT; }
static bool sda_in(void)  { return (GPIOB->IDR & SDA_BIT) != 0u; }

static bool scl_hi(void)
{
    GPIOB->BSRR = SCL_BIT;
    uint32_t t0 = DWT->CYCCNT;
    while (!(GPIOB->IDR & SCL_BIT)) {               /* clock stretching */
        if ((DWT->CYCCNT - t0) > STRETCH_TIMEOUT_CYC) return false;
    }
    return true;
}

static bool start(void)
{
    sda_hi(); dly();
    if (!scl_hi()) return false;
    dly();
    if (!sda_in()) return false;     /* someone is holding SDA */
    sda_lo(); dly();
    scl_lo(); dly();
    return true;
}

static void stop(void)
{
    sda_lo(); dly();
    scl_hi(); dly();
    sda_hi(); dly();
}

static bool write_byte(uint8_t b)
{
    for (int i = 0; i < 8; i++) {
        if (b & 0x80u) sda_hi(); else sda_lo();
        b <<= 1;
        dly();
        if (!scl_hi()) return false;
        dly();
        scl_lo();
    }
    sda_hi(); dly();
    if (!scl_hi()) return false;
    dly();
    bool ack = !sda_in();
    scl_lo();
    return ack;
}

static bool read_byte(uint8_t *out, bool ack)
{
    uint8_t b = 0;
    sda_hi();
    for (int i = 0; i < 8; i++) {
        dly();
        if (!scl_hi()) return false;
        dly();
        b = (uint8_t)((b << 1) | (sda_in() ? 1u : 0u));
        scl_lo();
    }
    if (ack) sda_lo(); else sda_hi();
    dly();
    if (!scl_hi()) return false;
    dly();
    scl_lo();
    sda_hi();
    *out = b;
    return true;
}

void i2c_bus_recover(void)
{
    sda_hi();
    for (int i = 0; i < 9 && !sda_in(); i++) {
        scl_lo(); dly();
        (void)scl_hi(); dly();
    }
    stop();
}

void i2c_init(void)
{
    GPIOB->BSRR = SCL_BIT | SDA_BIT;
    gpio_cfg(GPIOB, PB_I2C_SCL, GPIO_OUT_OD_10M);
    gpio_cfg(GPIOB, PB_I2C_SDA, GPIO_OUT_OD_10M);
    delay_us(10);
    i2c_bus_recover();
}

static bool xfer(uint8_t addr, const uint8_t *prefix, const uint8_t *w, uint32_t wlen,
                 uint8_t *r, uint32_t rlen)
{
    bool ok = start();
    bool need_write = (prefix != 0) || (wlen > 0u) || (rlen == 0u);

    if (ok && need_write) {
        ok = write_byte((uint8_t)(addr << 1));
        if (ok && prefix) ok = write_byte(*prefix);
        for (uint32_t i = 0; ok && i < wlen; i++) ok = write_byte(w[i]);
        if (ok && rlen) ok = start();                /* repeated start */
    }
    if (ok && rlen) {
        ok = write_byte((uint8_t)((addr << 1) | 1u));
        for (uint32_t i = 0; ok && i < rlen; i++) ok = read_byte(&r[i], (i + 1u) < rlen);
    }
    stop();
    if (!ok) {
        s_errors++;
        i2c_bus_recover();
    }
    return ok;
}

bool i2c_probe(uint8_t addr)
{
    return xfer(addr, 0, 0, 0, 0, 0);
}

bool i2c_write(uint8_t addr, const uint8_t *data, uint32_t len)
{
    return xfer(addr, 0, data, len, 0, 0);
}

bool i2c_write_prefixed(uint8_t addr, uint8_t prefix, const uint8_t *data, uint32_t len)
{
    return xfer(addr, &prefix, data, len, 0, 0);
}

bool i2c_read(uint8_t addr, uint8_t *data, uint32_t len)
{
    return xfer(addr, 0, 0, 0, data, len);
}

bool i2c_write_read(uint8_t addr, const uint8_t *w, uint32_t wlen, uint8_t *r, uint32_t rlen)
{
    return xfer(addr, 0, w, wlen, r, rlen);
}

uint32_t i2c_error_count(void)
{
    return s_errors;
}
