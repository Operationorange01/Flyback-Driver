/*
 * settings.c - settings stored in the last 1 KB flash page (0x0800FC00).
 * Output enable is deliberately NOT stored: the board always boots OFF.
 */
#include "settings.h"
#include "board.h"
#include "config.h"
#include "xfmt.h"
#include <string.h>

#define SETTINGS_MAGIC   0x46425356u   /* "FBSV" */
#define SETTINGS_VERSION 1u

extern const uint8_t _settings_start[1024];      /* linker: last flash page */
#define SETTINGS_ADDR ((uint32_t)(uintptr_t)_settings_start)

typedef struct {
    uint32_t   magic;
    uint32_t   version;
    settings_t s;
    uint32_t   crc;
} stored_t;

settings_t g_set;

void settings_defaults(void)
{
    g_set.freq_hz  = DEF_FREQ_HZ;
    g_set.duty_x10 = DEF_DUTY_X10;
    g_set.vset_mv  = DEF_VSET_MV;
    g_set.ilim_ma  = DEF_ILIM_MA;
    g_set.trip_ma  = DEF_TRIP_MA;
    g_set.ss_ms    = DEF_SS_MS;
    g_set.reserved = 0;
}

#define CLAMP(v, lo, hi) do { if ((v) < (lo)) (v) = (lo); if ((v) > (hi)) (v) = (hi); } while (0)

void settings_clamp(void)
{
    CLAMP(g_set.freq_hz,  FREQ_MIN_HZ, FREQ_MAX_HZ);
    if (g_set.duty_x10 > DUTY_MAX_X10) g_set.duty_x10 = DUTY_MAX_X10;
    CLAMP(g_set.vset_mv,  VSET_MIN_MV, VSET_MAX_MV);
    CLAMP(g_set.ilim_ma,  ILIM_MIN_MA, ILIM_MAX_MA);
    CLAMP(g_set.trip_ma,  TRIP_MIN_MA, TRIP_MAX_MA);
    if (g_set.ss_ms > SS_MAX_MS) g_set.ss_ms = SS_MAX_MS;
}

bool settings_load(void)
{
    const stored_t *st = (const stored_t *)(const void *)_settings_start;
    if (st->magic == SETTINGS_MAGIC && st->version == SETTINGS_VERSION &&
        st->crc == crc32_calc(&st->s, sizeof st->s)) {
        g_set = st->s;
        settings_clamp();
        return true;
    }
    settings_defaults();
    return false;
}

static bool flash_wait(void)
{
    while (FLASH->SR & FLASH_SR_BSY) { }
    bool ok = !(FLASH->SR & (FLASH_SR_PGERR | FLASH_SR_WRPRTERR));
    FLASH->SR = FLASH_SR_EOP | FLASH_SR_PGERR | FLASH_SR_WRPRTERR;
    return ok;
}

/* Erasing stalls instruction fetch for ~20 ms. The caller makes sure the
 * output is off; the watchdog has ample margin. */
bool settings_save(void)
{
    stored_t st;
    memset(&st, 0, sizeof st);
    st.magic   = SETTINGS_MAGIC;
    st.version = SETTINGS_VERSION;
    st.s       = g_set;
    st.crc     = crc32_calc(&st.s, sizeof st.s);

    uint16_t hw[sizeof(stored_t) / 2];
    memcpy(hw, &st, sizeof hw);

    iwdg_kick();
    if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR = 0x45670123u;
        FLASH->KEYR = 0xCDEF89ABu;
    }
    bool ok = flash_wait();

    FLASH->CR |= FLASH_CR_PER;
    FLASH->AR  = SETTINGS_ADDR;
    FLASH->CR |= FLASH_CR_STRT;
    ok = flash_wait() && ok;
    FLASH->CR &= ~FLASH_CR_PER;

    FLASH->CR |= FLASH_CR_PG;
    for (uint32_t i = 0; ok && i < sizeof(hw) / 2u; i++) {
        *(volatile uint16_t *)(SETTINGS_ADDR + i * 2u) = hw[i];
        ok = flash_wait();
    }
    FLASH->CR &= ~FLASH_CR_PG;
    FLASH->CR |= FLASH_CR_LOCK;
    iwdg_kick();

    return ok && memcmp(_settings_start, &st, sizeof st) == 0;
}
