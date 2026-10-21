#include "devices.h"
#include "soft_i2c.h"
#include "clock.h"
#include "config.h"
#include "conv.h"

/* ---------------------------------------------------------------- */
/* MCP4725 (datasheet DS22039)                                      */
/* ---------------------------------------------------------------- */

bool mcp4725_write(uint8_t addr, uint16_t code)
{
    /* Fast mode: C2C1 = 00, PD1PD0 = 00 (normal) */
    uint8_t b[2] = { (uint8_t)((code >> 8) & 0x0Fu), (uint8_t)(code & 0xFFu) };
    return i2c_write(addr, b, 2);
}

bool mcp4725_read(uint8_t addr, uint16_t *dac, uint16_t *eeprom)
{
    uint8_t r[5];
    if (!i2c_read(addr, r, 5)) return false;
    if (dac)    *dac    = (uint16_t)((r[1] << 4) | (r[2] >> 4));
    if (eeprom) *eeprom = (uint16_t)(((r[3] & 0x0Fu) << 8) | r[4]);
    return true;
}

bool mcp4725_write_eeprom(uint8_t addr, uint16_t code)
{
    /* C2C1C0 = 011: write DAC register and EEPROM */
    uint8_t b[3] = { 0x60u, (uint8_t)(code >> 4), (uint8_t)((code & 0x0Fu) << 4) };
    if (!i2c_write(addr, b, 3)) return false;

    for (int i = 0; i < 20; i++) {          /* EEPROM write is <= 50 ms */
        delay_ms(5);
        uint8_t st;
        if (i2c_read(addr, &st, 1) && (st & 0x80u)) return true;   /* RDY */
    }
    return false;
}

/* ---------------------------------------------------------------- */
/* INA226 (datasheet SBOS547)                                       */
/* ---------------------------------------------------------------- */
#define INA_REG_CONFIG   0x00
#define INA_REG_SHUNT    0x01
#define INA_REG_BUS      0x02
#define INA_REG_CAL      0x05
#define INA_REG_MASK     0x06
#define INA_REG_MFG_ID   0xFE
#define INA_MASK_CVRF    0x0008u

static bool ina_rd(uint8_t reg, uint16_t *v)
{
    uint8_t r[2];
    if (!i2c_write_read(I2C_ADDR_INA226, &reg, 1, r, 2)) return false;
    *v = (uint16_t)((r[0] << 8) | r[1]);
    return true;
}

static bool ina_wr(uint8_t reg, uint16_t v)
{
    uint8_t b[2] = { (uint8_t)(v >> 8), (uint8_t)v };
    return i2c_write_prefixed(I2C_ADDR_INA226, reg, b, 2);
}

bool ina226_init(void)
{
    uint16_t id;
    if (!ina_rd(INA_REG_MFG_ID, &id) || id != 0x5449u) return false;   /* "TI" */
    if (!ina_wr(INA_REG_CONFIG, 0x8000u)) return false;               /* reset */
    delay_ms(2);
    if (!ina_wr(INA_REG_CONFIG, INA226_CONFIG)) return false;
    /* CAL = 0.00512 / (100 uA * 0.033 ohm) = 1552. Firmware computes current
     * from the shunt register itself; this just makes the chip's own
     * current/power registers meaningful for anyone reading them. */
    if (!ina_wr(INA_REG_CAL, 1552u)) return false;
    uint16_t chk;
    return ina_rd(INA_REG_CONFIG, &chk) && chk == INA226_CONFIG;
}

bool ina226_poll(ina226_sample_t *s, bool *fresh)
{
    uint16_t mask, sh, bus;
    *fresh = false;
    if (!ina_rd(INA_REG_MASK, &mask)) return false;      /* reading clears CVRF */
    if (!(mask & INA_MASK_CVRF)) return true;
    if (!ina_rd(INA_REG_SHUNT, &sh)) return false;
    if (!ina_rd(INA_REG_BUS, &bus)) return false;
    s->i_ma    = conv_ina_shunt_to_ma((int16_t)sh);
    s->vbus_mv = conv_ina_bus_to_mv(bus);
    s->p_mw    = (int32_t)(((int64_t)s->vbus_mv * s->i_ma) / 1000);
    *fresh = true;
    return true;
}
