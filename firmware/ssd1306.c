#include "ssd1306.h"
#include "gfx.h"
#include "soft_i2c.h"
#include "config.h"

static bool cmds(const uint8_t *c, uint32_t n)
{
    return i2c_write_prefixed(I2C_ADDR_OLED, 0x00, c, n);   /* Co=0, D/C=0 */
}

bool ssd1306_init(void)
{
    static const uint8_t init[] = {
        0xAE,             /* display off */
        0xD5, 0x80,       /* clock divide */
        0xA8, 0x1F,       /* multiplex 32 */
        0xD3, 0x00,       /* display offset */
        0x40,             /* start line 0 */
        0x8D, 0x14,       /* charge pump on (module runs from 3V3) */
        0x20, 0x02,       /* page addressing */
        0xA1,             /* segment remap */
        0xC8,             /* COM scan descending */
        0xDA, 0x02,       /* COM pins for 128x32 */
        0x81, 0x8F,       /* contrast */
        0xD9, 0xF1,       /* pre-charge */
        0xDB, 0x40,       /* VCOMH */
        0xA4,             /* display RAM */
        0xA6,             /* normal (not inverted) */
        0x2E,             /* no scroll */
        0xAF,             /* display on */
    };
    if (!i2c_probe(I2C_ADDR_OLED)) return false;
    if (!cmds(init, sizeof init)) return false;
    gfx_clear();
    for (uint8_t p = 0; p < GFX_PAGES; p++) {
        if (!ssd1306_flush_page(p)) return false;
    }
    return true;
}

bool ssd1306_flush_page(uint8_t page)
{
    uint8_t c[3] = { (uint8_t)(0xB0u | page), 0x00, 0x10 };
    if (!cmds(c, 3)) return false;
    return i2c_write_prefixed(I2C_ADDR_OLED, 0x40, &g_fb[page * GFX_W], GFX_W);
}
