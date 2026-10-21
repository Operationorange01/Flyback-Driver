#include "xfmt.h"
#include <string.h>

static int utoa_base(char *t, uint32_t v, uint32_t base)
{
    char r[12];
    int n = 0;
    do {
        uint32_t d = v % base;
        r[n++] = (char)(d < 10u ? '0' + d : 'a' + d - 10u);
        v /= base;
    } while (v);
    for (int i = 0; i < n; i++) t[i] = r[n - 1 - i];
    return n;
}

static uint32_t abs_u32(int32_t v)
{
    return v < 0 ? (uint32_t)(-(v + 1)) + 1u : (uint32_t)v;
}

int xvsnprintf(char *buf, size_t size, const char *fmt, va_list ap)
{
    size_t pos = 0;
#define OUT(ch) do { if (pos + 1u < size) buf[pos] = (char)(ch); pos++; } while (0)

    while (*fmt) {
        char c = *fmt++;
        if (c != '%') { OUT(c); continue; }
        if (*fmt == '%') { OUT('%'); fmt++; continue; }

        bool left = false, zero = false;
        int width = 0, prec = -1;
        for (;;) {
            if (*fmt == '-') { left = true; fmt++; }
            else if (*fmt == '0') { zero = true; fmt++; }
            else break;
        }
        while (*fmt >= '0' && *fmt <= '9') width = width * 10 + (*fmt++ - '0');
        if (*fmt == '.') {
            fmt++;
            prec = 0;
            while (*fmt >= '0' && *fmt <= '9') prec = prec * 10 + (*fmt++ - '0');
        }
        while (*fmt == 'l') fmt++;
        char conv = *fmt ? *fmt++ : '\0';

        char tmp[24];
        const char *s = tmp;
        int len = 0;
        bool neg = false;

        switch (conv) {
        case 'd': {
            int32_t v = va_arg(ap, int);
            neg = v < 0;
            len = utoa_base(tmp, abs_u32(v), 10);
            break;
        }
        case 'u': len = utoa_base(tmp, va_arg(ap, unsigned), 10); break;
        case 'x': len = utoa_base(tmp, va_arg(ap, unsigned), 16); break;
        case 'c': tmp[0] = (char)va_arg(ap, int); len = 1; break;
        case 's':
            s = va_arg(ap, const char *);
            if (!s) s = "(null)";
            len = (int)strlen(s);
            if (prec >= 0 && len > prec) len = prec;
            break;
        case 'k': {
            int32_t v = va_arg(ap, int);
            int p = prec < 0 ? 3 : (prec > 3 ? 3 : prec);
            uint32_t scale = 1, pw = 1;
            for (int i = p; i < 3; i++) scale *= 10u;
            for (int i = 0; i < p; i++) pw *= 10u;
            uint32_t u = (abs_u32(v) + scale / 2u) / scale;
            neg = (v < 0) && (u != 0u);
            len = utoa_base(tmp, u / pw, 10);
            if (p > 0) {
                uint32_t f = u % pw;
                tmp[len] = '.';
                for (int i = p; i >= 1; i--) { tmp[len + i] = (char)('0' + f % 10u); f /= 10u; }
                len += p + 1;
            }
            break;
        }
        default:
            OUT('%');
            if (conv) OUT(conv);
            continue;
        }

        int total = len + (neg ? 1 : 0);
        int pad = width > total ? width - total : 0;
        if (!left && !zero) for (; pad > 0; pad--) OUT(' ');
        if (neg) OUT('-');
        if (!left && zero) for (; pad > 0; pad--) OUT('0');
        for (int i = 0; i < len; i++) OUT(s[i]);
        if (left) for (; pad > 0; pad--) OUT(' ');
    }
    if (size) buf[pos < size ? pos : size - 1u] = '\0';
    return (int)pos;
#undef OUT
}

int xsnprintf(char *buf, size_t size, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    int n = xvsnprintf(buf, size, fmt, ap);
    va_end(ap);
    return n;
}

bool parse_fixed(const char *s, int decimals, int32_t *out)
{
    bool neg = false, any = false;
    int64_t v = 0;
    int frac = -1;

    while (*s == ' ') s++;
    if (*s == '-') { neg = true; s++; }
    else if (*s == '+') s++;

    for (; *s; s++) {
        if (*s >= '0' && *s <= '9') {
            any = true;
            if (frac < 0) {
                v = v * 10 + (*s - '0');
            } else if (frac < decimals) {
                v = v * 10 + (*s - '0');
                frac++;
            }
            if (v > 2000000000LL) return false;
        } else if (*s == '.' && frac < 0) {
            frac = 0;
        } else {
            break;
        }
    }
    if (!any) return false;
    while (*s && ((*s >= 'a' && *s <= 'z') || (*s >= 'A' && *s <= 'Z') || *s == '%')) s++;
    if (*s && *s != ' ') return false;

    if (frac < 0) frac = 0;
    for (; frac < decimals; frac++) v *= 10;
    if (v > 2000000000LL) return false;
    *out = (int32_t)(neg ? -v : v);
    return true;
}

uint32_t crc32_calc(const void *data, size_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFFu;
    while (len--) {
        crc ^= *p++;
        for (int i = 0; i < 8; i++) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return ~crc;
}
