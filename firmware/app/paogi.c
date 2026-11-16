/* $PAOGI formatter (paogi.h). */
#include "paogi.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>

static bool s_enabled = true;

void paogi_set_enabled(bool on) { s_enabled = on; }
bool paogi_enabled(void) { return s_enabled; }

typedef struct {
    char  *b;
    size_t cap, n;
    bool   ok;
} out_t;

static void put(out_t *o, const char *fmt, ...)
{
    if (!o->ok) return;
    va_list ap;
    va_start(ap, fmt);
    int r = vsnprintf(o->b + o->n, o->cap - o->n, fmt, ap);
    va_end(ap);
    if (r < 0 || (size_t)r >= o->cap - o->n) {
        o->ok = false;
        return;
    }
    o->n += (size_t)r;
}

static const int64_t k_pow10[] = { 1, 10, 100, 1000, 10000, 100000, 1000000, 10000000 };

/* ",value" with dec decimals, rounded half away from zero. */
static void put_fixed(out_t *o, double v, int dec)
{
    int64_t u = llround(v * (double)k_pow10[dec]);
    const char *sign = u < 0 ? "-" : "";
    if (u < 0) u = -u;
    if (dec == 0) {
        put(o, ",%s%lld", sign, (long long)u);
    } else {
        put(o, ",%s%lld.%0*lld", sign, (long long)(u / k_pow10[dec]), dec, (long long)(u % k_pow10[dec]));
    }
}

/* ",DDMM.MMMMMMM,H" with deg_w digits of degrees. */
static void put_coord(out_t *o, double deg, int deg_w, char pos, char neg)
{
    const int64_t per_min = k_pow10[7];
    int64_t u = llround(fabs(deg) * 60.0 * (double)per_min); /* rounded first */
    int64_t d = u / (60 * per_min);
    int64_t rem = u % (60 * per_min);
    put(o, ",%0*lld%02lld.%07lld,%c", deg_w, (long long)d, (long long)(rem / per_min),
        (long long)(rem % per_min), deg < 0.0 ? neg : pos);
}

size_t paogi_format(const paogi_fix_t *f, char *buf, size_t cap)
{
    out_t o = { buf, cap, 0, cap > 0 };
    put(&o, "$PAOGI");

    /* 1 time hhmmss.ss */
    if (f->time_valid) {
        uint32_t cs = ((f->utc_ms + 5u) / 10u) % 8640000u;
        put(&o, ",%02lu%02lu%02lu.%02lu", (unsigned long)(cs / 360000u), (unsigned long)(cs / 6000u % 60u),
            (unsigned long)(cs / 100u % 60u), (unsigned long)(cs % 100u));
    } else {
        put(&o, ",");
    }

    /* 2-5 lat, N/S, lon, E/W */
    if (f->pos_valid) {
        put_coord(&o, f->lat_deg, 2, 'N', 'S');
        put_coord(&o, f->lon_deg, 3, 'E', 'W');
    } else {
        put(&o, ",,,,");
    }

    /* 6 fix, 7 sats, 8 hdop */
    put(&o, ",%d", f->quality);
    if (f->sats >= 0) put(&o, ",%d", f->sats); else put(&o, ",");
    if (f->hdop >= 0.0f) put_fixed(&o, f->hdop, 1); else put(&o, ",");

    /* 9 alt, 10 age */
    if (f->pos_valid) put_fixed(&o, f->alt_msl_m, 3); else put(&o, ",");
    if (f->age_s >= 0.0f) put_fixed(&o, f->age_s, 1); else put(&o, ",");

    /* 11 speed, knots */
    if (f->speed_valid) put_fixed(&o, f->speed_mps / 0.514444, 3); else put(&o, ",");

    /* 12 heading 0..359.99, rounded before the wrap */
    if (f->heading_valid) {
        int64_t h = llround((double)f->heading_deg * 100.0) % 36000;
        if (h < 0) h += 36000;
        put(&o, ",%lld.%02lld", (long long)(h / 100), (long long)(h % 100));
    } else {
        put(&o, ",");
    }

    /* 13 roll, 14 pitch */
    if (f->att_valid) {
        put_fixed(&o, f->roll_deg, 2);
        put_fixed(&o, f->pitch_deg, 2);
    } else {
        put(&o, ",,");
    }

    /* 15 yaw rate */
    put_fixed(&o, f->yaw_rate_dps, 2);

    if (!o.ok) {
        if (cap > 0) buf[0] = '\0';
        return 0;
    }
    uint8_t ck = 0;
    for (size_t i = 1; i < o.n; i++) ck ^= (uint8_t)buf[i];
    put(&o, "*%02X\r\n", ck);
    if (!o.ok) {
        buf[0] = '\0';
        return 0;
    }
    return o.n;
}
