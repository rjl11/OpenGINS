/* Host test for firmware/app/paogi.c. Build and run from the repo root:
 *
 *   gcc -O2 -Wall -Wextra -I firmware/app firmware/tools/test_paogi.c \
 *       firmware/app/paogi.c -lm -o temp/test_paogi
 *   temp/test_paogi
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "paogi.h"

static int s_fail;

#define CHECK(c, ...) do { if (!(c)) { printf("FAIL %s:%d: ", __FILE__, __LINE__); \
                                       printf(__VA_ARGS__); printf("\n"); s_fail++; } } while (0)

/* The sentence without "*hh\r\n" must be body; the checksum and the field
 * count are checked on their own. */
static void expect(const char *name, const paogi_fix_t *f, const char *body)
{
    char buf[PAOGI_MAX];
    size_t n = paogi_format(f, buf, sizeof buf);
    CHECK(n == strlen(buf), "%s: length %zu, strlen %zu", name, n, strlen(buf));
    CHECK(n >= 6 && strcmp(buf + n - 2, "\r\n") == 0, "%s: no CRLF: %s", name, buf);
    const char *star = strchr(buf, '*');
    CHECK(star != NULL, "%s: no checksum: %s", name, buf);
    if (!star) return;
    size_t blen = (size_t)(star - buf);
    CHECK(blen == strlen(body) && memcmp(buf, body, blen) == 0, "%s:\n  got  %.*s\n  want %s", name, (int)blen,
          buf, body);
    unsigned ck = 0;
    for (size_t i = 1; i < blen; i++) ck ^= (unsigned char)buf[i];
    unsigned got = (unsigned)strtoul(star + 1, NULL, 16);
    CHECK(got == ck && star[3] == '\r', "%s: checksum %02X, want %02X", name, got, ck);
    int commas = 0;
    for (size_t i = 0; i < blen; i++) commas += buf[i] == ',';
    CHECK(commas == 15, "%s: %d fields, want 15", name, commas);
}

int main(void)
{
    paogi_fix_t f = {
        .time_valid = true, .utc_ms = ((12u * 3600u + 34u * 60u + 56u) * 1000u) + 780u,
        .pos_valid = true, .lat_deg = 59.5, .lon_deg = 17.25, .alt_msl_m = 25.1234,
        .quality = 4, .sats = 18, .hdop = 0.8f, .age_s = 1.0f,
        .speed_valid = true, .speed_mps = 2.57222f,
        .heading_valid = true, .heading_deg = 90.004f,
        .att_valid = true, .roll_deg = -1.234f, .pitch_deg = 0.5f, .yaw_rate_dps = 0.0f,
    };
    expect("full", &f,
           "$PAOGI,123456.78,5930.0000000,N,01715.0000000,E,4,18,0.8,25.123,1.0,5.000,90.00,-1.23,0.50,0.00");

    /* South / west, and the minutes carrying into the degrees. */
    paogi_fix_t g = f;
    g.lat_deg = -33.8688;
    g.lon_deg = -(70.0 + 59.99999996 / 60.0);
    g.heading_deg = 359.996f;
    g.yaw_rate_dps = -3.456f;
    expect("south-west", &g,
           "$PAOGI,123456.78,3352.1280000,S,07100.0000000,W,4,18,0.8,25.123,1.0,5.000,0.00,-1.23,0.50,-3.46");

    /* Time rounding into the next day wraps to 00:00:00. */
    paogi_fix_t t = f;
    t.utc_ms = 86399996u;
    expect("midnight", &t,
           "$PAOGI,000000.00,5930.0000000,N,01715.0000000,E,4,18,0.8,25.123,1.0,5.000,90.00,-1.23,0.50,0.00");

    /* Nothing known: every field empty but fix and yaw rate. */
    paogi_fix_t e = { .quality = 0, .sats = -1, .hdop = -1.0f, .age_s = -1.0f };
    expect("empty", &e, "$PAOGI,,,,,,0,,,,,,,,,0.00");

    /* Heading empty, roll present (aligning). */
    paogi_fix_t h = f;
    h.heading_valid = false;
    h.age_s = -1.0f;
    expect("no heading", &h,
           "$PAOGI,123456.78,5930.0000000,N,01715.0000000,E,4,18,0.8,25.123,,5.000,,-1.23,0.50,0.00");

    /* Too small a buffer. */
    char small[20];
    CHECK(paogi_format(&f, small, sizeof small) == 0 && small[0] == '\0', "small buffer accepted");

    if (s_fail) {
        printf("%d check(s) failed\n", s_fail);
        return 1;
    }
    printf("test_paogi: all checks passed\n");
    return 0;
}
