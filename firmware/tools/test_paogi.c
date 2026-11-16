/* Host test for firmware/app/paogi.c. Build and run from the repo root:
 *
 *   gcc -O2 -Wall -Wextra -I firmware/app firmware/tools/test_paogi.c \
 *       firmware/app/paogi.c -lm -o temp/test_paogi
 *   temp/test_paogi
 */
#include <math.h>
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

#define NEAR(a, b, tol) (fabs((double)(a) - (double)(b)) <= (tol))

/* The filter navigating with a fresh RTK fix, level, heading 90 deg. */
static paogi_in_t base_in(void)
{
    paogi_in_t in;
    memset(&in, 0, sizeof in);
    in.have_pos = true;
    in.pos_from_ins = true;
    in.ant_lat_deg = 59.5;
    in.ant_lon_deg = 17.25;
    in.ant_h_ell_m = 60.0;
    in.have_undulation = true;
    in.undulation_m = 30.0f;
    in.have_nav = true;
    in.tow_ms = (2u * 86400u + 12u * 3600u) * 1000u; /* Tuesday 12:00:00 GPS */
    in.leap_s = 18;
    in.sats = 20;
    in.rx_quality = 4;
    in.diff_age_s = 1.0f;
    in.gnss_fresh = true;
    in.hdop_valid = true;
    in.hdop = 0.7f;
    in.ins_vel_valid = true;
    in.vel_n_mps = 3.0f;
    in.vel_e_mps = 4.0f;
    in.rx_speed_valid = true;
    in.rx_speed_mps = 9.0f;
    in.att_valid = true;
    in.yaw_rad = M_PI / 2.0;
    in.heading_ok = true;
    in.yaw_sd_deg = 0.3f;
    in.heading_max_sd_deg = 1.0f;
    return in;
}

static void test_fill(void)
{
    paogi_fix_t f;
    paogi_in_t in = base_in();

    /* Navigating with fresh RTK: the receiver's quality, the filter's speed,
     * UTC = GPS - leap, MSL = ellipsoid - undulation. */
    paogi_fill(&in, &f);
    CHECK(f.pos_valid && f.quality == 4, "fresh: quality %d", f.quality);
    CHECK(NEAR(f.alt_msl_m, 30.0, 1e-9), "alt %f", f.alt_msl_m);
    CHECK(f.speed_valid && NEAR(f.speed_mps, 5.0, 1e-6), "ins speed %f", f.speed_mps);
    CHECK(f.time_valid && f.utc_ms == (11u * 3600u + 59u * 60u + 42u) * 1000u, "utc %u", (unsigned)f.utc_ms);
    CHECK(f.heading_valid && NEAR(f.heading_deg, 90.0, 1e-4), "heading %f", f.heading_deg);
    CHECK(f.sats == 20 && NEAR(f.hdop, 0.7, 1e-6) && NEAR(f.age_s, 1.0, 1e-6), "sats/hdop/age");

    /* The filter output 250 ms after the epoch moves the time on. */
    in.ins_dt_ms = 250;
    paogi_fill(&in, &f);
    CHECK(f.utc_ms == (11u * 3600u + 59u * 60u + 42u) * 1000u + 250u, "utc + dt %u", (unsigned)f.utc_ms);

    /* Early on Sunday the leap seconds wrap into the previous day. */
    in = base_in();
    in.tow_ms = 10000u;
    paogi_fill(&in, &f);
    CHECK(f.utc_ms == 86392000u, "wrap utc %u", (unsigned)f.utc_ms);

    /* No fresh receiver position while the filter runs on: 6, no age. */
    in = base_in();
    in.gnss_fresh = false;
    paogi_fill(&in, &f);
    CHECK(f.quality == 6 && f.age_s < 0.0f, "coasting: quality %d age %f", f.quality, f.age_s);

    /* Receiver fallback (filter not navigating): the receiver's quality,
     * even when not fresh by the 1 s rule; its speed; no heading. */
    in = base_in();
    in.pos_from_ins = false;
    in.gnss_fresh = false;
    in.rx_quality = 5;
    paogi_fill(&in, &f);
    CHECK(f.quality == 5 && !f.heading_valid, "fallback: quality %d heading %d", f.quality, f.heading_valid);
    CHECK(f.speed_valid && NEAR(f.speed_mps, 9.0, 1e-6), "fallback speed %f", f.speed_mps);
    CHECK(f.att_valid, "fallback keeps roll and pitch");

    /* No position: quality 0 and no position fields. */
    in = base_in();
    in.have_pos = false;
    in.pos_from_ins = false;
    in.rx_speed_valid = false;
    paogi_fill(&in, &f);
    CHECK(!f.pos_valid && f.quality == 0 && !f.speed_valid, "none: quality %d", f.quality);

    /* No live attitude: no roll, pitch or heading. */
    in = base_in();
    in.att_valid = false;
    paogi_fill(&in, &f);
    CHECK(!f.att_valid && !f.heading_valid, "no attitude");

    /* Yaw not from the INS in FULL / COASTING: roll and pitch, no heading. */
    in = base_in();
    in.heading_ok = false;
    paogi_fill(&in, &f);
    CHECK(f.att_valid && !f.heading_valid, "ahrs yaw sent as heading");

    /* INS yaw too uncertain (single antenna, not driven yet), or its
     * uncertainty unknown: roll and pitch, no heading. At the limit: sent. */
    in = base_in();
    in.yaw_sd_deg = 5.0f;
    paogi_fill(&in, &f);
    CHECK(f.att_valid && !f.heading_valid, "uncertain yaw sent as heading");
    in.yaw_sd_deg = -1.0f;
    paogi_fill(&in, &f);
    CHECK(!f.heading_valid, "unknown yaw sd sent as heading");
    in.yaw_sd_deg = 1.0f;
    paogi_fill(&in, &f);
    CHECK(f.heading_valid, "yaw sd at the limit not sent");

    /* Yaw rate: a right turn at 0.2 rad/s banked 30 deg right shows up as
     * q = 0.2 sin 30, r = 0.2 cos 30 in FRD, and comes back as +11.46 deg/s.
     * The bias is taken off first. */
    in = base_in();
    double roll = 30.0 / 57.29577951308232, w = 0.2;
    in.roll_rad = roll;
    in.gyr_n = 80;
    in.gyr_sum[1] = 80 * (w * sin(roll) + 0.01);
    in.gyr_sum[2] = 80 * (w * cos(roll) - 0.02);
    in.bias_valid = true;
    in.gyr_bias[1] = 0.01f;
    in.gyr_bias[2] = -0.02f;
    paogi_fill(&in, &f);
    CHECK(NEAR(f.yaw_rate_dps, w * 57.29577951308232, 1e-3), "yaw rate %f", f.yaw_rate_dps);

    /* A left turn is negative; no gyro samples give 0. */
    in.gyr_sum[1] = -80 * w * sin(roll);
    in.gyr_sum[2] = -80 * w * cos(roll);
    in.bias_valid = false;
    paogi_fill(&in, &f);
    CHECK(NEAR(f.yaw_rate_dps, -w * 57.29577951308232, 1e-3), "left yaw rate %f", f.yaw_rate_dps);
    in.gyr_n = 0;
    paogi_fill(&in, &f);
    CHECK(f.yaw_rate_dps == 0.0f, "no gyro: %f", f.yaw_rate_dps);
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

    test_fill();

    if (s_fail) {
        printf("%d check(s) failed\n", s_fail);
        return 1;
    }
    printf("test_paogi: all checks passed\n");
    return 0;
}
