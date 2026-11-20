/*
 * $PAOGI, the AgOpenGPS / AgOpenWeb sentence (plans/ethernet-w5500-agopenweb.md,
 * "Output format: $PAOGI at 10 Hz"). On the tractor platform main.c sends it
 * on the USB port at 10 Hz, between the status report's text lines; readers
 * skip the lines that don't start with $. The formatter here is pure C with
 * no SDK calls, so firmware/tools/test_paogi.c runs it on the host.
 *
 *   $PAOGI,time,lat,N/S,lon,E/W,fix,sats,hdop,alt,age,speed,heading,roll,pitch,yaw_rate*hh
 *
 * All 15 fields are always there; an unknown value is an empty field, never
 * 0 (a 0 heading is north, and AutoSteer would steer on it). Numbers are
 * rounded, not cut, from scaled integers: time hhmmss.ss, minutes of
 * latitude and longitude with 7 decimals (about 2 mm), rounded before
 * degrees and minutes are split so 59.99999999' carries into the degrees,
 * altitude 3 decimals, speed in knots 3, heading, roll, pitch and yaw rate 2.
 */
#ifndef PAOGI_H
#define PAOGI_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    bool     time_valid;
    uint32_t utc_ms;       /* UTC milliseconds of the day                   */
    bool     pos_valid;    /* lat, lon and alt; empty fields when false     */
    double   lat_deg, lon_deg;
    double   alt_msl_m;    /* above mean sea level, as in GGA               */
    int      quality;      /* GGA scale: 0 none, 1 single, 2 DGPS, 4 RTK fixed,
                              5 float, 6 dead reckoning                     */
    int      sats;         /* < 0: empty                                    */
    float    hdop;         /* < 0: empty                                    */
    float    age_s;        /* age of corrections, < 0: empty                */
    bool     speed_valid;
    float    speed_mps;
    bool     heading_valid;
    float    heading_deg;  /* clockwise from true north                     */
    bool     att_valid;    /* roll and pitch                                */
    float    roll_deg;     /* positive right side down                      */
    float    pitch_deg;    /* positive nose up                              */
    float    yaw_rate_dps; /* positive clockwise; always sent               */
} paogi_fix_t;

/* What main.c knows when a sentence is due, turned into the fields by
 * paogi_fill() (the field rules of plans/ethernet-w5500-agopenweb.md, kept
 * here so the host test covers them). */
typedef struct {
    /* Position: the ground point below the antenna when the antenna height
     * is set (ground_point.h), else the antenna; from the filter or the
     * receiver. AgOpenGPS / AgOpenWeb then run with an antenna height of 0. */
    bool     have_pos;       /* false: no position at all, quality 0         */
    bool     pos_from_ins;   /* the filter is navigating                     */
    double   lat_deg, lon_deg, h_ell_m;
    bool     have_undulation;
    float    undulation_m;   /* the receiver's, for the height above MSL     */

    /* The receiver's last epoch. */
    bool     have_nav;
    uint32_t tow_ms;         /* GPS time of week                             */
    int      leap_s;         /* GPS - UTC                                    */
    int64_t  ins_dt_ms;      /* filter output time - epoch time, added to the
                                time when the position is the filter's       */
    int      sats;
    int      rx_quality;     /* the epoch's position type on the GGA scale   */
    float    diff_age_s;     /* age of corrections, <= 0: none               */
    bool     gnss_fresh;     /* a usable receiver position within about 1 s  */
    bool     hdop_valid;
    float    hdop;

    /* Speed: the filter's while it navigates, else the receiver's. */
    bool     ins_vel_valid;
    float    vel_n_mps, vel_e_mps;
    bool     rx_speed_valid; /* fresh                                        */
    float    rx_speed_mps;

    /* Attitude: the filter's best source on a live IMU (roll, pitch), and
     * whether its yaw is the INS's in FULL or COASTING (the heading). */
    bool     att_valid;
    double   roll_rad, pitch_rad, yaw_rad;
    bool     heading_ok;
    float    yaw_sd_deg;     /* the INS's yaw 1-sigma, < 0: unknown (no heading) */
    float    heading_max_sd_deg; /* the heading is sent up to this yaw 1-sigma */

    /* Yaw rate: the corrected gyro (FRD, rad/s) summed since the last
     * sentence, and the filter's gyro bias. */
    uint32_t gyr_n;
    double   gyr_sum[3];
    bool     bias_valid;
    float    gyr_bias[3];
} paogi_in_t;

/* The fields from the inputs: quality 0 without a position; the receiver's
 * quality while it is the source or fresh; 6 while the filter runs on with
 * no fresh receiver position; the heading only with heading_ok while the
 * filter navigates and its yaw 1-sigma is known and at most
 * heading_max_sd_deg; the yaw rate as the NED yaw rate, clockwise positive. */
void paogi_fill(const paogi_in_t *in, paogi_fix_t *f);

/* Writes the sentence with its checksum and "\r\n" into buf, NUL-terminated.
 * Returns its length, or 0 if cap is too small (PAOGI_MAX is enough). */
#define PAOGI_MAX 160
size_t paogi_format(const paogi_fix_t *f, char *buf, size_t cap);

/* The USB command "paogi on" / "paogi off" (usb_log.c); on at boot. */
void paogi_set_enabled(bool on);
bool paogi_enabled(void);

#endif
