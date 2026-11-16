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

/* Writes the sentence with its checksum and "\r\n" into buf, NUL-terminated.
 * Returns its length, or 0 if cap is too small (PAOGI_MAX is enough). */
#define PAOGI_MAX 160
size_t paogi_format(const paogi_fix_t *f, char *buf, size_t cap);

/* The USB command "paogi on" / "paogi off" (usb_log.c); on at boot. */
void paogi_set_enabled(bool on);
bool paogi_enabled(void);

#endif
