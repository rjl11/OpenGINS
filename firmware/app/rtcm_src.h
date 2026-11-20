/*
 * RTCM correction source arbiter: only one source feeds the receiver at a
 * time (plans/ethernet-w5500-agopenweb.md, "RTCM over Ethernet";
 * plans/rtcm-over-bluetooth.md). Frames from the other source are dropped,
 * so two casters' streams never mix in the receiver.
 *
 *  - auto: the first source to deliver a frame is taken, and kept until it
 *    has been silent for hold_us; then the next source to deliver wins.
 *  - eth / bt: only that source; the other's frames are always dropped.
 *
 * No hardware calls: host-tested in firmware/tools/test_rtcm_src.c.
 */
#ifndef RTCM_SRC_H
#define RTCM_SRC_H

#include <stdbool.h>
#include <stdint.h>

enum { RTCM_SRC_ETH = 0, RTCM_SRC_BT, RTCM_SRC_N };

/* settings_t.rtcm_src */
#define RTCM_SRC_MODE_AUTO 0
#define RTCM_SRC_MODE_ETH  1
#define RTCM_SRC_MODE_BT   2

typedef struct {
    uint32_t frames;   /* accepted                 */
    uint32_t dropped;  /* refused: not the source  */
    uint64_t t_last;   /* last frame offered, 0 = never */
} rtcm_src_stat_t;

typedef struct {
    unsigned mode;
    int      active;   /* RTCM_SRC_*, -1 = none */
    uint64_t hold_us;
    uint32_t switches;
    rtcm_src_stat_t s[RTCM_SRC_N];
} rtcm_src_t;

void rtcm_src_init(rtcm_src_t *r, uint64_t hold_us);
void rtcm_src_set_mode(rtcm_src_t *r, unsigned mode);

/* One whole frame from src at time now: true = pass it to the receiver. */
bool rtcm_src_accept(rtcm_src_t *r, unsigned src, uint64_t now);

/* The source in use at now (auto: -1 once the active one has gone silent). */
int rtcm_src_active(const rtcm_src_t *r, uint64_t now);

/* "ethernet", "bluetooth"; "auto" etc. for the modes. NULL if unknown. */
const char *rtcm_src_name(int src);
const char *rtcm_src_mode_name(unsigned mode);

#endif /* RTCM_SRC_H */
