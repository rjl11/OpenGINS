#include "rtcm_src.h"

#include <string.h>

void rtcm_src_init(rtcm_src_t *r, uint64_t hold_us)
{
    memset(r, 0, sizeof *r);
    r->active = -1;
    r->hold_us = hold_us;
}

void rtcm_src_set_mode(rtcm_src_t *r, unsigned mode)
{
    if (mode == r->mode) {
        return;
    }
    r->mode = mode;
    r->active = mode == RTCM_SRC_MODE_ETH ? RTCM_SRC_ETH : mode == RTCM_SRC_MODE_BT ? RTCM_SRC_BT : -1;
}

int rtcm_src_active(const rtcm_src_t *r, uint64_t now)
{
    if (r->mode != RTCM_SRC_MODE_AUTO || r->active < 0) {
        return r->active;
    }
    return now - r->s[r->active].t_last < r->hold_us ? r->active : -1;
}

bool rtcm_src_accept(rtcm_src_t *r, unsigned src, uint64_t now)
{
    if (src >= RTCM_SRC_N) {
        return false;
    }
    rtcm_src_stat_t *st = &r->s[src];
    if (r->mode == RTCM_SRC_MODE_AUTO && rtcm_src_active(r, now) < 0) {
        if (r->active >= 0 && r->active != (int)src) {
            r->switches++;
        }
        r->active = (int)src;
    }
    st->t_last = now;
    if (r->active != (int)src) {
        st->dropped++;
        return false;
    }
    st->frames++;
    return true;
}

const char *rtcm_src_name(int src)
{
    switch (src) {
    case RTCM_SRC_ETH: return "ethernet";
    case RTCM_SRC_BT:  return "bluetooth";
    default:           return NULL;
    }
}

const char *rtcm_src_mode_name(unsigned mode)
{
    switch (mode) {
    case RTCM_SRC_MODE_AUTO: return "auto";
    case RTCM_SRC_MODE_ETH:  return "ethernet";
    case RTCM_SRC_MODE_BT:   return "bluetooth";
    default:                 return NULL;
    }
}
