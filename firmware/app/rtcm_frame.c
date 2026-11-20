#include "rtcm_frame.h"

#include <string.h>

uint32_t rtcm_crc24q(const uint8_t *p, size_t n)
{
    uint32_t crc = 0;
    for (size_t i = 0; i < n; i++) {
        crc ^= (uint32_t)p[i] << 16;
        for (int b = 0; b < 8; b++) {
            crc <<= 1;
            if (crc & 0x1000000u) {
                crc ^= 0x1864CFBu;
            }
        }
    }
    return crc & 0xFFFFFFu;
}

unsigned rtcm_frame_msg_type(const uint8_t *f, size_t len)
{
    if (len < 3u + 2u + 3u) {
        return 0;
    }
    return ((unsigned)f[3] << 4) | (f[4] >> 4);
}

void rtcm_frame_init(rtcm_frame_t *r) { memset(r, 0, sizeof *r); }

/* Drops k bytes from the front. */
static void drop(rtcm_frame_t *r, size_t k)
{
    memmove(r->buf, r->buf + k, r->n - k);
    r->n -= k;
}

/* Works through what is buffered: delivers frames, drops junk, and stops
 * when the frame at the front is still incomplete. */
static void scan(rtcm_frame_t *r, rtcm_frame_cb cb, void *ctx)
{
    for (;;) {
        /* Find a preamble. */
        size_t k = 0;
        while (k < r->n && r->buf[k] != 0xD3) {
            k++;
        }
        if (k > 0) {
            r->skipped += (uint32_t)k;
            drop(r, k);
        }
        if (r->n < 3) {
            return;
        }
        if (r->buf[1] & 0xFC) { /* reserved bits set: not a frame */
            r->skipped++;
            drop(r, 1);
            continue;
        }
        size_t len = ((size_t)(r->buf[1] & 0x03) << 8) | r->buf[2];
        size_t total = 3u + len + 3u;
        if (r->n < total) {
            return;
        }
        uint32_t want = ((uint32_t)r->buf[3 + len] << 16) | ((uint32_t)r->buf[4 + len] << 8) | r->buf[5 + len];
        if (rtcm_crc24q(r->buf, 3u + len) == want) {
            r->frames++;
            cb(ctx, r->buf, total);
            drop(r, total);
        } else {
            r->crc_errors++;
            r->skipped++;
            drop(r, 1);
        }
    }
}

void rtcm_frame_push(rtcm_frame_t *r, const uint8_t *data, size_t len, rtcm_frame_cb cb, void *ctx)
{
    while (len > 0) {
        size_t room = sizeof r->buf - r->n;
        size_t take = len < room ? len : room;
        memcpy(r->buf + r->n, data, take);
        r->n += take;
        data += take;
        len -= take;
        scan(r, cb, ctx);
    }
}
