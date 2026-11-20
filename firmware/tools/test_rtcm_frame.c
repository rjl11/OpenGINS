/*
 * Host test for app/rtcm_frame.c (RTCM 3 framing across UDP chunks).
 *
 *   gcc -O2 -Wall -Wextra -I firmware/app firmware/tools/test_rtcm_frame.c firmware/app/rtcm_frame.c -o temp/test_rtcm_frame && temp/test_rtcm_frame
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rtcm_frame.h"

static int s_fail;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); s_fail++; } } while (0)

/* A frame with message type t and n bytes of message (n >= 2). */
static size_t make_frame(uint8_t *out, unsigned t, size_t n, uint8_t seed)
{
    out[0] = 0xD3;
    out[1] = (uint8_t)(n >> 8);
    out[2] = (uint8_t)n;
    out[3] = (uint8_t)(t >> 4);
    out[4] = (uint8_t)((t & 0xF) << 4);
    for (size_t i = 2; i < n; i++) {
        out[3 + i] = (uint8_t)(seed + i * 7u); /* includes 0xD3 bytes now and then */
    }
    uint32_t c = rtcm_crc24q(out, 3 + n);
    out[3 + n] = (uint8_t)(c >> 16);
    out[4 + n] = (uint8_t)(c >> 8);
    out[5 + n] = (uint8_t)c;
    return 6 + n;
}

typedef struct {
    unsigned types[64];
    size_t lens[64];
    int n;
} got_t;

static void on_frame(void *ctx, const uint8_t *f, size_t len)
{
    got_t *g = ctx;
    if (g->n < 64) {
        g->types[g->n] = rtcm_frame_msg_type(f, len);
        g->lens[g->n] = len;
        g->n++;
    }
}

int main(void)
{
    /* CRC-24Q check value. */
    CHECK(rtcm_crc24q((const uint8_t *)"123456789", 9) == 0xCDE703u);

    static uint8_t stream[8192];
    size_t sn = 0;
    const unsigned types[] = { 1005, 1077, 1087, 1097, 1127, 1230, 1006, 1077 };
    const size_t lens[] = { 19, 400, 300, 350, 1023, 8, 21, 2 };
    size_t frame_at[8];
    /* Junk first, including a 0xD3 that is no frame. */
    const uint8_t junk[] = { 0x00, 0xD3, 0xFF, 0x12, 0xD3, 0x00 };
    memcpy(stream, junk, sizeof junk);
    sn = sizeof junk;
    for (int i = 0; i < 8; i++) {
        frame_at[i] = sn;
        sn += make_frame(stream + sn, types[i], lens[i], (uint8_t)i);
    }

    /* 1. In 256-byte chunks, as AgOpenWeb sends them. */
    {
        rtcm_frame_t r;
        got_t g = { 0 };
        rtcm_frame_init(&r);
        for (size_t o = 0; o < sn; o += 256) {
            size_t k = sn - o < 256 ? sn - o : 256;
            rtcm_frame_push(&r, stream + o, k, on_frame, &g);
        }
        CHECK(g.n == 8);
        for (int i = 0; i < g.n && i < 8; i++) {
            CHECK(g.types[i] == types[i]);
            CHECK(g.lens[i] == lens[i] + 6);
        }
        CHECK(r.frames == 8);
        CHECK(r.n == 0);
    }

    /* 2. One byte at a time and in random chunks: the same frames. */
    for (int pass = 0; pass < 20; pass++) {
        rtcm_frame_t r;
        got_t g = { 0 };
        rtcm_frame_init(&r);
        srand((unsigned)pass);
        for (size_t o = 0; o < sn;) {
            size_t k = pass == 0 ? 1 : 1 + (size_t)(rand() % 700);
            if (k > sn - o) k = sn - o;
            rtcm_frame_push(&r, stream + o, k, on_frame, &g);
            o += k;
        }
        CHECK(g.n == 8);
    }

    /* 3. One corrupted byte in frame 2: only that frame is lost. */
    {
        static uint8_t bad[8192];
        memcpy(bad, stream, sn);
        bad[frame_at[2] + 50] ^= 0x01;
        rtcm_frame_t r;
        got_t g = { 0 };
        rtcm_frame_init(&r);
        for (size_t o = 0; o < sn; o += 256) {
            size_t k = sn - o < 256 ? sn - o : 256;
            rtcm_frame_push(&r, bad + o, k, on_frame, &g);
        }
        CHECK(g.n == 7);
        CHECK(r.crc_errors >= 1);
        CHECK(g.n >= 3 && g.types[2] == types[3]);
    }

    /* 4. A lost chunk (a dropped datagram) in the middle of frame 1. */
    {
        static uint8_t cut[8192];
        size_t a = frame_at[1] + 100, b = a + 256;
        memcpy(cut, stream, a);
        memcpy(cut + a, stream + b, sn - b);
        size_t cn = sn - 256;
        rtcm_frame_t r;
        got_t g = { 0 };
        rtcm_frame_init(&r);
        for (size_t o = 0; o < cn; o += 256) {
            size_t k = cn - o < 256 ? cn - o : 256;
            rtcm_frame_push(&r, cut + o, k, on_frame, &g);
        }
        /* Frame 1 is lost; whether frame 2 survives depends on where the
         * false length lands, but 3..7 must all come through. */
        CHECK(g.n >= 6);
        CHECK(g.types[0] == types[0]);
        CHECK(g.types[g.n - 1] == types[7]);
    }

    if (s_fail == 0) {
        printf("test_rtcm_frame: all passed\n");
        return 0;
    }
    printf("test_rtcm_frame: %d failed\n", s_fail);
    return 1;
}
