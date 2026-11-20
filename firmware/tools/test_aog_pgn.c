/*
 * Host test for app/aog_pgn.c (AgOpenGPS scan / hello replies) and
 * app/rtcm_src.c (one RTCM correction source at a time).
 *
 *   gcc -O2 -Wall -Wextra -I firmware/app firmware/tools/test_aog_pgn.c firmware/app/aog_pgn.c firmware/app/rtcm_src.c -o temp/test_aog_pgn && temp/test_aog_pgn
 */
#include <stdio.h>
#include <string.h>

#include "aog_pgn.h"
#include "rtcm_src.h"

static int s_fail;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); s_fail++; } } while (0)

static void test_pgn(void)
{
    /* AgIO's scan request: 80 81 7F CA 03 CA CA 05, then a constant 0x47
     * (the sum would be 0xE5); both are taken. */
    const uint8_t scan[] = { 0x80, 0x81, 0x7F, 0xCA, 0x03, 0xCA, 0xCA, 0x05, 0x47 };
    CHECK(aog_pgn_crc(scan, 8) == 0xE5);
    CHECK(aog_pgn_parse(scan, sizeof scan) == AOG_REQ_SCAN);
    uint8_t bad[sizeof scan];
    memcpy(bad, scan, sizeof scan);
    bad[8] = 0xE5;
    CHECK(aog_pgn_parse(bad, sizeof bad) == AOG_REQ_SCAN);
    bad[7] = 4; /* wrong fixed byte */
    CHECK(aog_pgn_parse(bad, sizeof bad) == AOG_REQ_NONE);
    CHECK(aog_pgn_parse(scan, 5) == AOG_REQ_NONE);
    CHECK(aog_pgn_parse(scan, 8) == AOG_REQ_NONE); /* CRC byte missing */

    /* Hello from AgIO: 80 81 7F C8 03 data(3) CRC. */
    uint8_t hello[] = { 0x80, 0x81, 0x7F, 0xC8, 0x03, 0x38, 0x00, 0x00, 0 };
    hello[8] = aog_pgn_crc(hello, 8);
    CHECK(aog_pgn_parse(hello, sizeof hello) == AOG_REQ_HELLO);

    /* Some other PGN (e.g. 254 steer data) is not answered. */
    uint8_t other[] = { 0x80, 0x81, 0x7F, 0xFE, 0x01, 0x00, 0 };
    other[6] = aog_pgn_crc(other, 6);
    CHECK(aog_pgn_parse(other, sizeof other) == AOG_REQ_NONE);

    const uint8_t ip[4] = { 192, 168, 5, 124 };
    uint8_t out[AOG_PGN_MAX];
    size_t n = aog_pgn_reply(AOG_REQ_SCAN, ip, out, sizeof out);
    const uint8_t want_scan[] = { 0x80, 0x81, 0x78, 0xCB, 0x07, 192, 168, 5, 124, 192, 168, 5, 0 };
    CHECK(n == sizeof want_scan);
    CHECK(memcmp(out, want_scan, 12) == 0);
    CHECK(out[12] == aog_pgn_crc(out, 12));

    n = aog_pgn_reply(AOG_REQ_HELLO, ip, out, sizeof out);
    /* 80 81 78 78 05 00 00 00 00 00 CRC, CRC = 78+78+05 = 0xF5 */
    const uint8_t want_hello[] = { 0x80, 0x81, 0x78, 0x78, 0x05, 0, 0, 0, 0, 0, 0xF5 };
    CHECK(n == sizeof want_hello && memcmp(out, want_hello, n) == 0);
    CHECK(aog_pgn_reply(AOG_REQ_NONE, ip, out, sizeof out) == 0);
}

static void test_src(void)
{
    const uint64_t S = 1000000u;
    rtcm_src_t r;
    rtcm_src_init(&r, 5 * S);

    /* auto: Ethernet first, it holds; Bluetooth dropped meanwhile. */
    CHECK(rtcm_src_active(&r, 1 * S) == -1);
    CHECK(rtcm_src_accept(&r, RTCM_SRC_ETH, 1 * S));
    CHECK(!rtcm_src_accept(&r, RTCM_SRC_BT, 2 * S));
    CHECK(rtcm_src_accept(&r, RTCM_SRC_ETH, 3 * S));
    CHECK(rtcm_src_active(&r, 7 * S) == RTCM_SRC_ETH);
    /* Ethernet silent for 5 s: Bluetooth takes over, and then holds. */
    CHECK(rtcm_src_active(&r, 8 * S + 1) == -1);
    CHECK(rtcm_src_accept(&r, RTCM_SRC_BT, 9 * S));
    CHECK(!rtcm_src_accept(&r, RTCM_SRC_ETH, 10 * S));
    CHECK(r.switches == 1);
    CHECK(r.s[RTCM_SRC_ETH].frames == 2 && r.s[RTCM_SRC_ETH].dropped == 1);
    CHECK(r.s[RTCM_SRC_BT].frames == 1 && r.s[RTCM_SRC_BT].dropped == 1);

    /* Fixed to Ethernet: Bluetooth never passes, even with Ethernet silent. */
    rtcm_src_set_mode(&r, RTCM_SRC_MODE_ETH);
    CHECK(rtcm_src_active(&r, 100 * S) == RTCM_SRC_ETH);
    CHECK(!rtcm_src_accept(&r, RTCM_SRC_BT, 100 * S));
    CHECK(rtcm_src_accept(&r, RTCM_SRC_ETH, 101 * S));
    /* Back to auto: nothing active until a source delivers. */
    rtcm_src_set_mode(&r, RTCM_SRC_MODE_AUTO);
    CHECK(rtcm_src_active(&r, 101 * S) == -1);
    CHECK(rtcm_src_accept(&r, RTCM_SRC_BT, 102 * S));
    CHECK(!rtcm_src_accept(&r, RTCM_SRC_N, 102 * S));

    CHECK(strcmp(rtcm_src_mode_name(RTCM_SRC_MODE_BT), "bluetooth") == 0);
    CHECK(rtcm_src_mode_name(9) == NULL);
}

int main(void)
{
    test_pgn();
    test_src();
    if (s_fail == 0) {
        printf("test_aog_pgn: all passed\n");
        return 0;
    }
    printf("test_aog_pgn: %d failed\n", s_fail);
    return 1;
}
