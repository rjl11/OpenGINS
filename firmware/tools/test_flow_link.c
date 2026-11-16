/* Host test for common/flow_link.h: CRC check value, and the example frame
 * (FLOW_LINK_EXAMPLE_HEX) decodes to the values its comment states. The Pi
 * side checks the same frame in pi/flow/test_flow_link.py. Build and run on
 * any host with a C compiler:
 *   gcc -O2 -Wall -Wextra -I common firmware/tools/test_flow_link.c -o /tmp/t && /tmp/t
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "flow_link.h"

static int fails;

#define CHECK(c)                                                    \
    do {                                                            \
        if (!(c)) {                                                 \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c);     \
            fails++;                                                \
        }                                                           \
    } while (0)

static size_t from_hex(const char *s, uint8_t *out, size_t max)
{
    size_t n = 0;
    while (s[0] && s[1] && n < max) {
        unsigned v;
        if (sscanf(s, "%2x", &v) != 1) break;
        out[n++] = (uint8_t)v;
        s += 2;
    }
    return n;
}

int main(void)
{
    CHECK(flow_link_crc16((const uint8_t *)"123456789", 9) == 0x29B1u);

    uint8_t f[FLOW_LINK_HEADER_LEN + FLOW_LINK_MAX_PAYLOAD + FLOW_LINK_CRC_LEN];
    size_t  n = from_hex(FLOW_LINK_EXAMPLE_HEX, f, sizeof f);
    CHECK(n == FLOW_LINK_HEADER_LEN + FLOW_MSG_FLOW_LEN + FLOW_LINK_CRC_LEN);
    CHECK(f[0] == FLOW_LINK_SYNC0 && f[1] == FLOW_LINK_SYNC1);
    CHECK(f[2] == FLOW_LINK_VERSION && f[3] == FLOW_MSG_FLOW && f[4] == FLOW_MSG_FLOW_LEN);

    size_t   len = f[4];
    uint16_t crc = (uint16_t)(f[5 + len] | (f[6 + len] << 8));
    CHECK(flow_link_crc16(&f[2], 3 + len) == crc);

    flow_msg_flow_t m = {0};
    CHECK(flow_link_decode_flow(&f[5], len, &m) == 0);
    CHECK(m.frame_seq == 1000u);
    CHECK(m.pi_time_us == 123456789u);
    CHECK(m.pi_dt_us == 3333u);
    CHECK(fabsf(m.disp_x_rad - 0.01f) < 1e-9f);
    CHECK(fabsf(m.disp_y_rad + 0.002f) < 1e-9f);
    CHECK(fabsf(m.stddev_rad - 0.0005f) < 1e-9f);
    CHECK(m.n_tracked == 87u);
    CHECK(m.exposure_100us == 12u);
    CHECK(m.flags == (FLOW_F_VALID | FLOW_F_TRIGGERED));
    CHECK(flow_link_decode_flow(&f[5], len - 1, &m) == -1);

    /* The encoder rebuilds the example byte for byte. */
    uint8_t e[FLOW_LINK_FLOW_FRAME_LEN + 4];
    CHECK(flow_link_encode_flow(&m, e, sizeof e) == FLOW_LINK_FLOW_FRAME_LEN);
    CHECK(memcmp(e, f, FLOW_LINK_FLOW_FRAME_LEN) == 0);
    CHECK(flow_link_encode_flow(&m, e, FLOW_LINK_FLOW_FRAME_LEN - 1) == 0);

    f[10] ^= 1; /* one bit flipped in the payload */
    CHECK(flow_link_crc16(&f[2], 3 + len) != crc);

    printf(fails ? "test_flow_link: %d FAILED\n" : "test_flow_link: OK\n", fails);
    return fails ? EXIT_FAILURE : EXIT_SUCCESS;
}
