/* Host test for common/flow_link.h: CRC check value, and the example frame
 * (FLOW_LINK_EXAMPLE_HEX) decodes to the values its comment states. The Pi
 * side checks the same frame in pi/flow/test_flow_link.py. Build and run on
 * any host with a C compiler:
 *   gcc -O2 -Wall -Wextra -I common firmware/tools/test_flow_link.c -o /tmp/t && /tmp/t
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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
    uint8_t type = 0;
    CHECK(flow_link_check(f, n, &type) == -1);
    f[10] ^= 1;
    CHECK(flow_link_check(f, n, &type) == (int)FLOW_MSG_FLOW_LEN && type == FLOW_MSG_FLOW);

    /* Clock sync: both examples decode and are rebuilt byte for byte. */
    uint8_t q[32], r[32];
    size_t  qn = from_hex(FLOW_LINK_EXAMPLE_TIME_REQ_HEX, q, sizeof q);
    CHECK(qn == FLOW_LINK_TIME_REQ_FRAME_LEN);
    CHECK(flow_link_check(q, qn, &type) == (int)FLOW_MSG_TIME_REQ_LEN && type == FLOW_MSG_TIME_REQ);
    uint32_t seq = 0;
    CHECK(flow_link_decode_time_req(&q[5], FLOW_MSG_TIME_REQ_LEN, &seq) == 0 && seq == 7u);
    CHECK(flow_link_encode_time_req(7u, e, sizeof e) == qn && memcmp(e, q, qn) == 0);

    size_t rn = from_hex(FLOW_LINK_EXAMPLE_TIME_RESP_HEX, r, sizeof r);
    CHECK(rn == FLOW_LINK_TIME_RESP_FRAME_LEN);
    CHECK(flow_link_check(r, rn, &type) == (int)FLOW_MSG_TIME_RESP_LEN && type == FLOW_MSG_TIME_RESP);
    flow_msg_time_resp_t tr = {0};
    CHECK(flow_link_decode_time_resp(&r[5], FLOW_MSG_TIME_RESP_LEN, &tr) == 0);
    CHECK(tr.seq == 7u && tr.turnaround_us == 42u && tr.unit_rx_us == 0x1234567890ull);
    uint8_t e2[32];
    CHECK(flow_link_encode_time_resp(&tr, e2, sizeof e2) == rn && memcmp(e2, r, rn) == 0);
    CHECK(flow_link_decode_time_resp(&r[5], FLOW_MSG_TIME_RESP_LEN - 1, &tr) == -1);

    /* Camera velocity: decodes to the values in its comment, rebuilt byte
     * for byte. */
    uint8_t c[64];
    size_t  cn = from_hex(FLOW_LINK_EXAMPLE_CAM_VEL_HEX, c, sizeof c);
    CHECK(cn == FLOW_LINK_CAM_VEL_FRAME_LEN);
    CHECK(flow_link_check(c, cn, &type) == (int)FLOW_MSG_CAM_VEL_LEN && type == FLOW_MSG_CAM_VEL);
    flow_msg_cam_vel_t cv = {0};
    CHECK(flow_link_decode_cam_vel(&c[5], FLOW_MSG_CAM_VEL_LEN, &cv) == 0);
    CHECK(cv.frame_seq == 2000u && cv.pi_time_us == 987654321u && cv.pi_dt_us == 10000u);
    CHECK(cv.vel_x_mps == 1.5f && cv.vel_y_mps == -0.25f && cv.stddev_mps == 0.02f);
    CHECK(cv.height_m == 1.2f && cv.height_sd_m == 0.03f);
    CHECK(cv.n_tracked == 64u && cv.exposure_100us == 8u && cv.flags == (FLOW_F_VALID | FLOW_F_RANGE));
    uint8_t e3[64];
    CHECK(flow_link_encode_cam_vel(&cv, e3, sizeof e3) == cn && memcmp(e3, c, cn) == 0);
    CHECK(flow_link_decode_cam_vel(&c[5], FLOW_MSG_CAM_VEL_LEN - 1, &cv) == -1);

    /* Unit settings (Pico -> unit). */
    uint8_t u[64];
    size_t  un = from_hex(FLOW_LINK_EXAMPLE_UNIT_CFG_HEX, u, sizeof u);
    CHECK(un == FLOW_LINK_UNIT_CFG_FRAME_LEN);
    CHECK(flow_link_check(u, un, &type) == (int)FLOW_MSG_UNIT_CFG_LEN && type == FLOW_MSG_UNIT_CFG);
    flow_msg_unit_cfg_t uc = {0};
    CHECK(flow_link_decode_unit_cfg(&u[5], FLOW_MSG_UNIT_CFG_LEN, &uc) == 0);
    CHECK(uc.seq == 3u && uc.height_m == 1.25f && uc.cam_yaw_rad == 0.5f);
    CHECK(uc.imu_rpy_rad[0] == 0.0f && uc.imu_rpy_rad[1] == -0.25f && uc.imu_rpy_rad[2] == 1.5f);
    CHECK(uc.flags == FLOW_CFG_F_FLOW_ON);
    uint8_t e4[64];
    CHECK(flow_link_encode_unit_cfg(&uc, e4, sizeof e4) == un && memcmp(e4, u, un) == 0);
    CHECK(flow_link_decode_unit_cfg(&u[5], FLOW_MSG_UNIT_CFG_LEN + 1, &uc) == -1);

    /* IMU batch: decodes to the values in its comment, rebuilt byte for
     * byte; out of range is clamped and flagged; a bad length refused. */
    uint8_t ib[160];
    size_t  ibn = from_hex(FLOW_LINK_EXAMPLE_IMU_BATCH_HEX, ib, sizeof ib);
    CHECK(ibn == FLOW_LINK_HEADER_LEN + 8u + 2u * 14u + FLOW_LINK_CRC_LEN);
    CHECK(flow_link_check(ib, ibn, &type) == 36 && type == FLOW_MSG_IMU_BATCH);
    flow_msg_imu_batch_t bt;
    memset(&bt, 0, sizeof bt);
    CHECK(flow_link_decode_imu_batch(&ib[5], 36, &bt) == 0);
    CHECK(bt.t0_us == 1000000u && bt.seq == 9u && bt.n == 2u && bt.flags == 0u);
    CHECK(bt.s[0].dt_us == 0u && bt.s[0].gyr[0] == 0.5f && bt.s[0].gyr[1] == -0.25f && bt.s[0].acc[2] == -9.8125f);
    CHECK(bt.s[1].dt_us == 1250u && bt.s[1].gyr[2] == 1.0f && bt.s[1].acc[0] == 1.0f && bt.s[1].acc[1] == -1.0f);
    uint8_t e5[160];
    CHECK(flow_link_encode_imu_batch(&bt, e5, sizeof e5) == ibn && memcmp(e5, ib, ibn) == 0);
    CHECK(flow_link_decode_imu_batch(&ib[5], 35, &bt) == -1);
    flow_msg_imu_batch_t big;
    memset(&big, 0, sizeof big);
    big.n = 1;
    big.s[0].gyr[0] = 9.0f;
    CHECK(flow_link_encode_imu_batch(&big, e5, sizeof e5) != 0);
    CHECK(flow_link_decode_imu_batch(&e5[5], 22, &bt) == 0 && (bt.flags & FLOW_IMU_F_CLAMPED) && bt.s[0].gyr[0] < 8.0f);
    big.n = FLOW_IMU_BATCH_MAX;
    CHECK(flow_link_encode_imu_batch(&big, e5, sizeof e5) == FLOW_LINK_HEADER_LEN + 120u + FLOW_LINK_CRC_LEN);
    big.n = FLOW_IMU_BATCH_MAX + 1u;
    CHECK(flow_link_encode_imu_batch(&big, e5, sizeof e5) == 0);

    printf(fails ? "test_flow_link: %d FAILED\n" : "test_flow_link: OK\n", fails);
    return fails ? EXIT_FAILURE : EXIT_SUCCESS;
}
