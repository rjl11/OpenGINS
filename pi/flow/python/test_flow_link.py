"""Link tests: the Python side must produce the example frame from
common/flow_link.h byte for byte (the C side is firmware/tools/test_flow_link.c).

    python3 -m unittest -v test_flow_link        (from pi/flow)
"""

import unittest
from pathlib import Path

import flow_link as fl

HEADER = Path(__file__).resolve().parents[3] / "common" / "flow_link.h"

EXAMPLE = fl.FlowMsg(
    frame_seq=1000,
    pi_time_us=123456789,
    pi_dt_us=3333,
    disp_x_rad=0.01,
    disp_y_rad=-0.002,
    stddev_rad=0.0005,
    n_tracked=87,
    exposure_100us=12,
    flags=fl.F_VALID | fl.F_TRIGGERED,
)


class LinkTest(unittest.TestCase):
    def test_crc_check_value(self):
        self.assertEqual(fl.crc16(b"123456789"), 0x29B1)

    def test_example_matches_header(self):
        want = fl.example_hex_from_header(HEADER)
        self.assertEqual(len(want), 2 * (5 + fl.FLOW_LEN + 2))
        self.assertEqual(fl.frame(fl.MSG_FLOW, EXAMPLE.payload()).hex(), want)

    def test_time_examples_match_header(self):
        req = fl.frame(fl.MSG_TIME_REQ, fl.time_req_payload(7))
        self.assertEqual(req.hex(), fl.example_hex_from_header(HEADER, "FLOW_LINK_EXAMPLE_TIME_REQ_HEX"))
        resp = fl.frame(fl.MSG_TIME_RESP, fl.time_resp_payload(7, 42, 0x1234567890))
        self.assertEqual(resp.hex(), fl.example_hex_from_header(HEADER, "FLOW_LINK_EXAMPLE_TIME_RESP_HEX"))
        msgs, _ = fl.parse(req + resp)
        self.assertEqual(fl.time_req_from_payload(msgs[0][1]), 7)
        self.assertEqual(fl.time_resp_from_payload(msgs[1][1]), (7, 42, 0x1234567890))

    def test_cam_vel_example_matches_header(self):
        m = fl.CamVelMsg(2000, 987654321, 10000, 1.5, -0.25, 0.02, 1.2, 0.03, 64, 8,
                         fl.F_VALID | fl.F_RANGE)
        f = fl.frame(fl.MSG_CAM_VEL, m.payload())
        self.assertEqual(f.hex(), fl.example_hex_from_header(HEADER, "FLOW_LINK_EXAMPLE_CAM_VEL_HEX"))
        msgs, _ = fl.parse(f)
        self.assertEqual(msgs[0][0], fl.MSG_CAM_VEL)
        got = fl.CamVelMsg.from_payload(msgs[0][1])
        self.assertEqual(got.frame_seq, 2000)
        self.assertAlmostEqual(got.height_m, 1.2, places=6)
        self.assertEqual(got.flags, fl.F_VALID | fl.F_RANGE)

    def test_unit_cfg_example_matches_header(self):
        f = fl.frame(fl.MSG_UNIT_CFG, fl.unit_cfg_payload(3, 1.25, 0.5, (0.0, -0.25, 1.5), fl.CFG_F_FLOW_ON))
        self.assertEqual(f.hex(), fl.example_hex_from_header(HEADER, "FLOW_LINK_EXAMPLE_UNIT_CFG_HEX"))
        msgs, _ = fl.parse(f)
        self.assertEqual(msgs[0][0], fl.MSG_UNIT_CFG)
        self.assertEqual(fl.unit_cfg_from_payload(msgs[0][1]), (3, 1.25, 0.5, (0.0, -0.25, 1.5), 1))

    def test_imu_batch_example_matches_header(self):
        s = [(0, (0.5, -0.25, 0.0), (0.0, 0.0, -9.8125)), (1250, (0.0, 0.0, 1.0), (1.0, -1.0, -9.8125))]
        f = fl.frame(fl.MSG_IMU_BATCH, fl.imu_batch_payload(1000000, 9, s))
        self.assertEqual(f.hex(), fl.example_hex_from_header(HEADER, "FLOW_LINK_EXAMPLE_IMU_BATCH_HEX"))
        msgs, _ = fl.parse(f)
        self.assertEqual(fl.imu_batch_from_payload(msgs[0][1]), (1000000, 9, 0, s))
        # Out of range: clamped and flagged.
        p = fl.imu_batch_payload(0, 0, [(0, (9.0, 0.0, 0.0), (0.0, 0.0, 0.0))])
        self.assertEqual(fl.imu_batch_from_payload(p)[2], fl.IMU_F_CLAMPED)

    def test_parse_round_trip_with_noise(self):
        f = fl.frame(fl.MSG_FLOW, EXAMPLE.payload())
        bad = bytearray(f)
        bad[12] ^= 0x40  # CRC fails, parser must resync on the next frame
        stream = b"\x00\xf7\xf7" + bytes(bad) + f + f[:7]
        msgs, rest = fl.parse(stream)
        self.assertEqual(len(msgs), 1)
        t, p = msgs[0]
        self.assertEqual(t, fl.MSG_FLOW)
        got = fl.FlowMsg.from_payload(p)
        self.assertEqual(got.frame_seq, 1000)
        self.assertAlmostEqual(got.disp_x_rad, 0.01, places=7)
        self.assertEqual(rest, f[:7])


if __name__ == "__main__":
    unittest.main()
