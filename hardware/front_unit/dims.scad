// Front unit housing: every dimension of the bought parts, in mm, with its
// source. Caliper readings by the user are dated; figures read off a
// drawing's proportions rather than a dimension are marked "estimate".

$fn = 64;
eps = 0.01;

// ---------------------------------------------------------------- box
wall      = 2.5;   // side walls
floor_t   = 3.0;   // floor, faces the ground
lid_t     = 3.0;
fit       = 0.4;   // clearance around a part in a pocket

// ---------------------------------------------------------------- OV9281
// external/CAM-OV9281RAW-V2/CAM-OV9281RAW V2 User Manual V1.4.pdf, p.6,
// and "Lens drawing/Lens Mount.png", "Lens drawing/Lens.png".
cam_pcb        = 32.0;   // square PCB
cam_pcb_t      = 1.6;
cam_hole_pitch = 28.0;   // 4 holes on a 28 x 28 square
cam_holder     = 17.1;   // M12 holder body, square
cam_holder_ear = 24.2;   // over its two ears
cam_lens_d     = 14.0;   // lens front
// Lens front to the PCB's holder-side face, lens focused (calipers, 2026-11-25).
cam_lens_drop  = 25.0;
// FPC connector top to the PCB's holder-side face is 4 mm (calipers,
// 2026-11-25), so this much above the back.
cam_back_h     = 4.0 - cam_pcb_t;
cam_fpc_w      = 16.0;
cam_fov_half   = 74;     // 148 deg diagonal FoV
cam_focus_d    = 22.0;   // floor hole round the lens: room for fingers to turn it (user, 2026-11-26)
cam_corner_t   = 1.6;    // locating corners round the PCB: wall, legs, height above the PCB underside
cam_corner_leg = 5.0;
cam_corner_h   = 1.5;
tfl_cable_gap  = 6.5;    // camera PCB edge to TF-Luna body: its cable leaves on that side (user, 2026-11-26: 5 mm more)

// ---------------------------------------------------------------- TM171
// docs/datasheets/TransducerM_TM151_TM171_Datasheet_EN_V116-R.pdf, p.7,
// and SYD's STEP models of the core and the baseboard, combined by
// tm171_combine.py (external/syd_dynamics/TransducerM_TM171_Combined.*).
// Measured on the models; "offset" is towards the pin edge (+x).
tm_l      = 40.0;   // baseboard, along the TM171's x axis (pins at +x)
tm_w      = 34.0;
tm_h      = 12.6;   // baseboard + core
tm_base_t = 1.6;
tm_case_l = 26.0;   // aluminium core, along x
tm_case_w = 30.0;
tm_case_off = 0.7;  // core centre from the board centre
// Baseboard mounting holes, Ø2.1 (M2), at the board's corners.
tm_hole_dx  = 31.0;   // along x
tm_hole_dy  = 30.0;   // along y
tm_hole_off = 0.406;  // pattern centre from the board centre
tm_pin_in   = 2.44;   // 5 pins at 2.54, Ø1.2 holes, this far in from the +x edge
// USB-C on the top side at the -x edge, from y = -10.52 to -1.88 in the
// TM171's y (its y is the box's +y when it lies case up, x along the box),
// 3.2 high.
tm_usb_y  = [1.88, 10.52];   // magnitudes, on the -y side

// ---------------------------------------------------------------- TF-Luna
// docs/datasheets/SJ-PM-TF-Luna_A05_Product_Manual.pdf, p.11, fig. 3, and
// the user (2026-11-25): the ears are flush with the back (connector side).
tfl_l          = 35.0;   // over the ears
tfl_body_l     = 26.5;   // body, along the lens row
tfl_w          = 21.25;
tfl_body_h     = 12.0;   // front face to back
tfl_rim_h      = 1.5;    // lens rims proud of the front face (13.5 overall)
tfl_lens_d     = 10.5;
tfl_lens_pitch = 10.5;   // emitter and receiver side by side along l
tfl_lens_off   = 9.0;    // lens centres from one long edge (that edge faces the camera)
tfl_hole_pitch = 30.5;
tfl_ear_t      = 2.0;
tfl_ear_z      = tfl_body_h - tfl_ear_t;   // ear underside above the front face
tfl_ear_off    = tfl_lens_off;   // ear holes on the lens centreline (user, 2026-11-25)

// ---------------------------------------------------------------- ROCK 4SE
// Radxa ROCK 4SE product brief rev 1.8, p.5 (mechanical drawing, top view
// drawn as seen from below), and the V1.53 schematic p.5 (J8203 = camera).
// Board corner (0,0): the mounting-hole / uSD end, on the power edge
// (USB-C, HDMI, audio); x along the 85 mm edge towards the RJ45 / USB end,
// y across towards the 40-pin header. [x0, x1] / [y0, y1] ranges in mm.
rk_l = 85.0;  rk_w = 56.0;  rk_t = 1.6;
rk_hole_in = 3.5;  rk_hole_dx = 58.0;  rk_hole_dy = 49.0;
rk_usbc  = [[7.0, 14.3], [-1.1, 4.1], 3.3];    // x, y, height
rk_hdmi  = [[24.8, 39.3], [-1.2, 8.1], 6.5];   // full size, height estimate
rk_audio = [[49.9, 55.9], [-2.2, 12.0], 6.0];  // y1 and height estimate
rk_csi   = [[42.2, 47.8], [0.9, 22.0], 3.0];   // J8203 camera connector, height estimate
rk_rj45  = [[66.3, 86.8], [2.4, 18.2], 14.0];  // height: user, 2026-11-25
rk_usb   = [[[69.1, 86.8], [22.4, 35.7]], [[70.3, 86.8], [40.2, 54.0]]];
rk_usb_h = 16.0;                                // user, 2026-11-25
rk_overhang = 86.8 - rk_l;                      // RJ45 / USB past the board end
rk_header_y = 52.6;                             // 40-pin header centre line
rk_header_x = [6.6, 58.4];
// Underside: the RK3399 (centre 41.7, 27.3, ~21 mm square at 45 deg), the
// eMMC socket, the M.2 slot and the two LPDDR4 (bounding boxes from the
// drawing). The user: 5 mm clearance under the board is enough.
rk_under = 5.0;
// [x range, y range, 0 = box / 1 = diamond (square turned 45 deg, corners
// at the middle of each side of the range)]
rk_under_parts = [
    [[0.4, 14.7], [14.1, 32.7], 0],    // M.2 slot
    [[5.4, 23.9], [35.0, 48.5], 0],    // eMMC socket
    [[26.7, 56.7], [12.3, 42.3], 1],   // RK3399
    [[46.5, 68.4], [4.8, 25.3], 1],    // LPDDR4
    [[46.5, 68.4], [28.0, 49.9], 1],   // LPDDR4
];
// No heatsink and no fan for now (user, 2026-11-25: winter).

// ---------------------------------------------------------------- fasteners
// Screws thread straight into the plastic, no inserts (user, 2026-11-25).
// Pilot holes at about 0.85 x the thread size: a common rule for machine
// screws cut into PETG, not a vendor figure. stl/tap_test.stl tries each at
// -0.1 / nominal / +0.1 mm; set the one that grips without splitting here.
tap_m2   = 1.7;   // M2: camera, TM171, TF-Luna
tap_m25  = 2.1;   // M2.5: ROCK 4SE
tap_m3   = 2.5;   // M3: lid
tap_m5   = 4.2;   // M5: bracket
tap_m3_depth = 9;    // lid screws M3 x 10 (3 mm lid + 7 in the post)
tap_m5_depth = 12;   // bracket screws, up to 10 mm into the wall
clear_m3 = 3.4;
// Camera, TM171 and TF-Luna: "stake" = printed pins through their M2 holes
// (Ø2.1-2.2), tops melted over with a soldering iron (user, 2026-11-26: no
// M2 screws); "screw" = M2 pilot holes instead.
m2_fix    = "stake";
stake_d   = 1.8;   // pin, 0.3-0.4 mm under the holes
stake_len = 2.0;   // pin above the part, melted into a head
slot_m6  = 6.6;

fpc_len  = 150;   // camera FPC
fpc_slack = 20;   // asked for on top of the routed length
