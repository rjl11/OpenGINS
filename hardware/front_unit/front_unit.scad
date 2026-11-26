// Front unit housing: OV9281 camera, ROCK 4SE, TM171, TF-Luna.
// Bench / field-test version: vented lid, open lens aperture, no fan.
//
//   openscad -o stl/base.stl -D 'part="base"' front_unit.scad
//
// part = "base" | "lid" | "tap_test" | "assembly" | "open" | "section"
//        | "check" (with check_a / check_b, used by check.sh)
//
// Axes: z up, away from the ground (the floor faces the ground); x along
// the long side, from the TM171 end towards the TF-Luna / RJ45 end.
// The cavity's inner corner is the origin; the floor's underside is z = 0.

include <dims.scad>

part = "assembly";
check_a = "base";
check_b = "cam";

// ================================================================ layout
gap      = 2;
rim      = 1.6;                       // TM171 pocket rim
rk_y0    = 1;                         // board edge to the port wall
inner_y  = rk_y0 + rk_w + 6;          // 6 mm on the header side for wires
comp_y   = rk_y0 + rk_hole_in + rk_hole_dy / 2;   // floor parts' centre line

// TM171: platform with a pocket, at the x = 0 end
tmp_x0   = gap;
tmp_l    = tm_l + 2 * fit + 2 * rim;
tmp_w    = tm_w + 2 * fit + 2 * rim;
tmp_h    = 3;                         // platform above the floor
tm_c     = [tmp_x0 + tmp_l / 2, comp_y];

// Camera next to it
cam_x    = tmp_x0 + tmp_l + 3 + cam_pcb / 2;   // locating corners 1 mm clear of the platform
cam_c    = [cam_x, comp_y];
lens_z   = 0.5;                       // lens front above the floor's underside
cam_pcb_z = lens_z + cam_lens_drop;   // PCB underside

// TF-Luna next to the camera, long side along y
tfl_x0   = cam_x + cam_pcb / 2 + tfl_cable_gap; // body edge on the camera side
tfl_c    = [tfl_x0 + tfl_w / 2, comp_y];
tfl_pad  = 3;                         // pad under it, holds the screws
tfl_lens_x = tfl_x0 + tfl_lens_off;
cam_tfl_offset = tfl_lens_x - cam_x;

inner_x  = tfl_x0 + tfl_w + gap;

// ROCK 4SE above, RJ45 / USB end at the x = inner_x wall
rk_x0    = inner_x - 0.5 - rk_overhang - rk_l;
function mid(r) = (r[0] + r[1]) / 2;
// The camera sits under the RK3399 / LPDDR4 on the board's underside, so
// the board clears the camera's FPC connector by rk_under.
rk_z     = max(cam_pcb_z + cam_pcb_t + cam_back_h + rk_under + 0.5,
               floor_t + tmp_h + tm_h + rk_under + 1);
rk_top   = rk_z + rk_t;
wall_h   = rk_top + max(rk_usb_h, rk_rj45[2]) + 3.5;
rk_holes = [for (i = [0, 1], j = [0, 1])
            [rk_x0 + rk_hole_in + i * rk_hole_dx, rk_y0 + rk_hole_in + j * rk_hole_dy]];

corner_r = 4;      // lid screw bosses, outside the corners
corner_off = 3.5;
corners  = [for (x = [-corner_off, inner_x + corner_off], y = [-corner_off, inner_y + corner_off]) [x, y]];

// M5 screw holes for a bracket, from the outside of the header-side wall,
// above the ear and its ribs (14 mm high at the wall)
m5_x     = [inner_x / 2 - 15, inner_x / 2 + 15];
m5_z     = 20;
assert(m5_z + 6 < rk_z, "M5 screw blocks run into the ROCK 4SE");

// FPC: camera connector (on the camera's -x edge) over the TM171, up past
// the ROCK 4SE's uSD end, across the board to its CSI connector.
fpc_p1   = [cam_x - cam_pcb / 2, comp_y, cam_pcb_z + cam_pcb_t + 1];
fpc_up_x = rk_x0 - 3;
fpc_route = (fpc_p1[0] - fpc_up_x) + (rk_top + 2 - fpc_p1[2])
          + abs(rk_x0 + mid(rk_csi[0]) - fpc_up_x) + abs(rk_y0 + mid(rk_csi[1]) - comp_y);
echo(str("FPC routed ", fpc_route, " mm + ", fpc_slack, " slack, have ", fpc_len));
assert(fpc_route + fpc_slack <= fpc_len, "camera FPC too short for this layout");
echo(str("box outside ", inner_x + 2 * wall, " x ", inner_y + 2 * wall, " x ",
         wall_h + lid_t, " mm"));
echo(str("camera axis to TF-Luna emitter/receiver: ", cam_tfl_offset, " mm in x, +-",
         tfl_lens_pitch / 2, " mm in y"));
assert(cam_focus_d / 2 + lens_z * tan(77) < tfl_lens_x - cam_x - tfl_lens_d / 2 - 2,
       "TF-Luna opening runs into the camera aperture");

// ================================================================ helpers
module rrect(size, r) { offset(r = r) offset(delta = -r) square(size); }

module footprint() {
    translate([-wall, -wall]) rrect([inner_x + 2 * wall, inner_y + 2 * wall], 3);
    for (c = corners) translate(c) circle(r = corner_r);
}

// A blind pilot hole going down from z = 0.
module tap_hole(d, depth) { translate([0, 0, -depth]) cylinder(d = d, h = depth + eps); }

module slot(len, d) { hull() for (s = [-1, 1]) translate([s * len / 2, 0]) circle(d = d); }

// ================================================================ base
// Pins for the M2 holes: stand on z = 0, through a part `t` thick.
module stake(t) { cylinder(d = stake_d, h = t + stake_len, $fn = 24); }

// Where each M2 fixing sits: [x, y, z of the part's underside, part thickness].
function m2_points() = concat(
    [for (sx = [-1, 1], sy = [-1, 1])
        [cam_x + sx * cam_hole_pitch / 2, comp_y + sy * cam_hole_pitch / 2, cam_pcb_z, cam_pcb_t]],
    [for (sx = [-1, 1], sy = [-1, 1])
        [tm_c[0] + tm_hole_off + sx * tm_hole_dx / 2, comp_y + sy * tm_hole_dy / 2, floor_t + tmp_h, tm_base_t]],
    [for (s = [-1, 1])
        [tfl_x0 + tfl_ear_off, comp_y + s * tfl_hole_pitch / 2, floor_t + tfl_pad + tfl_ear_z, tfl_ear_t]]);

module base() {
    if (m2_fix == "stake")
        for (p = m2_points()) translate([p[0], p[1], p[2] - eps]) stake(p[3] + eps);
    difference() {
        union() {
            difference() {
                linear_extrude(wall_h) footprint();
                translate([0, 0, floor_t]) cube([inner_x, inner_y, wall_h]);
            }
            ears();
            // TM171 platform and pocket rim
            translate([tmp_x0, comp_y - tmp_w / 2, floor_t - eps])
                cube([tmp_l, tmp_w, tmp_h + eps]);
            translate([tmp_x0, comp_y - tmp_w / 2, floor_t + tmp_h - eps])
                difference() {
                    cube([tmp_l, tmp_w, 3]);
                    translate([rim, rim, -1]) cube([tmp_l - 2 * rim, tmp_w - 2 * rim, 5]);
                }
            // camera bosses, joined in pairs by webs along x
            for (sy = [-1, 1]) {
                for (sx = [-1, 1])
                    translate([cam_x + sx * cam_hole_pitch / 2, comp_y + sy * cam_hole_pitch / 2, floor_t - eps])
                        cylinder(d = 6, h = cam_pcb_z - floor_t + eps);
                translate([cam_x - cam_hole_pitch / 2, comp_y + sy * cam_hole_pitch / 2 - 0.8, floor_t - eps])
                    cube([cam_hole_pitch, 1.6, cam_pcb_z - floor_t - 2]);
            }
            // locating corners: an L outside each PCB corner, 0.3 mm clear, from the floor
            for (sx = [-1, 1], sy = [-1, 1]) translate([cam_x, comp_y, floor_t - eps]) scale([sx, sy, 1]) {
                h = cam_pcb / 2 + 0.3;
                hz = cam_pcb_z + cam_corner_h - floor_t + eps;
                translate([h, h - cam_corner_leg, 0]) cube([cam_corner_t, cam_corner_leg + cam_corner_t, hz]);
                translate([h - cam_corner_leg, h, 0]) cube([cam_corner_leg + cam_corner_t, cam_corner_t, hz]);
            }
            // TF-Luna pad
            translate([tfl_x0 - 1, comp_y - tfl_l / 2 - 1, floor_t - eps])
                cube([tfl_w + 2, tfl_l + 2, tfl_pad + eps]);
            // standoffs up to the ears at the TF-Luna's back, trimmed clear of its body
            for (s = [-1, 1]) difference() {
                translate([tfl_x0 + tfl_ear_off, comp_y + s * tfl_hole_pitch / 2, floor_t + tfl_pad - eps])
                    cylinder(d = 4.5, h = tfl_ear_z + eps);
                tfl_body_clear();
            }
            // ROCK 4SE posts
            for (h = rk_holes) translate([h[0], h[1], floor_t - eps]) cylinder(d = 6, h = rk_z - floor_t + eps);
            // M5 screw blocks on the header-side wall
            for (x = m5_x) translate([x - 6, inner_y - 11, floor_t - eps]) cube([12, 11 + eps, m5_z + 6 - floor_t]);
        }

        // camera aperture: barrel hole, then a cone wider than the FoV
        translate([cam_x, comp_y, -1]) cylinder(d = cam_focus_d, h = floor_t + 2);
        translate([cam_x, comp_y, -eps]) {
            r0 = cam_focus_d / 2;
            cylinder(r1 = r0 + (lens_z + eps) * tan(77), r2 = r0, h = lens_z + 2 * eps);
        }
        // TF-Luna: emitter and receiver openings
        for (s = [-1, 1])
            translate([tfl_lens_x, comp_y + s * tfl_lens_pitch / 2, -1]) cylinder(d = tfl_lens_d + 1, h = floor_t + tfl_pad + 2);
        // M2 pilot holes for the camera, TM171 and TF-Luna (with m2_fix = "screw"),
        // 6 mm deep, or 5.5 into the TM171 platform
        if (m2_fix == "screw")
            for (p = m2_points()) translate([p[0], p[1], p[2]]) tap_hole(tap_m2, p[2] == floor_t + tmp_h ? 5.5 : 6);
        // TM171 pin row relief
        translate([tm_c[0] + tm_l / 2 - tm_pin_in - 2.5, comp_y - 8, floor_t]) cube([5, 16, tmp_h + 1]);
        // ROCK 4SE screws
        for (h = rk_holes) translate([h[0], h[1], rk_z - 8]) cylinder(d = tap_m25, h = 9);
        // lid screws, M3 into the corner posts
        for (c = corners) translate([c[0], c[1], wall_h]) tap_hole(tap_m3, tap_m3_depth);
        // M5 bracket screws, from outside
        for (x = m5_x) translate([x, inner_y + wall + eps, m5_z]) rotate([90, 0, 0]) cylinder(d = tap_m5, h = tap_m5_depth);
        // ports on the y = 0 wall
        // (plug overmoulds are bigger than the sockets: USB-C +5 / +4.5, HDMI +3 / +3)
        translate([rk_x0 + mid(rk_usbc[0]), 0, rk_top + rk_usbc[2] / 2]) port([rk_usbc[0][1] - rk_usbc[0][0] + 5, rk_usbc[2] + 4.5]);
        translate([rk_x0 + mid(rk_hdmi[0]), 0, rk_top + rk_hdmi[2] / 2]) port([rk_hdmi[0][1] - rk_hdmi[0][0] + 3, rk_hdmi[2] + 3]);
        translate([rk_x0 + mid(rk_audio[0]), 0, rk_top + rk_audio[2] / 2]) port([rk_audio[0][1] - rk_audio[0][0] + 2, rk_audio[2] + 2]);
        // RJ45 / USB window on the x = inner_x wall
        translate([inner_x - 1, rk_y0 + 1, rk_top - 1]) cube([wall + 2, rk_w - 2, max(rk_usb_h, rk_rj45[2]) + 1.8]);
        // no vents in the base walls (user, 2026-11-26): all in the lid
        // TM171 x axis, engraved on the header-side wall
        // rotate([90, 0, 180]) maps the text's +x to world -x (readable from
        // outside) and its extrusion to +y; the arrow points to world +x.
        translate([inner_x / 2, inner_y + wall - 0.6, wall_h - 8]) rotate([90, 0, 180])
            linear_extrude(1) {
                text("TM171 +X", size = 5, halign = "center", valign = "center");
                translate([-28, 0]) polygon([[0, -3], [-8, 0], [0, 3]]);
                translate([-28, -0.75]) square([10, 1.5]);
            }
    }
}

// A rectangular cutout through the y = 0 wall, centred at (x, z).
module port(size) {
    translate([-size[0] / 2, -wall - 1, -size[1] / 2]) cube([size[0], wall + rk_y0 + 1.5, size[1]]);
}

// Mounting ears at floor level on both long sides, M6 slots along y.
module ears() {
    for (s = [0, 1]) {
        y0 = s == 0 ? -wall - 20 : inner_y + wall - 1;
        difference() {
            translate([inner_x / 2 - 14, y0, 0]) linear_extrude(5) rrect([28, 21], 3);
            translate([inner_x / 2, s == 0 ? -wall - 11 : inner_y + wall + 10, -1])
                linear_extrude(7) rotate(90) slot(10, slot_m6);
        }
        // ribs up the wall
        for (dx = [-13, 11]) translate([inner_x / 2 + dx, s == 0 ? -wall - 8 : inner_y + wall - 1, 0])
            hull() {
                cube([2, 9, 5]);
                translate([0, s == 0 ? 8 : 0, 0]) cube([2, 1, 14]);
            }
    }
}

// ================================================================ lid
module lid() {
    difference() {
        union() {
            linear_extrude(lid_t) footprint();
            // locating lip on three sides (not over the RJ45 / USB end)
            translate([0, 0, -3]) difference() {
                translate([fit, fit, 0]) cube([inner_x - fit - 1, inner_y - 2 * fit, 3 + eps]);
                translate([fit + 1.6, fit + 1.6, -1]) cube([inner_x, inner_y - 2 * fit - 3.2, 5]);
            }
        }
        for (c = corners) translate([c[0], c[1], -5]) cylinder(d = clear_m3, h = 20);
        // vent slots, all along y, over the whole board (no fan)
        for (x = [10 : 6 : inner_x - 10]) translate([x, 12, -1]) cube([3, inner_y - 24, lid_t + 2]);
    }
}

// ================================================================ tap test coupon
// One row per screw size, pilot holes at -0.1 / nominal / +0.1 mm, 10 mm deep,
// in a boss like the box's (2 mm or more of wall around each hole).
module tap_test() {
    sizes = [tap_m2, tap_m25, tap_m3, tap_m5];
    names = ["M2", "M2.5", "M3", "M5"];
    difference() {
        cube([66, 4 * 14 + 4, 12]);
        for (i = [0 : 3], j = [0 : 2])
            translate([22 + j * 16, 9 + i * 14, 12]) tap_hole(sizes[i] + (j - 1) * 0.1, 10);
        for (i = [0 : 3]) translate([2, 9 + i * 14, 12 - 0.6]) linear_extrude(1)
            text(names[i], size = 3.5, valign = "center");
        for (j = [0 : 2]) translate([22 + j * 16, 1.2, 12 - 0.6]) linear_extrude(1)
            text(["-", "0", "+"][j], size = 3, halign = "center");
    }
}

// ================================================================ dummy parts
module cam() {
    translate([cam_x, comp_y, 0]) {
        difference() {
            translate([-cam_pcb / 2, -cam_pcb / 2, cam_pcb_z]) cube([cam_pcb, cam_pcb, cam_pcb_t]);
            for (sx = [-1, 1], sy = [-1, 1]) translate([sx * cam_hole_pitch / 2, sy * cam_hole_pitch / 2, cam_pcb_z - 1])
                cylinder(d = 2.2, h = 4);
        }
        translate([-cam_holder / 2, -cam_holder / 2, cam_pcb_z - 10]) cube([cam_holder, cam_holder, 10]);
        translate([-2.5, -cam_holder_ear / 2, cam_pcb_z - 3]) cube([5, cam_holder_ear, 3]);
        translate([0, 0, lens_z]) cylinder(d = cam_lens_d, h = cam_pcb_z - 10 - lens_z);
        translate([-cam_pcb / 2, -cam_fpc_w / 2 - 2, cam_pcb_z + cam_pcb_t]) cube([6, cam_fpc_w + 4, cam_back_h]);
    }
}

module tm171() {
    translate([tm_c[0], comp_y, floor_t + tmp_h]) {
        difference() {
            translate([-tm_l / 2, -tm_w / 2, 0]) cube([tm_l, tm_w, tm_base_t]);
            for (sx = [-1, 1], sy = [-1, 1]) translate([tm_hole_off + sx * tm_hole_dx / 2, sy * tm_hole_dy / 2, -1])
                cylinder(d = 2.1, h = 4);
        }
        translate([tm_case_off - tm_case_l / 2, -tm_case_w / 2, 0]) cube([tm_case_l, tm_case_w, tm_h]);
        translate([-tm_l / 2, -tm_usb_y[1], tm_base_t]) cube([7, tm_usb_y[1] - tm_usb_y[0], 3.2]);  // USB-C
        for (i = [-2 : 2]) translate([tm_l / 2 - tm_pin_in, i * 2.54, -2.5])                        // header pins
            cylinder(d = 0.9, h = tm_base_t + 2.5 + 8.5, $fn = 8);
    }
}

// Lens face down on the pad, lens rims in the openings, ears at the back.
module tfluna() {
    z = floor_t + tfl_pad;
    translate([tfl_x0, comp_y - tfl_body_l / 2, z]) cube([tfl_w, tfl_body_l, tfl_body_h]);
    for (s = [-1, 1]) translate([tfl_x0 + tfl_ear_off, comp_y + s * tfl_hole_pitch / 2, z + tfl_ear_z]) difference() {
        hull() {
            cylinder(r = 2.25, h = tfl_ear_t);
            translate([-2.25, -s * 2.5 - 0.5, 0]) cube([4.5, 1, tfl_ear_t]);
        }
        translate([0, 0, -1]) cylinder(d = 2.2, h = tfl_ear_t + 2);
    }
    for (s = [-1, 1]) translate([tfl_lens_x, comp_y + s * tfl_lens_pitch / 2, z - tfl_rim_h])
        cylinder(d = tfl_lens_d, h = tfl_rim_h + eps);
}

// The TF-Luna body's footprint with 0.3 mm clearance, from the pad up.
module tfl_body_clear() {
    translate([tfl_x0 - 0.3, comp_y - tfl_body_l / 2 - 0.3, floor_t + tfl_pad - 1])
        cube([tfl_w + 0.6, tfl_body_l + 0.6, tfl_body_h + 2]);
}

module rk_box(r, h, z) { translate([r[0][0], r[1][0], z]) cube([r[0][1] - r[0][0], r[1][1] - r[1][0], h]); }

module rock4se() {
    translate([rk_x0, rk_y0, rk_z]) {
        difference() {
            cube([rk_l, rk_w, rk_t]);
            for (h = rk_holes) translate([h[0] - rk_x0, h[1] - rk_y0, -1]) cylinder(d = 2.7, h = 4);
        }
        rk_box(rk_rj45, rk_rj45[2], rk_t);
        for (u = rk_usb) rk_box(u, rk_usb_h, rk_t);
        rk_box(rk_usbc, rk_usbc[2], rk_t);
        rk_box(rk_hdmi, rk_hdmi[2], rk_t);
        rk_box(rk_audio, rk_audio[2], rk_t);
        rk_box(rk_csi, rk_csi[2], rk_t);
        rk_box([rk_header_x, [rk_header_y - 2.6, rk_header_y + 2.6]], 8.5, rk_t);     // 40-pin header
        for (u = rk_under_parts)                                                       // underside
            if (u[2] == 1) translate([0, 0, -rk_under]) linear_extrude(rk_under)
                polygon([[mid(u[0]), u[1][0]], [u[0][1], mid(u[1])], [mid(u[0]), u[1][1]], [u[0][0], mid(u[1])]]);
            else rk_box(u, rk_under, -rk_under);
    }
}

module dummies() { cam(); tm171(); tfluna(); rock4se(); }

lid_gap = 0.2;   // drawn gap between base and lid, for the pictures and checks
module lid_placed() { translate([0, 0, wall_h + lid_gap]) lid(); }

// ================================================================ output
module half() { intersection() { children(); translate([-50, comp_y, -50]) cube([300, 100, 200]); } }

module named(n) {
    if (n == "base") base();
    else if (n == "lid") lid_placed();
    else if (n == "cam") cam();
    else if (n == "tm171") tm171();
    else if (n == "tfluna") tfluna();
    else if (n == "rock") rock4se();
}

if (part == "base") base();
else if (part == "lid") translate([0, 0, lid_t]) mirror([0, 0, 1]) lid();   // print top down
else if (part == "tap_test") tap_test();
else if (part == "assembly" || part == "open") {
    color("SteelBlue", part == "open" ? 1 : 0.5) base();
    if (part == "assembly") color("LightSteelBlue", 0.35) lid_placed();
    color("DimGray") cam();
    color("Silver") tm171();
    color("Black") tfluna();
    color("ForestGreen") rock4se();
}
else if (part == "section") {
    // cut along x through the camera axis, seen from -y
    color("SteelBlue") half() base();
    color("LightSteelBlue") half() lid_placed();
    color("DimGray") half() cam();
    color("Silver") half() tm171();
    color("Black") half() tfluna();
    color("ForestGreen") half() rock4se();
    // the camera's 148 deg diagonal FoV, 8 mm deep from the lens front
    color("Orange") half() translate([cam_x, comp_y, lens_z]) mirror([0, 0, 1])
        cylinder(r1 = 0, r2 = 8 * tan(cam_fov_half), h = 8);
}
else if (part == "check") intersection() { named(check_a); translate([0, 0, 0.02]) named(check_b); }
