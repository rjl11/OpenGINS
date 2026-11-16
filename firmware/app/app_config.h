/*
 * Settings of the demo that are not pins: mounting, noise figures, options.
 * Pins and baud rates are in board_pins.h.
 */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* TM210 accelerometer sign (plans/imu-tm210.md, "Decide on the bench").
 * 1: the TM210 reports the gravity vector, FRD = (-ax, ay, az) * g.
 * 0: the TM210 reports specific force,     FRD = (ax, -ay, -az) * g.
 * Check: level and at rest, the status line must show acc z near -9.81.
 * If it shows +9.81, flip this value. */
#define APP_TM210_ACC_IS_GRAVITY    1

/* GNSS antenna position relative to the IMU, body frame FRD [m].
 * Measure on the vehicle. Zero is right for a bench test with the antenna
 * close to the IMU. */
#define APP_GNSS_LEVERARM_X_M       0.0f
#define APP_GNSS_LEVERARM_Y_M       0.0f
#define APP_GNSS_LEVERARM_Z_M       0.0f

/* IMU noise from the TM210 Allan variance run of 2026-11-13/14 (12 h static
 * on the bench, 9.4 h analysed; plans/imu-tm210.md, "INSLIB noise
 * parameters"). Bench values at 21.7-24.6 degC: the gyro bias random walk is
 * only an upper bound (not resolved in 12 h), and outdoor temperature swings
 * may need the two bias random walks raised. */
#define APP_ACC_PSD                 2.1e-6f   /* (m/s^2)^2/Hz */
#define APP_GYR_PSD                 1.0e-7f   /* (rad/s)^2/Hz */
#define APP_GYR_BIAS_RW             7.0e-7f   /* rad/s/sqrt(s) */
#define APP_ACC_BIAS_RW             3.0e-5f   /* m/s^2/sqrt(s) */

/* Stillness variance gate of INSLIB's auto-ZUPT/ZARU detector: the largest
 * gyro sample stddev (per-axis RMS over a 0.2 s window) that still counts as
 * standing still. INSLIB's default of 0.1 deg/s is below the TM210's own
 * noise at 800 Hz: measured 0.27-0.34 deg/s at rest on 2026-11-14 (18.8 degC,
 * the "noise" status line), so the unit never counted as still. 0.5 deg/s
 * leaves 1.5x margin over the worst window. The accel gate stays at
 * INSLIB's 0.15 m/s^2 (measured 0.029-0.034). */
#define APP_STILL_GYR_STDDEV_DPS    0.5f
#define APP_STILL_ACC_STDDEV_MPS2   0.15f  /* INS_DEFAULT_STATIC_ACC_STDDEV_MPS2 */

/* Gyro bias vs temperature, learned at standstills on core 0
 * (plans/imu-auto-calibration.md, Phase 1). Model per axis:
 * b(T) = b0 + k * (T - APP_CALIB_T_REF_C). b0 goes to INSLIB as the fixed
 * gyro bias; core 0 subtracts k * (T - T_ref) before queueing a sample. A
 * learned set is applied at the next boot. Start values, not yet tuned. */
#define APP_CALIB_ENABLE            1
#define APP_CALIB_T_REF_C           25.0f
#define APP_CALIB_CHUNK_MIN_S       10.0f  /* shortest usable standstill chunk */
#define APP_CALIB_CHUNK_MAX_S       60.0f  /* a longer stop gives several chunks */
#define APP_CALIB_CHUNK_MAX_DT_C    0.5f   /* temperature span within a chunk */
/* Priors (zero mean). b0: start value, check against the TM210 datasheet's
 * bias figure. k: the datasheet's +-0.004 deg/s/degC. */
#define APP_CALIB_B0_PRIOR_DPS      1.0f
#define APP_CALIB_K_PRIOR_DPS_C     0.004f
/* k is fitted only once the chunks span this much temperature; below it,
 * k = 0. On the bench (2026-11-15) the y bias drifted 0.01 deg/s over 3 min
 * at a constant 19.1-19.2 degC, which a fit over that span read as a slope. */
#define APP_CALIB_K_MIN_SPAN_C      3.0f
/* Added to each chunk's white-noise variance: bias instability and what the
 * line doesn't model. Chunk-to-chunk scatter on the bench was up to about
 * 0.005 deg/s (3 chunks, 2026-11-15); a first figure, not a measurement. */
#define APP_CALIB_FLOOR_DPS         0.005f
/* At each boot the stored information is scaled by this, so turn-on
 * repeatability and ageing don't freeze the fit. */
#define APP_CALIB_FORGET_PER_BOOT   0.95f
/* A learned set outside these limits is not applied. */
#define APP_CALIB_MAX_B0_DPS        2.0f
#define APP_CALIB_MAX_K_DPS_C       0.02f
/* Earth rate removal: latitude used before any GNSS fix has been seen. An
 * error of 5 deg costs about 6e-6 rad/s. */
#define APP_CALIB_DEFAULT_LAT_DEG   60.0f
/* A quiet window counts as still only if it also looks like rest: mean
 * rate and accelerometer magnitude. Without this, a steady turn (constant
 * rate, constant centripetal acceleration) passes the variance test. */
#define APP_CALIB_STILL_MAX_RATE_DPS 0.5f
#define APP_CALIB_STILL_MAX_DG_MPS2  0.5f  /* | |a| - 9.81 |, raw */

/* Accelerometer from static poses (plans/imu-auto-calibration.md, Phase 2):
 * a = M * (raw - b), M upper triangular, fitted so |a| = local g in every
 * pose. Each still run of at least POSE_MIN_S is one pose, averaged over at
 * most POSE_MAX_S, kept in one of 30 bins by direction. Start values. */
#define APP_CALIB_POSE_MIN_S        3.0f
#define APP_CALIB_POSE_MAX_S        30.0f
/* Added to each pose's white-noise sigma: bias settling and temperature
 * (the TM210's x accelerometer settled by 25 mm/s^2 over hours,
 * plans/imu-tm210.md). Estimate. */
#define APP_CALIB_ACC_FLOOR_MPS2    0.01f
/* Priors (identity, zero bias). The TM210's direction is accurate and its
 * magnitude isn't, so scale gets the widest prior. */
#define APP_CALIB_ACC_SD_SCALE      0.02f
#define APP_CALIB_ACC_SD_MIS_DEG    0.5f
#define APP_CALIB_ACC_SD_BIAS_MPS2  0.1f
/* A learned set outside these limits is not applied. */
#define APP_CALIB_ACC_MAX_SCALE_DEV 0.05f
#define APP_CALIB_ACC_MAX_MIS_DEG   2.0f
#define APP_CALIB_ACC_MAX_BIAS_MPS2 0.5f

/* Magnetometer (plans/imu-auto-calibration.md, Phase 3). Start values. */
/* Samples: a group of GROUP consecutive magnetometer samples (75 Hz) while
 * the unit turns slower than MAX_RATE is one sample; it carries the down
 * direction when the accelerometer magnitude stays within DOWN_MAX_DG of g. */
#define APP_CALIB_MAG_GROUP              8
#define APP_CALIB_MAG_MAX_RATE_DPS       30.0f
#define APP_CALIB_MAG_DOWN_MAX_DG_MPS2   0.3f
#define APP_CALIB_MAG_MIN_SAMPLES        12      /* bins before an ellipsoid fit */
#define APP_CALIB_MAG_OUTLIER_REL        0.15f   /* | |m| - F | / F to drop a sample */
#define APP_CALIB_MAG_SOLVE_PERIOD_US    2000000u
/* Fit: sigmas of one sample's magnitude and vertical component, and the
 * priors (M diagonal around 1, the raw unit; off-diagonal; hard iron). */
#define APP_CALIB_MAG_SD_UT              0.5f
#define APP_CALIB_MAG_DIP_SD_UT          1.0f
#define APP_CALIB_MAG_SD_DIAG_REL        0.3f
#define APP_CALIB_MAG_SD_OFFDIAG         0.1f
#define APP_CALIB_MAG_SD_HARD_UT         100.0f
/* Reference fit: sigma of one reference sample (attitude error x F), the
 * prior around the ellipsoid result (0.05 allows about 3 deg of rotation),
 * and when a sample is taken: INS in FULL for REF_FULL_S, one per
 * REF_PERIOD, turning slower than REF_MAX_RATE; with the course as heading
 * at REF_MIN_SPEED or more, and on a fixed wing only in straight flight. */
#define APP_CALIB_MAG_REF_SD_UT          1.5f
#define APP_CALIB_MAG_REF_SD_M           0.05f
#define APP_CALIB_MAG_REF_SD_B_UT        5.0f
#define APP_CALIB_MAG_REF_FULL_S         60
#define APP_CALIB_MAG_REF_PERIOD_US      1000000u
#define APP_CALIB_MAG_REF_MAX_RATE_DPS   5.0f
#define APP_CALIB_MAG_REF_MIN_SPEED_MPS  3.0f
#define APP_CALIB_MAG_REF_FW_MAX_YAW_DPS 2.0f
/* A learned set outside these is not applied. */
#define APP_CALIB_MAG_MIN_DIAG           0.5f
#define APP_CALIB_MAG_MAX_DIAG           2.0f
#define APP_CALIB_MAG_MAX_OFFDIAG        0.3f    /* relative to the mean diagonal */
#define APP_CALIB_MAG_MAX_HARD_UT        200.0f
#define APP_CALIB_MAG_MAX_RMS_REL        0.05f
/* Magnetometer data counts towards the save rule as one unit per this many
 * groups / reference samples. */
#define APP_CALIB_MAG_GROUPS_PER_UNSAVED 20
#define APP_CALIB_MAG_REFS_PER_UNSAVED   60

/* Gyro scale and misalignment (plans/imu-auto-calibration.md, Phase 4).
 * Rotations between still poses (at least POSE_MIN_S each): turned by at
 * least ROT_MIN_ANGLE within ROT_MAX_S; SD is the sigma per component of the
 * gravity-direction residual. Turns with a GNSS heading before and after:
 * at least TURN_MIN_ANGLE within TURN_MAX_S, the ends where the yaw rate is
 * below TURN_STRAIGHT; TURN_SD_FLOOR is added to the two heading sigmas.
 * Priors around the matrix in use; an element changes at the next boot
 * only once its posterior sigma is below ACCEPT_SD. Start values. */
#define APP_CALIB_ROT_MIN_ANGLE_DEG      30.0f
#define APP_CALIB_ROT_MAX_S              20.0f
#define APP_CALIB_ROT_SD_RAD             0.003f
#define APP_CALIB_TURN_MIN_ANGLE_DEG     90.0f
#define APP_CALIB_TURN_MAX_S             90.0f
#define APP_CALIB_TURN_STRAIGHT_DPS      2.0f
#define APP_CALIB_TURN_SD_FLOOR_RAD      0.002f
#define APP_CALIB_GYR_SD_SCALE           0.02f
#define APP_CALIB_GYR_SD_MIS_DEG         0.5f
#define APP_CALIB_GYR_ACCEPT_SD_SCALE    0.001f
#define APP_CALIB_GYR_ACCEPT_SD_MIS_DEG  0.05f
#define APP_CALIB_GYR_MAX_SCALE_DEV      0.03f
#define APP_CALIB_GYR_MAX_MIS_DEG        2.0f

/* Saving: only at standstill, after this many new chunks or poses, at most
 * this often. */
#define APP_CALIB_SAVE_MIN_CHUNKS   3
#define APP_CALIB_SAVE_MIN_S        600

/* 0 = INSLIB default (2 m/s). Below this speed the GNSS course is not used
 * for yaw (plans/inslib-nav-suite.md). */
#define APP_AUTOMOTIVE_MIN_SPEED    0.0f

/* Longest dead reckoning after the last GNSS position, in seconds: the
 * default of the web page's setting (max_deadreckoning_s). INSLIB
 * coasts on the IMU (mode COASTING) for this long, then stops giving a
 * position (ATTITUDE_ONLY) until the next fix. Only a position resets it:
 * speed aiding (flow, wheel) slows the drift but doesn't extend it.
 * 0 = INSLIB's default (10 s). 60 s is a test value: the TM210's drift
 * over that time has not been measured yet. */
#define APP_MAX_DEADRECKONING_SEC   60.0f

/* The TM210 magnetometer is fed to INSLIB (the AHRS and the INS). INSLIB
 * applies its own World Magnetic Model for the declination and the expected
 * field strength once core 1 has a GNSS position, and gates samples whose
 * field strength is off (a steel tractor). 0 leaves the AHRS off and yaw
 * comes from the INS (GNSS course or dual-antenna heading) or the ARS.
 * This is the default of the web page's magnetometer switch (sensor_mag). */
#define APP_FEED_MAG                1

/* Estimate the magnetometer hard-iron bias in the INS (18 error states).
 * It converges while the vehicle turns. The on-vehicle calibration
 * (inslib_mag_calib.py, plans/imu-tm210.md) will add fixed soft- and
 * hard-iron terms later. */
#define APP_ESTIMATE_MAG_BIAS       1

/* GNSS antenna mode (plans/gnss-um98x-config.md, "Antenna mode").
 * Single: UM980, or UM982 with the slave antenna off; yaw from the GNSS
 * course over ground. Dual: UM982 with both antennas; yaw from its heading.
 * Dual with a UM980 fitted falls back to single. Set it at build time with
 * cmake -DGNSS_ANTENNA_MODE=dual. The web page will override it from flash
 * once it exists; until then this default is the setting. */
#define APP_ANTENNA_SINGLE          0
#define APP_ANTENNA_DUAL            1
#ifndef APP_GNSS_ANTENNA_MODE
#define APP_GNSS_ANTENNA_MODE       APP_ANTENNA_SINGLE
#endif

/* Platform (plans/imu-auto-calibration.md, "Gates per platform"). It decides
 * when the GNSS course over ground counts as a heading, for the magnetometer
 * reference fit: tractor (driving forward above the course speed) and
 * fixed-wing (straight flight) yes, survey pole and multirotor never. A
 * dual-antenna heading counts on every platform. Set it at build time with
 * cmake -DPLATFORM=tractor|pole|multirotor|fixed_wing; the web page can
 * override it, applied at reboot. */
#define APP_PLATFORM_TRACTOR        0
#define APP_PLATFORM_POLE           1
#define APP_PLATFORM_MULTIROTOR     2
#define APP_PLATFORM_FIXED_WING     3
#ifndef APP_PLATFORM
#define APP_PLATFORM                APP_PLATFORM_POLE
#endif

/* Tractor and survey pole only: height of the GNSS antenna (phase centre)
 * above the ground, along the body down axis [m]. Default of the web page's
 * setting (ant_height_m). Above 0, the output position (best_position() in
 * main.c) is the point on the ground below the antenna, tilted with the INS
 * roll and pitch, instead of the antenna. 0 = the antenna position. Ignored
 * on the UAV platforms. */
#define APP_ANT_HEIGHT_M            2.0f

/* The ground point follows the tilt only while the filter's own (INS)
 * attitude is good: the 1-sigma of the body down axis's direction
 * (ground_axis_sd(): roll and pitch, plus yaw times sin(tilt)) at most this
 * [deg]. Otherwise, and whenever the attitude is not the INS's, the point
 * is taken straight down, as for a levelled pole. 2 deg is the threshold
 * the Leica GS18 T uses (docs/survey-pole/TS04E_luo_schaufler_et_al_9407.pdf,
 * section 3.2); at 2 m it is about 7 cm of tip error, 1-sigma. */
#define APP_GROUND_TILT_MAX_SD_DEG  2.0f

/* $PAOGI (app/paogi.h) sends the INS heading only while the INS's own yaw
 * 1-sigma is at most this [deg]; the field stays empty otherwise, and
 * AgOpenGPS falls back to its fix-to-fix heading. AgOpenGPS takes a $PAOGI
 * heading as a dual-antenna heading with no fusion of its own, and with a
 * single antenna the yaw is not observable until the vehicle has driven
 * (bench 2026-11-16: it drifted 4 deg in a minute standing still in FULL).
 * 1 deg is about 5 cm of offset 3 m ahead; an estimate, not yet measured. */
#define APP_PAOGI_HEADING_MAX_SD_DEG 1.0f

/* Survey-pole aiding (app/pole_aid.h, plans/pole-tilt-aiding.md): on the
 * pole platform with an antenna height, from the natural sway of a pole
 * standing on a point. All estimates, not yet measured; the other
 * thresholds are in pole_aid_default_cfg().
 *  TIP_ERR_MAX       planted while the specific force moved to the tip
 *                    stays this close to g, mean over 0.25 s [m/s^2]
 *  TIP_SPEED_MAX     and the INS's own tip speed below this [m/s]
 *  TIP_SD_M          1-sigma of the tip-stays-put constraint, per axis [m]
 *  FIT_GNSS_SD_MAX_M fixes this good (RTK) count for the yaw fit [m] */
#define APP_POLE_TIP_ERR_MAX        0.25f
#define APP_POLE_TIP_SPEED_MAX      0.2f
#define APP_POLE_TIP_SD_M           0.01f
#define APP_POLE_FIT_GNSS_SD_MAX_M  0.03f

/* Pole axis in the IMU frame (app/pole_mount.h, plans/pole-tilt-aiding.md,
 * item 5), learned from the planted episodes. All estimates, not yet
 * measured.
 *  TILT_PRIOR_DEG  prior sigma of the IMU's tilt on the pole
 *  LEN_PRIOR_M     prior sigma of the pole length about the antenna height
 *  FIX_SD_M        1-sigma of one horizontal RTK fix component
 *  CORR            variance inflation: 20 Hz RTK errors are correlated over
 *                  about half a second, so 10 fixes count as one
 *  EP_RMS_MAX_M    an episode fitting worse than this is dropped; once 5
 *  EP_RMS_REL      are in, also one worse than this times the overall RMS
 *  SD_MAX_DEG      the learned axis is used when its tilt sigma is below
 *                  this (7 mm at 2 m) ...
 *  TILT_MAX_DEG    ... the tilt below this (else: check the mounting) ...
 *  LEN_TOL_M       ... and the length within this of the antenna height
 *  SAVE_S          new episodes are saved to flash at most this often */
#define APP_POLE_MOUNT_TILT_PRIOR_DEG 3.0f
#define APP_POLE_MOUNT_LEN_PRIOR_M    0.05f
#define APP_POLE_MOUNT_FIX_SD_M       0.015f
#define APP_POLE_MOUNT_CORR           10.0f
#define APP_POLE_MOUNT_EP_RMS_MAX_M   0.02f
#define APP_POLE_MOUNT_EP_RMS_REL     1.5f
#define APP_POLE_MOUNT_SD_MAX_DEG     0.2f
#define APP_POLE_MOUNT_TILT_MAX_DEG   5.0f
#define APP_POLE_MOUNT_LEN_TOL_M      0.3f
#define APP_POLE_MOUNT_SAVE_S         300u

/* Dual antenna only. Direction of the ANT1 -> ANT2 baseline in the body
 * frame, clockwise from the body x-axis (forward) [deg]. Across the cab,
 * left to right, is +90. Measure on the vehicle. */
#define APP_BASELINE_YAW_DEG        90.0f

/* Dual antenna only. A heading epoch is dropped when its baseline length
 * differs from the running median by more than this (start value, tune on
 * the bench). */
#define APP_BASELINE_GATE_M         0.05f

/* Dual antenna only. Floor on the heading sigma, for a 1 m baseline; it
 * scales with 1 / length (datasheet: 0.1 deg RMS at 1 m). */
#define APP_HEADING_SD_FLOOR_DEG_1M 0.1f

/* A barometer sample older than this when it reaches the filter is dropped
 * (start value, not yet measured). */
#define APP_BARO_MAX_AGE_US         100000

/* When Wi-Fi and the web server run (plans/web-config-server.md, "When
 * Wi-Fi runs"). Set at build time with cmake -DWEB_MODE=always|switch|off.
 * switch: GPIO22 (PIN_WEB_MODE_SWITCH, internal pull-up) is read once at
 * boot; pulled to GND = Wi-Fi and the web server on, open = off for this
 * boot. The station SSID and password come from cmake too (WIFI_STA_SSID,
 * WIFI_STA_PASSWORD; an empty password means an open network). */
#define APP_WEB_OFF                 0
#define APP_WEB_ALWAYS              1
#define APP_WEB_SWITCH              2
#ifndef APP_WEB_MODE
#define APP_WEB_MODE                APP_WEB_OFF
#endif
/* The web code is linked in (always or switch); whether it runs this boot is
 * decided at runtime in switch mode. */
#define APP_WEB_BUILT               (APP_WEB_MODE != APP_WEB_OFF)
#ifndef APP_WIFI_STA_SSID
#define APP_WIFI_STA_SSID           "MyWiFi"
#endif
#ifndef APP_WIFI_STA_PASSWORD
#define APP_WIFI_STA_PASSWORD       ""
#endif
/* Network mode (cmake -DWIFI_MODE=auto|sta|ap). auto: join the station
 * network; if that hasn't worked within APP_WIFI_AP_FALLBACK_S of boot, run
 * the open AP instead until the next reboot. sta: station only. ap: open AP
 * from boot. The AP is open (no password) by decision, at 192.168.4.1. */
#define APP_WIFI_AUTO               0
#define APP_WIFI_STA                1
#define APP_WIFI_AP                 2
#ifndef APP_WIFI_MODE
#define APP_WIFI_MODE               APP_WIFI_AUTO
#endif
#define APP_WIFI_AP_SSID            "OpenGINS"
#define APP_WIFI_AP_FALLBACK_S      30

/* DHCP and mDNS name: http://opengins.local/ where mDNS resolves. */
#define APP_WEB_HOSTNAME            "opengins"

/* Optical-flow camera trigger (plans/optical-flow-aiding.md, drivers/flow_io).
 * The PWM runs from boot whether a camera is wired or not; one pulse in
 * FLOW_LINK_GAP_PERIOD is left out for the Pi's frame matching. 0 Hz = off. */
#define APP_FLOW_TRIGGER_HZ         100u
#define APP_FLOW_TRIGGER_US         100u

/* Sensors fitted: the defaults of the web page's on/off switches (settings.h,
 * sensor_*). Not every vehicle has every sensor. A sensor that is off is not
 * started or polled and nothing of it reaches INSLIB. The magnetometer's
 * default is APP_FEED_MAG above. */
#define APP_SENSOR_BARO             1
#define APP_SENSOR_FLOW             1
#define APP_SENSOR_WHEEL            1

/* Flow into INSLIB (app/flow_nav.h): the Pi's flow becomes a ground speed on
 * core 0 and goes to INSLIB's speed input while the flow sensor is on. The
 * height, image yaw and lever arm below are the defaults of the web page's
 * settings; the rest is compile-time.
 *  HEIGHT_M        lens above level ground [m]
 *  HEIGHT_REL_SD   1-sigma of that height as a fraction (crop, sinkage, bounce)
 *  CAM_YAW_DEG     image top from body forward, clockwise seen from above
 *  LEVERARM_*_M    camera from the IMU, body FRD [m]
 *  EXPOSURE_DELAY  trigger edge to exposure start when no strobe is wired [us]
 *                  (opto-coupler and sensor; not measured, 0 until it is)
 *  STROBE_WINDOW   a strobe edge this soon after a trigger belongs to it [us]
 *  FUSE_PERIOD     frames averaged into one speed sample [s] (10 Hz: the
 *                  error is a slowly changing scale, not white noise)
 *  MIN_SD / MAX    floor of the 1-sigma, and the fastest speed believed [m/s]
 *  MAX_TILT_DEG    steeper roll or pitch: the ground is not under the lens
 *  MIN_FEATURES    fewer tracked features: the frame is not used */
#define APP_FLOW_HEIGHT_M           1.0f
#define APP_FLOW_HEIGHT_REL_SD      0.05f
#define APP_FLOW_CAM_YAW_DEG        0.0f
#define APP_FLOW_LEVERARM_X_M       0.0f
#define APP_FLOW_LEVERARM_Y_M       0.0f
#define APP_FLOW_LEVERARM_Z_M       0.0f
#define APP_FLOW_EXPOSURE_DELAY_US  0u
#define APP_FLOW_STROBE_WINDOW_US   5000u
#define APP_FLOW_FUSE_PERIOD_S      0.1f
#define APP_FLOW_MIN_SD_MPS         0.05f
#define APP_FLOW_MAX_MPS            20.0f
#define APP_FLOW_MAX_TILT_DEG       30.0f
#define APP_FLOW_MIN_FEATURES       15u

/* Wheel / ground-speed pulses (plans/wheel-odometry-aiding.md). The internal
 * pull-up suits an open-collector output (opto stage, Hall sensor); the
 * raw pulses-per-metre figure in the status report needs at least this GNSS
 * speed. ISO 11786 nominal: 130 pulses per metre. */
#define APP_SPEED_PULL_UP           1
#define APP_SPEED_PPM_MIN_MPS       1.0f
#define APP_SPEED_ISO11786_PPM      130.0f

/* Core 1 stack. nav_suite_update() needs about 13 KB worst case. */
#define APP_CORE1_STACK_BYTES       (32 * 1024)

#endif /* APP_CONFIG_H */
