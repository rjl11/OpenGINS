/*
 * The field-test inputs on core 0: the wheel / ground-speed pulses
 * (pulse_in, plans/wheel-odometry-aiding.md) and the optical-flow link
 * (flow_io, plans/optical-flow-aiding.md). Everything goes to the USB log
 * (usb_log.h); the status report and /status.json show live rates, the
 * quadrature direction and the raw pulses per metre against the GNSS speed.
 * The flow messages also become a ground speed (flow_nav.h) that goes to
 * INSLIB's speed input on core 1 when APP_FLOW_FUSE is set; the wheel pulses
 * are not fused yet.
 */
#ifndef FIELD_IO_H
#define FIELD_IO_H

#include <stdbool.h>
#include <stdint.h>

#include "app_config.h"
#include "nav_core1.h"
#include "settings.h"
#include "um980.h"
#if APP_WEB_BUILT
#include "web/jsonw.h"
#endif

void field_io_init(void);

/* The settings for these inputs: the flow camera on/off and its geometry
 * (height, image yaw, lever arm), the wheel pulses on/off. At boot after
 * field_io_init(), and on every save. */
void field_io_apply_settings(const settings_t *s);

/* Every IMU sample as it goes to core 1 (calibrated gyro, body FRD): the
 * flow's rotation correction. */
void field_io_imu(const nav_imu_t *n);

/* Takes every new edge, event and flow frame (to the log and the
 * counters). Returns true if it did work. */
bool field_io_poll(uint64_t now);

/* Each BESTNAV epoch: the GNSS speed for the raw pulses-per-metre figure. */
void field_io_gnss(const um980_nav_t *g);

/* Once a second, before printing: closes the rate window. */
void field_io_roll(uint64_t now);

void field_io_print_status(void);
#if APP_WEB_BUILT
void field_io_json(jw_t *w);
#endif

#endif /* FIELD_IO_H */
