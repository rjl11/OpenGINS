/*
 * USB log mode: every sensor sample, the INSLIB output, the wheel-speed
 * edges and the optical-flow link, streamed as framed binary records on the
 * USB CDC port for field tests (plans/wheel-odometry-aiding.md, "Field
 * testing"). firmware/tools/field_log.py records and decodes it.
 *
 * Off at boot. A line "log on" sent to the USB port turns it on, "log off"
 * turns it off; it also goes off when the host drops DTR for 2 s. While it is
 * on, everything printf() writes (the 1 s status report, driver messages)
 * goes into TEXT records in the same stream, so nothing interleaves with the
 * binary data.
 *
 * Record frame (little-endian):
 *   0  u8   sync0  USB_LOG_SYNC0 (0xA7)
 *   1  u8   sync1  USB_LOG_SYNC1 (0x4C)
 *   2  u8   type   USB_LOG_*
 *   3  u8   len    payload bytes N
 *   4  N    payload
 *   4+N u16 crc    CRC-16/CCITT-FALSE over bytes 2 .. 3+N (flow_link_crc16)
 * Payloads are listed at the USB_LOG_* types below; field_log.py has the
 * same layouts. Times are time_us_64() on the Pico.
 *
 * GNSS time: time_us_64() restarts at 0 when the Pico reboots, so each
 * GNSS record carries both clocks (t_valid on the Pico clock from PPS, and
 * week / tow_ms), and once a second, on the PPS-tagged epoch at a whole
 * second, a TIME record gives the same pair with the leap seconds (GPS - UTC).
 * field_log.py decode maps every Pico time to GPS time from these, per log
 * session (each START), so logs from before and after a reboot line up.
 *
 * The records go into a RAM ring and leave through the CDC port as fast as
 * the host takes them; when the ring is full, records are dropped and a
 * DROPPED record says how many bytes were lost.
 *
 * Core 0 only.
 */
#ifndef USB_LOG_H
#define USB_LOG_H

#include <stdbool.h>
#include <stdint.h>

#include "flow_io.h"
#include "nav_core1.h"
#include "pulse_in.h"
#include "um980.h"

#define USB_LOG_SYNC0   0xA7u
#define USB_LOG_SYNC1   0x4Cu
#define USB_LOG_VERSION 2u /* 2: TIME record */

enum {
    USB_LOG_START       = 1,  /* u8 version, u64 t_us, u32 sys_hz, char name[]                     */
    USB_LOG_TEXT        = 2,  /* printf output                                                     */
    USB_LOG_IMU         = 3,  /* u64 t, f32 dt, f32 gyr[3] (as sent to INSLIB: learned correction
                                 applied), f32 acc[3], f32 mag[3], u8 flags (1 mag_new, 2 gap)   */
    USB_LOG_GNSS        = 4,  /* BESTNAV, see usb_log.c                                            */
    USB_LOG_HEADING     = 5,  /* UNIHEADING, see usb_log.c                                         */
    USB_LOG_BARO        = 6,  /* u64 t, f32 pressure_pa, f32 temperature_c                         */
    USB_LOG_NAV         = 7,  /* INSLIB output at 10 Hz, see usb_log.c                             */
    USB_LOG_SPEED_EDGES = 8,  /* u64 t_base, then (u32 dt_us, u8 levels) per edge (pulse_in.h)     */
    USB_LOG_FLOW_EVENT  = 9,  /* u64 t, u32 seq, u8 kind (FLOW_EV_*), u8 level                     */
    USB_LOG_FLOW_FRAME  = 10, /* u64 t_arrival, u8 msg type, then the message payload              */
    USB_LOG_DROPPED     = 11, /* u32 bytes, u32 records lost since the last DROPPED                */
    USB_LOG_TIME        = 12, /* u64 t (the PPS edge), u16 week, u32 tow_ms, u8 leap_s, see below    */
};

typedef struct {
    bool     on;
    uint32_t bytes_per_s;   /* sent to the host, last full second */
    uint32_t dropped_bytes; /* since log on                        */
    uint32_t dropped_recs;
    uint32_t ring_max;      /* highest ring fill since log on [bytes] */
} usb_log_status_t;

void usb_log_init(void);

/* Reads commands from the host, sends what the ring holds, flushes old
 * speed edges. Call on every main-loop pass. Returns true if it did work. */
bool usb_log_poll(uint64_t now);

bool usb_log_on(void);
void usb_log_status(usb_log_status_t *out);

/* Records. Each does nothing while the log is off. */
void usb_log_imu(const nav_imu_t *n, bool gap);
void usb_log_gnss(const um980_nav_t *g);
void usb_log_heading(const um980_heading_t *h);
void usb_log_baro(uint64_t t_us, float pressure_pa, float temperature_c);
void usb_log_nav(const nav_output_t *o);
void usb_log_speed_edge(const pulse_edge_t *e);
void usb_log_flow_event(const flow_event_t *e);
void usb_log_flow_frame(const flow_frame_t *f);

#endif /* USB_LOG_H */
