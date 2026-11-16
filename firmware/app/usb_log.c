#include "usb_log.h"

#include <string.h>

#include "hardware/clocks.h"
#include "pico/stdio.h"
#include "pico/stdio/driver.h"
#include "pico/stdio_usb.h"
#include "pico/time.h"
#include "tusb.h"

#include "flow_link.h"
#include "paogi.h"

#define RING_BITS        15 /* 32 KB: about 0.6 s of the full stream */
#define RING_SIZE        (1u << RING_BITS)
#define EDGE_MAX         48u      /* per SPEED_EDGES record: 8 + 48 * 5 = 248 */
#define EDGE_HOLD_US     50000u   /* a batch is sent at the latest this old  */
#define OFF_AFTER_US     2000000u /* DTR down this long: log off             */
#define SEND_MIN_BYTES   1024u    /* send in pieces of at least this ...      */
#define SEND_MAX_WAIT_US 5000u    /* ... or when the oldest byte waited this  */

static uint8_t s_ring[RING_SIZE];
static uint32_t s_wr, s_rd; /* free-running */

static bool s_on;
static bool s_host; /* DTR up: a host is reading; records are kept only then */
static uint32_t s_drop_bytes, s_drop_recs;     /* since the last DROPPED record */
static uint32_t s_drop_total, s_drop_recs_total;
static uint32_t s_ring_max;
static uint32_t s_sent, s_sent_prev, s_bps;
static uint64_t s_next_sec, s_disc_since, s_last_send;

static char s_cmd[32];
static unsigned s_cmd_n;

/* SPEED_EDGES batch being filled. */
static uint8_t s_edges[8u + EDGE_MAX * 5u];
static unsigned s_edge_n;
static uint64_t s_edge_t0;

/* ---- ring and frames ---------------------------------------------------- */

static uint32_t ring_used(void) { return s_wr - s_rd; }

static void ring_put(const uint8_t *p, uint32_t n)
{
    uint32_t off = s_wr & (RING_SIZE - 1u);
    uint32_t first = RING_SIZE - off < n ? RING_SIZE - off : n;
    memcpy(&s_ring[off], p, first);
    memcpy(s_ring, p + first, n - first);
    s_wr += n;
    if (ring_used() > s_ring_max) {
        s_ring_max = ring_used();
    }
}

/* CRC-16/CCITT-FALSE by table: the same value as flow_link_crc16(), about
 * eight times faster, for some 1000 records a second. */
static uint16_t s_crc_tab[256];

static void crc_init(void)
{
    for (unsigned i = 0; i < 256u; i++) {
        uint16_t c = (uint16_t)(i << 8);
        for (int k = 0; k < 8; k++) {
            c = (c & 0x8000u) ? (uint16_t)((c << 1) ^ 0x1021u) : (uint16_t)(c << 1);
        }
        s_crc_tab[i] = c;
    }
}

static uint16_t crc_update(uint16_t crc, const uint8_t *p, uint32_t n)
{
    while (n--) {
        crc = (uint16_t)((crc << 8) ^ s_crc_tab[(uint8_t)((crc >> 8) ^ *p++)]);
    }
    return crc;
}

static bool put_frame(uint8_t type, const uint8_t *payload, uint32_t len)
{
    uint8_t hdr[4] = { USB_LOG_SYNC0, USB_LOG_SYNC1, type, (uint8_t)len };
    uint32_t total = 4u + len + 2u;
    if (RING_SIZE - ring_used() < total) {
        s_drop_bytes += total;
        s_drop_recs++;
        s_drop_total += total;
        s_drop_recs_total++;
        return false;
    }
    /* CRC over type, len and payload. */
    uint16_t crc = crc_update(crc_update(0xFFFFu, &hdr[2], 2u), payload, len);
    uint8_t c[2] = { (uint8_t)crc, (uint8_t)(crc >> 8) };
    ring_put(hdr, 4);
    ring_put(payload, len);
    ring_put(c, 2);
    return true;
}

static void record(uint8_t type, const uint8_t *payload, uint32_t len)
{
    if (!s_on || !s_host) {
        return; /* no host: nothing to keep (and no drops to count) */
    }
    if (s_drop_recs > 0) {
        /* Records were lost: say so first, once there is room for both. */
        if (RING_SIZE - ring_used() < (4u + 8u + 2u) + (4u + len + 2u)) {
            s_drop_bytes += 4u + len + 2u;
            s_drop_recs++;
            s_drop_total += 4u + len + 2u;
            s_drop_recs_total++;
            return;
        }
        uint8_t d[8];
        memcpy(&d[0], &s_drop_bytes, 4);
        memcpy(&d[4], &s_drop_recs, 4);
        s_drop_bytes = 0;
        s_drop_recs = 0;
        put_frame(USB_LOG_DROPPED, d, 8);
    }
    put_frame(type, payload, len);
}

/* Little-endian packer (the RP2350 is little-endian, so memcpy). */
typedef struct {
    uint8_t  b[255];
    uint32_t n;
} pk_t;
#define PK(p, v) do { __typeof__(v) _v = (v); memcpy(&(p)->b[(p)->n], &_v, sizeof _v); (p)->n += sizeof _v; } while (0)

/* ---- text: printf while the log is on ----------------------------------- */

static void text_out(const char *buf, int len)
{
    while (len > 0) {
        int n = len > 250 ? 250 : len;
        record(USB_LOG_TEXT, (const uint8_t *)buf, (uint32_t)n);
        buf += n;
        len -= n;
    }
}

static stdio_driver_t s_text_driver = {
    .out_chars = text_out,
#if PICO_STDIO_ENABLE_CRLF_SUPPORT
    .crlf_enabled = false,
#endif
};

/* ---- on / off ----------------------------------------------------------- */

static void edges_flush(void);

static void log_set(bool on)
{
    if (on == s_on) {
        return;
    }
    if (on) {
        s_wr = s_rd = 0;
        s_drop_bytes = s_drop_recs = s_drop_total = s_drop_recs_total = 0;
        s_ring_max = 0;
        s_edge_n = 0;
        s_host = true;
        /* The CDC FIFO can still hold the end of an earlier session that
         * the host closed before reading; it would come out first. */
        tud_cdc_write_clear();
        s_on = true;
        pk_t p = { .n = 0 };
        PK(&p, (uint8_t)USB_LOG_VERSION);
        PK(&p, time_us_64());
        PK(&p, (uint32_t)clock_get_hz(clk_sys));
        static const char k_name[] = "OpenGINS";
        memcpy(&p.b[p.n], k_name, sizeof k_name - 1);
        p.n += sizeof k_name - 1;
        record(USB_LOG_START, p.b, p.n);
        /* printf now goes into TEXT records. */
        stdio_set_driver_enabled(&s_text_driver, true);
        stdio_set_driver_enabled(&stdio_usb, false);
        printf("[log] on\n");
    } else {
        stdio_set_driver_enabled(&s_text_driver, false);
        s_on = false;
        s_wr = s_rd = 0; /* what was not sent yet is dropped */
        stdio_set_driver_enabled(&stdio_usb, true);
        printf("[log] off (%lu bytes dropped in %lu records)\n", (unsigned long)s_drop_total,
               (unsigned long)s_drop_recs_total);
    }
}

static void command(const char *c)
{
    if (strcmp(c, "log on") == 0) {
        log_set(true);
    } else if (strcmp(c, "log off") == 0) {
        log_set(false);
    } else if (strcmp(c, "paogi on") == 0 || strcmp(c, "paogi off") == 0) {
        /* $PAOGI on the tractor platform (paogi.h, main.c). */
        paogi_set_enabled(c[7] == 'n');
        printf("[paogi] %s\n", paogi_enabled() ? "on (sent on the tractor platform only)" : "off");
    } else if (c[0] != '\0') {
        printf("[log] unknown command '%s' (log on | log off | paogi on | paogi off)\n", c);
    }
}

static bool read_commands(void)
{
    char buf[16];
    bool any = false;
    for (;;) {
        /* Straight from the USB driver: it is taken out of stdio while the
         * log is on, but still reads. */
        int n = stdio_usb.in_chars(buf, sizeof buf);
        if (n <= 0) {
            return any;
        }
        any = true;
        for (int i = 0; i < n; i++) {
            char ch = buf[i];
            if (ch == '\r' || ch == '\n') {
                s_cmd[s_cmd_n] = '\0';
                command(s_cmd);
                s_cmd_n = 0;
            } else if (s_cmd_n < sizeof s_cmd - 1u) {
                s_cmd[s_cmd_n++] = ch;
            }
        }
    }
}

/* ---- sending ------------------------------------------------------------ */

/* Each CDC write costs a mutex, a FIFO copy and a USB transfer set-up, so
 * the ring goes out in pieces, not record by record. */
static bool send(uint64_t now)
{
    if (!s_on || ring_used() == 0) {
        s_last_send = now;
        return false;
    }
    if (ring_used() < SEND_MIN_BYTES && now - s_last_send < SEND_MAX_WAIT_US) {
        return false;
    }
    s_last_send = now;
    if (!stdio_usb_connected()) {
        return false;
    }
    bool any = false;
    for (;;) {
        uint32_t used = ring_used();
        uint32_t avail = tud_cdc_write_available();
        if (used == 0 || avail == 0) {
            return any;
        }
        uint32_t off = s_rd & (RING_SIZE - 1u);
        uint32_t n = RING_SIZE - off; /* contiguous */
        if (n > used) n = used;
        if (n > avail) n = avail;
        /* At most what the CDC FIFO takes now, so this never waits. */
        stdio_usb.out_chars((const char *)&s_ring[off], (int)n);
        s_rd += n;
        s_sent += n;
        any = true;
    }
}

void usb_log_init(void)
{
    crc_init();
    s_next_sec = time_us_64() + 1000000u;
}

bool usb_log_poll(uint64_t now)
{
    /* Commands are rare; each read takes the USB driver's mutex. */
    static uint64_t s_next_cmd;
    bool worked = false;
    if (now >= s_next_cmd) {
        s_next_cmd = now + 20000u;
        worked = read_commands();
    }
    if (s_on) {
        s_host = stdio_usb_connected();
        if (s_host) {
            s_disc_since = 0;
        } else if (s_disc_since == 0) {
            s_disc_since = now;
        } else if (now - s_disc_since > OFF_AFTER_US) {
            log_set(false);
            return true;
        }
        if (s_edge_n > 0 && now - s_edge_t0 > EDGE_HOLD_US) {
            edges_flush();
        }
        worked |= send(now);
    }
    if (now >= s_next_sec) {
        s_bps = s_sent - s_sent_prev;
        s_sent_prev = s_sent;
        s_next_sec += 1000000u;
    }
    return worked;
}

bool usb_log_on(void) { return s_on; }

void usb_log_status(usb_log_status_t *out)
{
    out->on = s_on;
    out->bytes_per_s = s_bps;
    out->dropped_bytes = s_drop_total;
    out->dropped_recs = s_drop_recs_total;
    out->ring_max = s_ring_max;
}

/* ---- records ------------------------------------------------------------ */

void usb_log_imu(const nav_imu_t *n, bool gap)
{
    if (!s_on) return;
    pk_t p = { .n = 0 };
    PK(&p, n->t_us);
    PK(&p, n->dt_s);
    for (int i = 0; i < 3; i++) PK(&p, n->gyr[i]);
    for (int i = 0; i < 3; i++) PK(&p, n->acc[i]);
    for (int i = 0; i < 3; i++) PK(&p, n->mag[i]);
    PK(&p, (uint8_t)((n->mag_new ? 1u : 0u) | (gap ? 2u : 0u)));
    record(USB_LOG_IMU, p.b, p.n);
}

/* GNSS: u64 t_valid, u64 t_arrival, u16 week, u32 tow_ms, u8 pos_type,
 * u8 sol_status, u8 vel_type, u8 flags (1 t_from_pps, 2 pos_usable,
 * 4 vel_usable), f64 lat_deg, f64 lon_deg, f64 hgt_msl_m, f32 undulation_m,
 * f32 sd_lat, sd_lon, sd_hgt [m], f32 diff_age_s, u8 n_tracked, u8 n_used,
 * f32 hor_speed_mps, f32 track_deg, f32 vert_speed_mps (up), f32 sd_hor,
 * f32 sd_vert. */
void usb_log_gnss(const um980_nav_t *g)
{
    if (!s_on) return;
    pk_t p = { .n = 0 };
    PK(&p, g->t_valid_us);
    PK(&p, g->t_arrival_us);
    PK(&p, g->week);
    PK(&p, g->tow_ms);
    PK(&p, (uint8_t)g->pos_type);
    PK(&p, (uint8_t)g->sol_status);
    PK(&p, (uint8_t)g->vel_type);
    PK(&p, (uint8_t)((g->t_from_pps ? 1u : 0u) | (g->pos_usable ? 2u : 0u) | (g->vel_usable ? 4u : 0u)));
    PK(&p, g->lat_deg);
    PK(&p, g->lon_deg);
    PK(&p, g->hgt_msl_m);
    PK(&p, g->undulation_m);
    PK(&p, g->sd_lat_m);
    PK(&p, g->sd_lon_m);
    PK(&p, g->sd_hgt_m);
    PK(&p, g->diff_age_s);
    PK(&p, g->n_tracked);
    PK(&p, g->n_used);
    PK(&p, (float)g->hor_speed_mps);
    PK(&p, (float)g->track_deg);
    PK(&p, (float)g->vert_speed_mps);
    PK(&p, g->sd_hor_speed_mps);
    PK(&p, g->sd_vert_speed_mps);
    record(USB_LOG_GNSS, p.b, p.n);

    /* TIME: the PPS second on both clocks, once a second (usb_log.h). Only
     * from a PPS-tagged epoch: an arrival time is late by the output
     * latency. */
    if (g->t_from_pps && g->week != 0 && g->tow_ms % 1000u == 0) {
        pk_t t = { .n = 0 };
        PK(&t, g->t_valid_us);
        PK(&t, g->week);
        PK(&t, g->tow_ms);
        PK(&t, g->leap_s);
        record(USB_LOG_TIME, t.b, t.n);
    }
}

/* HEADING: u64 t_valid, u16 week, u32 tow_ms, u8 pos_type, u8 flags
 * (1 fixed, 2 t_from_pps), f32 length_m, heading_deg, pitch_deg,
 * sd_heading_deg, sd_pitch_deg, u8 n_tracked, u8 n_used. */
void usb_log_heading(const um980_heading_t *h)
{
    if (!s_on) return;
    pk_t p = { .n = 0 };
    PK(&p, h->t_valid_us);
    PK(&p, h->week);
    PK(&p, h->tow_ms);
    PK(&p, (uint8_t)h->pos_type);
    PK(&p, (uint8_t)((h->fixed ? 1u : 0u) | (h->t_from_pps ? 2u : 0u)));
    PK(&p, h->length_m);
    PK(&p, h->heading_deg);
    PK(&p, h->pitch_deg);
    PK(&p, h->sd_heading_deg);
    PK(&p, h->sd_pitch_deg);
    PK(&p, h->n_tracked);
    PK(&p, h->n_used);
    record(USB_LOG_HEADING, p.b, p.n);
}

void usb_log_baro(uint64_t t_us, float pressure_pa, float temperature_c)
{
    if (!s_on) return;
    pk_t p = { .n = 0 };
    PK(&p, t_us);
    PK(&p, pressure_pa);
    PK(&p, temperature_c);
    record(USB_LOG_BARO, p.b, p.n);
}

/* NAV: u64 t, u8 mode, u8 att_source, u8 flags (1 rpy, 2 pos, 4 vel,
 * 8 bias, 16 standing still, 32 height), f32 rpy_rad[3], f64 llh[3]
 * (rad, rad, m), f32 vel_ned[3], f32 gyr_bias[3], f32 acc_bias[3],
 * f32 height_m. */
void usb_log_nav(const nav_output_t *o)
{
    if (!s_on) return;
    pk_t p = { .n = 0 };
    PK(&p, o->t_us);
    PK(&p, (uint8_t)o->mode);
    PK(&p, (uint8_t)o->att_source);
    PK(&p, (uint8_t)((o->rpy_valid ? 1u : 0u) | (o->pos_valid ? 2u : 0u) | (o->vel_valid ? 4u : 0u) |
                     (o->bias_valid ? 8u : 0u) | (o->zaru_active ? 16u : 0u) | (o->height_valid ? 32u : 0u)));
    for (int i = 0; i < 3; i++) PK(&p, o->rpy_rad[i]);
    for (int i = 0; i < 3; i++) PK(&p, o->llh[i]);
    for (int i = 0; i < 3; i++) PK(&p, o->vel_ned[i]);
    for (int i = 0; i < 3; i++) PK(&p, o->gyr_bias[i]);
    for (int i = 0; i < 3; i++) PK(&p, o->acc_bias[i]);
    PK(&p, o->height_m);
    record(USB_LOG_NAV, p.b, p.n);
}

static void edges_flush(void)
{
    if (s_edge_n == 0) {
        return;
    }
    record(USB_LOG_SPEED_EDGES, s_edges, 8u + s_edge_n * 5u);
    s_edge_n = 0;
}

void usb_log_speed_edge(const pulse_edge_t *e)
{
    if (!s_on) return;
    if (s_edge_n > 0 && (s_edge_n >= EDGE_MAX || e->t_us - s_edge_t0 > 0xFFFFFFFFu)) {
        edges_flush();
    }
    if (s_edge_n == 0) {
        s_edge_t0 = e->t_us;
        memcpy(&s_edges[0], &s_edge_t0, 8);
    }
    uint32_t dt = (uint32_t)(e->t_us - s_edge_t0);
    uint8_t *q = &s_edges[8u + s_edge_n * 5u];
    memcpy(q, &dt, 4);
    q[4] = e->levels;
    s_edge_n++;
}

void usb_log_flow_event(const flow_event_t *e)
{
    if (!s_on) return;
    pk_t p = { .n = 0 };
    PK(&p, e->t_us);
    PK(&p, e->seq);
    PK(&p, e->kind);
    PK(&p, e->level);
    record(USB_LOG_FLOW_EVENT, p.b, p.n);
}

void usb_log_flow_frame(const flow_frame_t *f)
{
    if (!s_on) return;
    pk_t p = { .n = 0 };
    PK(&p, f->t_arrival_us);
    PK(&p, f->type);
    memcpy(&p.b[p.n], f->payload, f->len);
    p.n += f->len;
    record(USB_LOG_FLOW_FRAME, p.b, p.n);
}
