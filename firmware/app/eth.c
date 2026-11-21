#include "eth.h"

#include <stdio.h>
#include <string.h>

#include "pico/time.h"
#include "pico/unique_id.h"

#include "aog_pgn.h"
#include "app_config.h"
#include "board_pins.h"
#include "clock_sync.h"
#include "flow_link.h"
#include "rtcm_frame.h"
#include "w5500.h"

#define SOCK_PAOGI        0u
#define SOCK_RTCM         1u
#define SOCK_FRONT        2u
#define SOCK_AOG          3u
#define AOG_PORT          8888u    /* module PGNs from AgIO / AgOpenWeb */
#define AOG_REPLY_PORT    9999u
#define FRONT_DGRAM_MAX   160u     /* a flow_link frame is at most 135 bytes */
/* Clock sync requests: faster for the first few, so the sync is valid a
 * few seconds after the unit's first frame. */
#define SYNC_PERIOD_US       1000000u
#define SYNC_FAST_PERIOD_US  250000u
#define SYNC_FAST_COUNT      8u
#define FRONT_GONE_US        5000000u /* no datagram this long: stop asking */
#define RETRY_US          5000000u /* fitted, but no chip: try again */
#define PHY_PERIOD_US     250000u
#define RTCM_PERIOD_US    1000u    /* datagrams come >= 10 ms apart (plan) */
#define RTCM_PER_POLL     8u
#define FRONT_PER_POLL    8u
#define RTCM_DGRAM_MAX    600u     /* AgOpenWeb: <= 256, bridge ceiling 512 */

static w5500_t        s_w;
static bool           s_pins_used;   /* we have driven the pins since boot */
static bool           s_dirty;       /* settings changed: apply on the next poll */
static bool           s_fitted;
static uint8_t        s_ip[4], s_mask[4], s_gw[4], s_bcast[4];
static eth_state_t    s_state;
static bool           s_socks_open;
static uint64_t       s_next_retry, s_next_phy, s_next_rtcm;
static w5500_phy_t    s_phy;
static eth_rtcm_out_fn s_rtcm_out;
static eth_flow_in_fn s_flow_in;
static rtcm_frame_t   s_rtcm;
static uint64_t       s_rtcm_last_us, s_front_last_us;
static uint8_t        s_front_ip[4];
static uint16_t       s_front_port;
static clock_sync_t   s_sync;
static uint64_t       s_next_sync;
/* The front unit's settings and heartbeat (FLOW_MSG_UNIT_CFG), once a second. */
static flow_msg_unit_cfg_t s_ucfg;
static uint32_t       s_ucfg_seq;
static uint64_t       s_next_ucfg;
#define UNIT_CFG_PERIOD_US 1000000u
static eth_status_t   s_st; /* the counters; state fields filled on read */

const char *eth_state_name(eth_state_t s)
{
    switch (s) {
    case ETH_OFF:       return "off";
    case ETH_NOT_FOUND: return "not_found";
    case ETH_NO_LINK:   return "no_link";
    case ETH_UP:        return "up";
    default:            return "?";
    }
}

void eth_init(eth_rtcm_out_fn rtcm_out, eth_flow_in_fn flow_in)
{
    s_rtcm_out = rtcm_out;
    s_flow_in = flow_in;
    rtcm_frame_init(&s_rtcm);
    pico_unique_board_id_t id;
    pico_get_unique_board_id(&id);
    /* Locally administered, unicast; the low five bytes of the unique ID. */
    s_st.mac[0] = 0x02;
    memcpy(&s_st.mac[1], &id.id[PICO_UNIQUE_BOARD_ID_SIZE_BYTES - 5], 5);
    s_st.rtcm_age_ms = -1;
    s_st.front_age_ms = -1;
    clock_sync_init(&s_sync);
}

void eth_apply_settings(const settings_t *s)
{
    settings_unit_cfg(s, 0, &s_ucfg); /* sent from the next second on */
    if (s->eth_fitted == s_fitted && memcmp(s->eth_ip, s_ip, 4) == 0 &&
        memcmp(s->eth_mask, s_mask, 4) == 0 && memcmp(s->eth_gw, s_gw, 4) == 0) {
        return;
    }
    s_fitted = s->eth_fitted != 0;
    memcpy(s_ip, s->eth_ip, 4);
    memcpy(s_mask, s->eth_mask, 4);
    memcpy(s_gw, s->eth_gw, 4);
    for (int i = 0; i < 4; i++) {
        s_bcast[i] = (uint8_t)(s_ip[i] | (uint8_t)~s_mask[i]);
    }
    s_dirty = true;
}

/* Addresses and sockets onto a chip that answers. */
static void configure(void)
{
    for (unsigned k = 0; k <= SOCK_AOG; k++) {
        w5500_close(&s_w, k);
    }
    s_socks_open = false;
    int rc = w5500_set_net(&s_w, s_st.mac, s_ip, s_mask, s_gw);
    if (rc == W5500_OK) rc = w5500_udp_open(&s_w, SOCK_PAOGI, (uint16_t)APP_ETH_PAOGI_PORT);
    if (rc == W5500_OK) rc = w5500_udp_open(&s_w, SOCK_RTCM, (uint16_t)APP_ETH_RTCM_PORT);
    if (rc == W5500_OK) rc = w5500_udp_open(&s_w, SOCK_FRONT, (uint16_t)APP_ETH_FRONT_PORT);
    if (rc == W5500_OK) rc = w5500_udp_open(&s_w, SOCK_AOG, (uint16_t)AOG_PORT);
    if (rc != W5500_OK) {
        printf("[eth] socket setup failed: %s; retrying in %u s\n", w5500_strerror(rc),
               (unsigned)(RETRY_US / 1000000u));
        s_next_retry = time_us_64() + RETRY_US;
        return;
    }
    s_socks_open = true;
    rtcm_frame_init(&s_rtcm);
    printf("[eth] %u.%u.%u.%u/%u.%u.%u.%u: $PAOGI to %u.%u.%u.%u:%u, RTCM on :%u, front unit on :%u, "
           "AOG scan on :%u\n", s_ip[0], s_ip[1], s_ip[2], s_ip[3], s_mask[0], s_mask[1], s_mask[2], s_mask[3],
           s_bcast[0], s_bcast[1], s_bcast[2], s_bcast[3], (unsigned)APP_ETH_PAOGI_PORT,
           (unsigned)APP_ETH_RTCM_PORT, (unsigned)APP_ETH_FRONT_PORT, (unsigned)AOG_PORT);
}

static void start(uint64_t now)
{
    const w5500_config_t cfg = {
        .spi = PIN_W5500_SPI,
        .pin_miso = PIN_W5500_MISO, .pin_cs = PIN_W5500_CS, .pin_sck = PIN_W5500_SCK,
        .pin_mosi = PIN_W5500_MOSI, .pin_rst = PIN_W5500_RST, .pin_int = PIN_W5500_INT,
        .baud_hz = APP_ETH_SPI_HZ,
        /* RTCM gets the big receive buffer: several datagrams between polls
         * (plan, "Hold several datagrams between polls"). The $PAOGI socket
         * receives nothing (other modules' broadcasts to 9999 would only
         * fill it), the RTCM socket sends nothing. */
        .tx_kb = { 2, 0, 2, 1, 0, 0, 0, 0 },
        .rx_kb = { 0, 8, 4, 1, 0, 0, 0, 0 },
    };
    s_pins_used = true;
    int rc = w5500_init(&s_w, &cfg);
    s_st.spi_hz = s_w.baud_actual;
    if (rc != W5500_OK) {
        if (s_state != ETH_NOT_FOUND) {
            printf("[eth] %s (SPI %lu Hz); retrying every %u s\n", w5500_strerror(rc),
                   (unsigned long)s_w.baud_actual, (unsigned)(RETRY_US / 1000000u));
        }
        w5500_hold_reset(&s_w);
        s_state = ETH_NOT_FOUND;
        s_next_retry = now + RETRY_US;
        return;
    }
    printf("[eth] W5500 found, SPI %lu Hz\n", (unsigned long)s_w.baud_actual);
    memset(&s_phy, 0, sizeof s_phy); /* so the first link up is reported */
    s_state = ETH_NO_LINK;
    s_next_phy = now;
    configure();
}

static void on_frame(void *ctx, const uint8_t *f, size_t len)
{
    (void)ctx;
    s_st.rtcm_last_type = rtcm_frame_msg_type(f, len);
    const eth_rtcm_result_t r = s_rtcm_out != NULL ? s_rtcm_out(f, len) : ETH_RTCM_NOT_SELECTED;
    if (r == ETH_RTCM_FORWARDED) {
        s_st.rtcm_forwarded++;
    } else if (r == ETH_RTCM_REFUSED) {
        s_st.rtcm_refused++;
    } else {
        s_st.rtcm_dropped++;
    }
}

static bool rtcm_poll(uint64_t now)
{
    static uint8_t buf[RTCM_DGRAM_MAX];
    bool worked = false;
    for (unsigned i = 0; i < RTCM_PER_POLL; i++) {
        uint8_t ip[4];
        int n = w5500_udp_recv(&s_w, SOCK_RTCM, ip, NULL, buf, sizeof buf);
        if (n <= 0) {
            break;
        }
        worked = true;
        s_st.rtcm_datagrams++;
        s_st.rtcm_bytes += (uint32_t)n;
        s_rtcm_last_us = now;
        memcpy(s_st.rtcm_src_ip, ip, 4);
        rtcm_frame_push(&s_rtcm, buf, (size_t)n, on_frame, NULL);
    }
    return worked;
}

static bool front_poll(void)
{
    static uint8_t buf[FRONT_DGRAM_MAX];
    bool worked = false;
    for (unsigned i = 0; i < FRONT_PER_POLL; i++) {
        uint8_t ip[4];
        uint16_t port;
        /* Taken just before the read, not at the start of the loop pass:
         * the datagram is already in the chip, so this is after its
         * arrival by at most the time since the last poll, and never
         * before it (the clock sync's round trip, the flow frame's
         * arrival). */
        const uint64_t t_rx = time_us_64();
        int n = w5500_udp_recv(&s_w, SOCK_FRONT, ip, &port, buf, sizeof buf);
        if (n <= 0) {
            break;
        }
        worked = true;
        s_st.front_datagrams++;
        uint8_t type;
        int len = flow_link_check(buf, (size_t)n, &type);
        if (len < 0) {
            s_st.front_rejected++; /* not a flow_link frame: its sender is not adopted */
            continue;
        }
        if (memcmp(ip, s_front_ip, 4) != 0 || port != s_front_port) {
            if (s_front_port != 0 && t_rx - s_front_last_us <= FRONT_GONE_US) {
                /* A second sender while the unit still sends: its frames
                 * would be timed by the wrong clock. A restarted unit is
                 * taken once the old address has been silent for
                 * FRONT_GONE_US. */
                s_st.front_other++;
                continue;
            }
            /* Another unit, or the same one restarted on a new port: its
             * clock is a different one. */
            memcpy(s_front_ip, ip, 4);
            s_front_port = port;
            clock_sync_init(&s_sync);
            s_next_sync = t_rx;
            printf("[eth] front unit at %u.%u.%u.%u:%u\n", ip[0], ip[1], ip[2], ip[3], port);
        }
        s_front_last_us = t_rx;
        memcpy(s_st.front_src_ip, ip, 4);
        const uint8_t *payload = &buf[FLOW_LINK_HEADER_LEN];
        if (type == FLOW_MSG_TIME_RESP) {
            flow_msg_time_resp_t r;
            if (flow_link_decode_time_resp(payload, (size_t)len, &r) == 0) {
                clock_sync_reply(&s_sync, r.seq, r.turnaround_us, r.unit_rx_us, t_rx);
            } else {
                s_st.front_rejected++;
            }
        } else if (s_flow_in == NULL || !s_flow_in(type, payload, (size_t)len, t_rx)) {
            s_st.front_rejected++;
        }
    }
    return worked;
}

/* The unit's settings and heartbeat, once a second while it is there. */
static bool unit_cfg_poll(uint64_t now)
{
    if (s_front_port == 0 || (int64_t)(now - s_front_last_us) > (int64_t)FRONT_GONE_US || now < s_next_ucfg) {
        return false;
    }
    uint8_t f[FLOW_LINK_UNIT_CFG_FRAME_LEN];
    s_ucfg.seq = s_ucfg_seq + 1u;
    size_t n = flow_link_encode_unit_cfg(&s_ucfg, f, sizeof f);
    int rc = w5500_udp_send(&s_w, SOCK_FRONT, s_front_ip, s_front_port, f, n);
    if (rc == W5500_E_BUSY) {
        return true; /* the previous datagram is still going out: next pass */
    }
    if (rc == W5500_OK || rc == W5500_E_SEND_FAIL) {
        s_ucfg_seq++;
        s_st.unit_cfg_sent++;
    }
    s_next_ucfg = now + UNIT_CFG_PERIOD_US;
    return true;
}

/* One time request to the unit, when due. */
static bool sync_poll(uint64_t now)
{
    /* Signed: s_front_last_us may be a little after this pass's now. */
    if (s_front_port == 0 || (int64_t)(now - s_front_last_us) > (int64_t)FRONT_GONE_US ||
        clock_sync_pending(&s_sync, now) || now < s_next_sync) {
        return false;
    }
    uint8_t f[FLOW_LINK_TIME_REQ_FRAME_LEN];
    const uint32_t seq = s_sync.seq + 1u;
    size_t n = flow_link_encode_time_req(seq, f, sizeof f);
    int rc = w5500_udp_send(&s_w, SOCK_FRONT, s_front_ip, s_front_port, f, n);
    if (rc == W5500_E_BUSY) {
        return true; /* the previous datagram is still going out: next pass */
    }
    if (rc != W5500_OK && rc != W5500_E_SEND_FAIL) {
        /* Nothing went out: not a request, so not a lost reply either. */
        s_st.sync_send_errors++;
        s_next_sync = now + SYNC_PERIOD_US;
        return true;
    }
    /* Timed by the SEND command itself (w5500.h, send_us), not after the
     * wait for the chip to take it. The destination is written to the chip
     * only when it changes, so the unit's address should not be resolved
     * by ARP again on every request; a request that does wait for ARP gets
     * a long round trip and is left out of the fit. The chip's own delay
     * from SEND to the wire is not yet measured. */
    clock_sync_request(&s_sync, s_w.send_us[SOCK_FRONT]);
    s_next_sync = now + (s_sync.requests <= SYNC_FAST_COUNT ? SYNC_FAST_PERIOD_US : SYNC_PERIOD_US);
    return true;
}

bool eth_front_clock(void *ctx, uint32_t unit_us, uint64_t now, uint64_t *pico_us)
{
    (void)ctx;
    return clock_sync_to_pico(&s_sync, unit_us, now, pico_us);
}

static bool aog_poll(void)
{
    uint8_t buf[AOG_PGN_MAX], out[AOG_PGN_MAX];
    int n = w5500_udp_recv(&s_w, SOCK_AOG, NULL, NULL, buf, sizeof buf);
    if (n <= 0) {
        return false;
    }
    aog_req_t req = aog_pgn_parse(buf, (size_t)n);
    size_t k = aog_pgn_reply(req, s_ip, out, sizeof out);
    if (k == 0) {
        return true;
    }
    static const uint8_t k_all[4] = { 255, 255, 255, 255 };
    int rc = w5500_udp_send(&s_w, SOCK_AOG, req == AOG_REQ_SCAN ? k_all : s_bcast, (uint16_t)AOG_REPLY_PORT, out, k);
    if (rc == W5500_OK || rc == W5500_E_SEND_FAIL) {
        if (req == AOG_REQ_SCAN) {
            s_st.aog_scans++;
            printf("[eth] AgOpenGPS scan answered: %u.%u.%u.%u\n", s_ip[0], s_ip[1], s_ip[2], s_ip[3]);
        } else {
            s_st.aog_hellos++;
        }
    }
    return true;
}

bool eth_poll(uint64_t now)
{
    if (s_dirty) {
        s_dirty = false;
        if (!s_fitted) {
            if (s_pins_used) {
                w5500_hold_reset(&s_w);
            }
            if (s_state != ETH_OFF) {
                printf("[eth] off (not fitted)\n");
            }
            s_state = ETH_OFF;
            s_socks_open = false;
            return true;
        }
        if (s_w.present) {
            configure(); /* new addresses */
        } else {
            start(now);
        }
        return true;
    }
    if (!s_fitted) {
        return false;
    }
    if (!s_w.present) {
        if (now >= s_next_retry) {
            start(now);
            return true;
        }
        return false;
    }
    if (!s_socks_open && now >= s_next_retry) {
        configure(); /* the last socket setup failed */
        return true;
    }
    bool worked = false;
    if (now >= s_next_phy) {
        s_next_phy = now + PHY_PERIOD_US;
        if (!w5500_alive(&s_w)) {
            /* Reset by a glitch, or unplugged: its settings are gone. */
            printf("[eth] W5500 stopped answering; starting over\n");
            w5500_hold_reset(&s_w);
            memset(&s_phy, 0, sizeof s_phy);
            s_socks_open = false;
            s_state = ETH_NOT_FOUND;
            s_next_retry = now;
            return true;
        }
        bool was = s_phy.link;
        w5500_phy(&s_w, &s_phy);
        if (s_phy.link != was) {
            if (s_phy.link) s_st.link_ups++;
            printf("[eth] link %s%s\n", s_phy.link ? "up" : "down",
                   s_phy.link ? (s_phy.mbps100 ? " 100 Mb/s" : " 10 Mb/s") : "");
        }
        s_state = s_phy.link ? ETH_UP : ETH_NO_LINK;
        worked = true;
    }
    if (s_state == ETH_UP && s_socks_open && now >= s_next_rtcm) {
        s_next_rtcm = now + RTCM_PERIOD_US;
        worked |= rtcm_poll(now);
        worked |= front_poll();
        worked |= aog_poll();
    } else if (s_state == ETH_UP && s_socks_open && clock_sync_pending(&s_sync, now)) {
        /* A reply is due: read it on every pass, not every 1 ms, so its
         * arrival time is close. */
        worked |= front_poll();
    }
    if (s_state == ETH_UP && s_socks_open) {
        const bool req = sync_poll(now);
        worked |= req;
        if (!req) worked |= unit_cfg_poll(now); /* never in the pass of a time request */
    }
    return worked;
}

bool eth_up(void) { return s_state == ETH_UP && s_socks_open; }

void eth_send_paogi(const char *line, size_t len)
{
    if (!eth_up()) {
        return;
    }
    int rc = w5500_udp_send(&s_w, SOCK_PAOGI, s_bcast, (uint16_t)APP_ETH_PAOGI_PORT, line, len);
    if (rc == W5500_OK) {
        s_st.paogi_sent++;
    } else {
        s_st.paogi_errors++;
        if (rc == W5500_E_SEND_FAIL) {
            s_st.paogi_sent++; /* the earlier one failed, this one went out */
        }
    }
}

/* Age in ms, 0 for a time stamped a little after now. */
static int32_t age_ms(uint64_t now, uint64_t t)
{
    return now > t ? (int32_t)((now - t) / 1000u) : 0;
}

void eth_get_status(eth_status_t *st, uint64_t now)
{
    *st = s_st;
    st->state = s_state;
    memcpy(st->ip, s_ip, 4);
    st->mbps100 = s_phy.mbps100;
    st->full_duplex = s_phy.full_duplex;
    st->rtcm_frames = s_rtcm.frames;
    st->rtcm_crc_errors = s_rtcm.crc_errors;
    st->rtcm_age_ms = s_rtcm_last_us == 0 ? -1 : age_ms(now, s_rtcm_last_us);
    st->front_age_ms = s_front_last_us == 0 ? -1 : age_ms(now, s_front_last_us);
    st->sync_valid = clock_sync_valid(&s_sync, now);
    st->sync_offset_us = s_sync.have ? clock_sync_offset_us(&s_sync, now) : 0.0;
    st->sync_drift_ppm = s_sync.have ? (float)clock_sync_drift_ppm(&s_sync) : 0.0f;
    st->sync_rtt_min_us = s_sync.rtt_min;
    st->sync_resid_us = s_sync.resid_us;
    st->sync_used = s_sync.used;
    st->sync_requests = s_sync.requests;
    st->sync_replies = s_sync.replies;
    st->sync_timeouts = s_sync.timeouts;
    st->sync_resets = s_sync.resets;
}
