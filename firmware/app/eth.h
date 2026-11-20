/*
 * Wired Ethernet to AgOpenWeb / AgOpenGPS over a W5500 (drivers/w5500,
 * plans/ethernet-w5500-agopenweb.md).
 *
 *  - Socket 0: $PAOGI out, one sentence per datagram, to the subnet
 *    broadcast on APP_ETH_PAOGI_PORT (AgOpenWeb's module port 9999). The
 *    sentence is the one main.c also sends on USB (app/paogi.h).
 *  - Socket 1: RTCM in on APP_ETH_RTCM_PORT (2233), unicast and broadcast,
 *    from AgOpenWeb's NTRIP client. The datagrams are pieces of the stream,
 *    so frames are put back together and CRC-checked (app/rtcm_frame.h);
 *    each good frame goes to the rtcm_out callback, which passes it through
 *    the correction-source arbiter (app/rtcm_src.h) to the receiver's COM1.
 *  - Socket 2: the front camera unit on APP_ETH_FRONT_PORT (5800,
 *    plans/front-camera-unit.md): one common/flow_link.h frame per
 *    datagram, handed to the flow_in callback with its arrival time, the
 *    same path as frames on USB. Once the unit has sent a datagram, the
 *    Pico sends it FLOW_MSG_TIME_REQ (every 1 s, faster at first) and
 *    keeps the clock sync (app/clock_sync.h) from its FLOW_MSG_TIME_RESP
 *    replies; eth_front_clock() hands it to the flow timing. Settings for
 *    the unit (camera height, mounting) are not sent yet.
 *  - Socket 3: AgOpenGPS / AgOpenWeb module PGNs on UDP 8888 (app/aog_pgn.h):
 *    the scan request is answered to 255.255.255.255:9999, so a scan also
 *    finds the unit on another subnet; the hello to the subnet broadcast.
 *  - Link up/down from the W5500 PHY; the MAC comes from the RP2350's
 *    unique ID with the locally administered bit set.
 *
 * Fitted or not, and the IPv4 addresses, are web settings applied live
 * (settings.h, eth_*). Not fitted: the pins are left alone. Fitted but no
 * chip answering: retried every few seconds.
 *
 * Core 0 only.
 */
#ifndef ETH_H
#define ETH_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "settings.h"

typedef enum {
    ETH_OFF = 0,     /* not fitted (web setting)                   */
    ETH_NOT_FOUND,   /* fitted, but the W5500 doesn't answer       */
    ETH_NO_LINK,     /* chip found, no cable / switch              */
    ETH_UP           /* link up, sockets open                      */
} eth_state_t;

/* Takes one whole RTCM frame; false if it couldn't be queued (the frame is
 * then counted as dropped). */
typedef bool (*eth_rtcm_out_fn)(const uint8_t *frame, size_t len);
/* Takes one datagram from the front camera unit (a flow_link frame). */
typedef bool (*eth_flow_in_fn)(const uint8_t *frame, size_t len, uint64_t t_arrival_us);

typedef struct {
    eth_state_t state;
    uint8_t  mac[6];
    uint8_t  ip[4];
    bool     mbps100, full_duplex;
    uint32_t link_ups;          /* link up events since boot              */
    uint32_t spi_hz;

    uint32_t paogi_sent;        /* datagrams queued in the chip           */
    uint32_t paogi_errors;      /* refused: busy, no space, ARP timeout   */

    uint32_t rtcm_datagrams;
    uint32_t rtcm_bytes;
    uint32_t rtcm_frames;       /* good frames                            */
    uint32_t rtcm_crc_errors;
    uint32_t rtcm_forwarded;    /* taken by rtcm_out                      */
    uint32_t rtcm_dropped;      /* refused by rtcm_out                    */
    int32_t  rtcm_age_ms;       /* since the last datagram, -1 = none yet */
    uint8_t  rtcm_src_ip[4];
    unsigned rtcm_last_type;    /* message number of the last good frame  */

    uint32_t front_datagrams;   /* from the front camera unit             */
    uint32_t front_rejected;    /* refused by flow_in (bad frame)         */
    int32_t  front_age_ms;      /* since the last datagram, -1 = none yet */
    uint8_t  front_src_ip[4];

    bool     sync_valid;        /* clock sync with the front unit         */
    double   sync_offset_us;    /* Pico - unit clock                      */
    float    sync_drift_ppm;
    uint32_t sync_rtt_min_us;   /* shortest round trip in the window      */
    float    sync_resid_us;     /* rms of the replies used about the fit  */
    uint32_t sync_used, sync_requests, sync_replies, sync_timeouts, sync_resets;

    uint32_t aog_scans;         /* scan requests answered                 */
    uint32_t aog_hellos;        /* hellos answered                        */
} eth_status_t;

void eth_init(eth_rtcm_out_fn rtcm_out, eth_flow_in_fn flow_in);

/* The settings in use, at boot and after every change. */
void eth_apply_settings(const settings_t *s);

/* Every pass of the core 0 loop. Returns true if it did work. */
bool eth_poll(uint64_t now);

/* Link up and the $PAOGI socket open. */
bool eth_up(void);

/* One whole $PAOGI line (with CR LF) as one datagram. */
void eth_send_paogi(const char *line, size_t len);

void eth_get_status(eth_status_t *st, uint64_t now);

/* The front unit's low 32 bits of time on the Pico clock (flow_nav_clock_fn);
 * false while the sync is not valid. */
bool eth_front_clock(void *ctx, uint32_t unit_us, uint64_t now, uint64_t *pico_us);
const char *eth_state_name(eth_state_t s);

#endif /* ETH_H */
