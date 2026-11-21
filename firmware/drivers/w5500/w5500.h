/*
 * WIZnet W5500 Ethernet controller on SPI, UDP sockets only
 * (plans/ethernet-w5500-agopenweb.md).
 *
 * A small register-level driver in place of WIZnet's ioLibrary: the W5500
 * runs the TCP/IP stack itself, and this firmware needs only static IP
 * settings, UDP sockets and the PHY link bit. Register map and SPI frame
 * format: W5500 datasheet v1.1, sections 2 (SPI frame), 4.1 (common
 * registers) and 4.2 (socket registers).
 *
 *  - SPI frame: 16-bit offset, control byte (block select, read/write,
 *    variable-length mode with CS framing), then the data.
 *  - Socket buffers: the offset into a socket's TX or RX block wraps in the
 *    chip, so the pointers are used as they are.
 *  - Sending is non-blocking: w5500_udp_send() issues SEND and returns; the
 *    next send on that socket first checks that the previous one ended
 *    (SEND_OK, or TIMEOUT when ARP for a unicast address got no answer).
 *    A broadcast needs no ARP and ends within microseconds.
 *
 * The INT pin is not used: the app polls. Runs on core 0. Not thread safe.
 * Error codes are below -100.
 */
#ifndef W5500_H
#define W5500_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "hardware/spi.h"

#ifdef __cplusplus
extern "C" {
#endif

#define W5500_OK              0
#define W5500_E_NOT_FOUND  (-101) /* VERSIONR did not read 0x04              */
#define W5500_E_SOCKET     (-102) /* socket number out of range, or not open */
#define W5500_E_BUSY       (-103) /* the previous send has not ended yet     */
#define W5500_E_TOO_BIG    (-104) /* more than the socket's free TX space    */
#define W5500_E_TIMEOUT    (-105) /* a command did not finish                */
#define W5500_E_SEND_FAIL  (-106) /* the previous send ended in TIMEOUT      */

#define W5500_SOCKETS      8

typedef struct {
    spi_inst_t *spi;
    unsigned    pin_miso, pin_cs, pin_sck, pin_mosi, pin_rst, pin_int;
    uint32_t    baud_hz;
    /* Buffer per socket in KB: 0, 1, 2, 4, 8 or 16; each set sums to <= 16. */
    uint8_t     tx_kb[W5500_SOCKETS];
    uint8_t     rx_kb[W5500_SOCKETS];
} w5500_config_t;

typedef struct {
    w5500_config_t cfg;
    uint32_t baud_actual;
    bool     present;                  /* VERSIONR read 0x04 at init          */
    bool     open[W5500_SOCKETS];
    bool     send_pending[W5500_SOCKETS];
    uint32_t send_timeouts[W5500_SOCKETS];
    /* time_us_64() right after the last SEND command was written to Sn_CR,
     * before waiting for the chip to take it: the closest the app gets to
     * when the datagram leaves (the clock sync times its requests by it). */
    uint64_t send_us[W5500_SOCKETS];
    /* Destination last written to Sn_DIPR / Sn_DPORT. Rewritten only when
     * it changes: the chip resolves a unicast address by ARP after DIPR
     * changes, so a fixed destination is not looked up on every send
     * (not yet checked on a W5500). */
    uint8_t  dst_ip[W5500_SOCKETS][4];
    uint16_t dst_port[W5500_SOCKETS];
    bool     dst_set[W5500_SOCKETS];
} w5500_t;

typedef struct {
    bool link;
    bool mbps100;
    bool full_duplex;
} w5500_phy_t;

/* Sets up SPI and the pins, pulses RST, checks the chip version and sets the
 * socket buffer sizes. Blocks about 2 ms. */
int w5500_init(w5500_t *w, const w5500_config_t *cfg);

/* Holds the chip in reset (RST low, about 0 mA from its PHY). */
void w5500_hold_reset(w5500_t *w);

/* MAC, IP, subnet mask and gateway. */
int w5500_set_net(w5500_t *w, const uint8_t mac[6], const uint8_t ip[4], const uint8_t mask[4],
                  const uint8_t gw[4]);

/* True while the chip still answers (VERSIONR reads 0x04). False after a
 * power glitch, a loose cable to the module or w5500_hold_reset(): the
 * caller starts over with w5500_init(). */
bool w5500_alive(w5500_t *w);

/* PHYCFGR: link, speed and duplex. */
bool w5500_phy(w5500_t *w, w5500_phy_t *out);

/* Opens socket sock as UDP on local port. Broadcasts are received. */
int w5500_udp_open(w5500_t *w, unsigned sock, uint16_t port);
void w5500_close(w5500_t *w, unsigned sock);

/* Queues one datagram (at most the socket's TX buffer) and issues SEND.
 * Returns W5500_OK, or W5500_E_BUSY while the previous send has not ended
 * (nothing is sent), or another error. W5500_E_SEND_FAIL reports that the
 * previous send timed out; this one was still sent. send_us[sock] is the
 * time of the SEND command. */
int w5500_udp_send(w5500_t *w, unsigned sock, const uint8_t ip[4], uint16_t port, const void *data,
                   size_t len);

/* Next received datagram into buf. Returns its length (copied up to cap,
 * the rest discarded), 0 when there is none, or a negative error. ip and
 * port, if not NULL, get the sender. */
int w5500_udp_recv(w5500_t *w, unsigned sock, uint8_t ip[4], uint16_t *port, void *buf, size_t cap);

const char *w5500_strerror(int rc);

#ifdef __cplusplus
}
#endif

#endif /* W5500_H */
