#include "w5500.h"

#include <string.h>

#include "hardware/gpio.h"
#include "pico/time.h"

/* Block select (datasheet 2.2.2). */
#define BSB_COMMON          0x00u
#define BSB_SREG(n)         ((uint8_t)((n) * 4u + 1u))
#define BSB_STX(n)          ((uint8_t)((n) * 4u + 2u))
#define BSB_SRX(n)          ((uint8_t)((n) * 4u + 3u))
#define CTRL_WRITE          0x04u /* RWB; OM = 00, variable length */

/* Common registers (4.1). */
#define MR                  0x0000u
#define GAR                 0x0001u
#define SUBR                0x0005u
#define SHAR                0x0009u
#define SIPR                0x000Fu
#define RTR                 0x0019u
#define RCR                 0x001Bu
#define PHYCFGR             0x002Eu
#define VERSIONR            0x0039u
#define MR_RST              0x80u
#define PHY_LNK             0x01u
#define PHY_SPD             0x02u
#define PHY_DPX             0x04u
#define CHIP_VERSION        0x04u

/* Socket registers (4.2). */
#define SN_MR               0x0000u
#define SN_CR               0x0001u
#define SN_IR               0x0002u
#define SN_SR               0x0003u
#define SN_PORT             0x0004u
#define SN_DIPR             0x000Cu
#define SN_DPORT            0x0010u
#define SN_RXBUF_SIZE       0x001Eu
#define SN_TXBUF_SIZE       0x001Fu
#define SN_TX_FSR           0x0020u
#define SN_TX_WR            0x0024u
#define SN_RX_RSR           0x0026u
#define SN_RX_RD            0x0028u
#define MR_UDP              0x02u
#define CR_OPEN             0x01u
#define CR_CLOSE            0x10u
#define CR_SEND             0x20u
#define CR_RECV             0x40u
#define IR_SEND_OK          0x10u
#define IR_TIMEOUT          0x08u
#define SR_CLOSED           0x00u
#define SR_UDP              0x22u

#define CMD_TIMEOUT_US      2000u

/* ---- SPI frames ----------------------------------------------------------- */

static void xfer(w5500_t *w, uint16_t addr, uint8_t ctrl, const uint8_t *tx, uint8_t *rx, size_t n)
{
    const uint8_t hdr[3] = { (uint8_t)(addr >> 8), (uint8_t)addr, ctrl };
    gpio_put(w->cfg.pin_cs, 0);
    spi_write_blocking(w->cfg.spi, hdr, sizeof hdr);
    if (tx != NULL) {
        spi_write_blocking(w->cfg.spi, tx, n);
    } else if (rx != NULL) {
        spi_read_blocking(w->cfg.spi, 0, rx, n);
    }
    gpio_put(w->cfg.pin_cs, 1);
}

static void wr(w5500_t *w, uint8_t bsb, uint16_t addr, const void *data, size_t n)
{
    xfer(w, addr, (uint8_t)((bsb << 3) | CTRL_WRITE), data, NULL, n);
}

static void rd(w5500_t *w, uint8_t bsb, uint16_t addr, void *data, size_t n)
{
    xfer(w, addr, (uint8_t)(bsb << 3), NULL, data, n);
}

static void wr8(w5500_t *w, uint8_t bsb, uint16_t addr, uint8_t v) { wr(w, bsb, addr, &v, 1); }

static uint8_t rd8(w5500_t *w, uint8_t bsb, uint16_t addr)
{
    uint8_t v;
    rd(w, bsb, addr, &v, 1);
    return v;
}

static void wr16(w5500_t *w, uint8_t bsb, uint16_t addr, uint16_t v)
{
    const uint8_t b[2] = { (uint8_t)(v >> 8), (uint8_t)v };
    wr(w, bsb, addr, b, 2);
}

static uint16_t rd16(w5500_t *w, uint8_t bsb, uint16_t addr)
{
    uint8_t b[2];
    rd(w, bsb, addr, b, 2);
    return (uint16_t)((b[0] << 8) | b[1]);
}

/* TX_FSR and RX_RSR change while the chip works: read until two agree
 * (datasheet 4.2, Sn_TX_FSR note). */
static uint16_t rd16_stable(w5500_t *w, uint8_t bsb, uint16_t addr)
{
    uint16_t a = rd16(w, bsb, addr), b;
    for (int i = 0; i < 8; i++) {
        b = rd16(w, bsb, addr);
        if (a == b) {
            break;
        }
        a = b;
    }
    return a;
}

static int sock_cmd(w5500_t *w, unsigned s, uint8_t cmd)
{
    wr8(w, BSB_SREG(s), SN_CR, cmd);
    uint64_t t0 = time_us_64();
    if (cmd == CR_SEND) {
        w->send_us[s] = t0;
    }
    while (rd8(w, BSB_SREG(s), SN_CR) != 0) {
        if (time_us_64() - t0 > CMD_TIMEOUT_US) {
            return W5500_E_TIMEOUT;
        }
    }
    return W5500_OK;
}

/* ---- Public --------------------------------------------------------------- */

int w5500_init(w5500_t *w, const w5500_config_t *cfg)
{
    memset(w, 0, sizeof *w);
    w->cfg = *cfg;

    gpio_init(cfg->pin_cs);
    gpio_set_dir(cfg->pin_cs, GPIO_OUT);
    gpio_put(cfg->pin_cs, 1);
    gpio_init(cfg->pin_rst);
    gpio_set_dir(cfg->pin_rst, GPIO_OUT);
    gpio_init(cfg->pin_int);
    gpio_set_dir(cfg->pin_int, GPIO_IN);
    gpio_pull_up(cfg->pin_int);

    w->baud_actual = spi_init(cfg->spi, cfg->baud_hz); /* mode 0, 8 bits, MSB first */
    gpio_set_function(cfg->pin_miso, GPIO_FUNC_SPI);
    gpio_set_function(cfg->pin_sck, GPIO_FUNC_SPI);
    gpio_set_function(cfg->pin_mosi, GPIO_FUNC_SPI);

    /* RSTn low >= 500 us, then up to 1 ms for the PLL to lock (5.5.1). */
    gpio_put(cfg->pin_rst, 0);
    sleep_us(600);
    gpio_put(cfg->pin_rst, 1);
    sleep_us(1500);

    if (rd8(w, BSB_COMMON, VERSIONR) != CHIP_VERSION) {
        return W5500_E_NOT_FOUND;
    }
    wr8(w, BSB_COMMON, MR, MR_RST); /* software reset as well; clears itself */
    uint64_t t0 = time_us_64();
    while (rd8(w, BSB_COMMON, MR) & MR_RST) {
        if (time_us_64() - t0 > CMD_TIMEOUT_US) {
            return W5500_E_TIMEOUT;
        }
    }
    /* ARP retries for a unicast destination: 100 ms x 3 (default 200 ms x 8,
     * which would hold a socket's send for 1.6 s). */
    wr16(w, BSB_COMMON, RTR, 1000); /* units of 100 us */
    wr8(w, BSB_COMMON, RCR, 3);
    for (unsigned s = 0; s < W5500_SOCKETS; s++) {
        wr8(w, BSB_SREG(s), SN_TXBUF_SIZE, cfg->tx_kb[s]);
        wr8(w, BSB_SREG(s), SN_RXBUF_SIZE, cfg->rx_kb[s]);
    }
    w->present = true;
    return W5500_OK;
}

void w5500_hold_reset(w5500_t *w)
{
    gpio_put(w->cfg.pin_rst, 0);
    w->present = false;
    memset(w->open, 0, sizeof w->open);
    memset(w->send_pending, 0, sizeof w->send_pending);
    memset(w->dst_set, 0, sizeof w->dst_set);
}

int w5500_set_net(w5500_t *w, const uint8_t mac[6], const uint8_t ip[4], const uint8_t mask[4],
                  const uint8_t gw[4])
{
    if (!w->present) {
        return W5500_E_NOT_FOUND;
    }
    wr(w, BSB_COMMON, SHAR, mac, 6);
    wr(w, BSB_COMMON, SIPR, ip, 4);
    wr(w, BSB_COMMON, SUBR, mask, 4);
    wr(w, BSB_COMMON, GAR, gw, 4);
    return W5500_OK;
}

bool w5500_alive(w5500_t *w)
{
    return w->present && rd8(w, BSB_COMMON, VERSIONR) == CHIP_VERSION;
}

bool w5500_phy(w5500_t *w, w5500_phy_t *out)
{
    memset(out, 0, sizeof *out);
    if (!w->present) {
        return false;
    }
    uint8_t p = rd8(w, BSB_COMMON, PHYCFGR);
    out->link = (p & PHY_LNK) != 0;
    out->mbps100 = (p & PHY_SPD) != 0;
    out->full_duplex = (p & PHY_DPX) != 0;
    return out->link;
}

int w5500_udp_open(w5500_t *w, unsigned s, uint16_t port)
{
    if (!w->present || s >= W5500_SOCKETS) {
        return W5500_E_SOCKET;
    }
    w5500_close(w, s);
    wr8(w, BSB_SREG(s), SN_MR, MR_UDP); /* broadcasts accepted (BCASTB 0) */
    wr16(w, BSB_SREG(s), SN_PORT, port);
    int rc = sock_cmd(w, s, CR_OPEN);
    if (rc != W5500_OK) {
        return rc;
    }
    if (rd8(w, BSB_SREG(s), SN_SR) != SR_UDP) {
        return W5500_E_SOCKET;
    }
    w->open[s] = true;
    return W5500_OK;
}

void w5500_close(w5500_t *w, unsigned s)
{
    if (!w->present || s >= W5500_SOCKETS) {
        return;
    }
    (void)sock_cmd(w, s, CR_CLOSE);
    wr8(w, BSB_SREG(s), SN_IR, 0xFF);
    w->open[s] = false;
    w->send_pending[s] = false;
    w->dst_set[s] = false;
}

int w5500_udp_send(w5500_t *w, unsigned s, const uint8_t ip[4], uint16_t port, const void *data,
                   size_t len)
{
    if (!w->present || s >= W5500_SOCKETS || !w->open[s]) {
        return W5500_E_SOCKET;
    }
    int ret = W5500_OK;
    if (w->send_pending[s]) {
        uint8_t ir = rd8(w, BSB_SREG(s), SN_IR);
        if (ir & IR_SEND_OK) {
            wr8(w, BSB_SREG(s), SN_IR, IR_SEND_OK);
        } else if (ir & IR_TIMEOUT) {
            wr8(w, BSB_SREG(s), SN_IR, IR_TIMEOUT);
            w->send_timeouts[s]++;
            ret = W5500_E_SEND_FAIL; /* reported, this one still goes out */
        } else {
            return W5500_E_BUSY;
        }
        w->send_pending[s] = false;
    }
    if (len > rd16_stable(w, BSB_SREG(s), SN_TX_FSR)) {
        return W5500_E_TOO_BIG;
    }
    if (!w->dst_set[s] || memcmp(w->dst_ip[s], ip, 4) != 0) {
        wr(w, BSB_SREG(s), SN_DIPR, ip, 4);
        memcpy(w->dst_ip[s], ip, 4);
    }
    if (!w->dst_set[s] || w->dst_port[s] != port) {
        wr16(w, BSB_SREG(s), SN_DPORT, port);
        w->dst_port[s] = port;
    }
    w->dst_set[s] = true;
    uint16_t ptr = rd16(w, BSB_SREG(s), SN_TX_WR);
    wr(w, BSB_STX(s), ptr, data, len); /* the offset wraps in the chip */
    wr16(w, BSB_SREG(s), SN_TX_WR, (uint16_t)(ptr + len));
    int rc = sock_cmd(w, s, CR_SEND);
    if (rc != W5500_OK) {
        return rc;
    }
    w->send_pending[s] = true;
    return ret;
}

int w5500_udp_recv(w5500_t *w, unsigned s, uint8_t ip[4], uint16_t *port, void *buf, size_t cap)
{
    if (!w->present || s >= W5500_SOCKETS || !w->open[s]) {
        return W5500_E_SOCKET;
    }
    uint16_t avail = rd16_stable(w, BSB_SREG(s), SN_RX_RSR);
    if (avail < 8u) {
        return 0;
    }
    uint16_t ptr = rd16(w, BSB_SREG(s), SN_RX_RD);
    uint8_t hdr[8]; /* source IP, source port, data length (5.2.2.2) */
    rd(w, BSB_SRX(s), ptr, hdr, sizeof hdr);
    uint16_t n = (uint16_t)((hdr[6] << 8) | hdr[7]);
    if ((size_t)n + 8u > avail) {
        /* Inconsistent: drop everything waiting. */
        wr16(w, BSB_SREG(s), SN_RX_RD, (uint16_t)(ptr + avail));
        (void)sock_cmd(w, s, CR_RECV);
        return 0;
    }
    size_t take = n < cap ? n : cap;
    if (take > 0) {
        rd(w, BSB_SRX(s), (uint16_t)(ptr + 8u), buf, take);
    }
    wr16(w, BSB_SREG(s), SN_RX_RD, (uint16_t)(ptr + 8u + n));
    (void)sock_cmd(w, s, CR_RECV);
    if (ip != NULL) {
        memcpy(ip, hdr, 4);
    }
    if (port != NULL) {
        *port = (uint16_t)((hdr[4] << 8) | hdr[5]);
    }
    return (int)take;
}

const char *w5500_strerror(int rc)
{
    switch (rc) {
    case W5500_OK:          return "ok";
    case W5500_E_NOT_FOUND: return "W5500 not found";
    case W5500_E_SOCKET:    return "socket not open";
    case W5500_E_BUSY:      return "previous send pending";
    case W5500_E_TOO_BIG:   return "no TX space";
    case W5500_E_TIMEOUT:   return "command timeout";
    case W5500_E_SEND_FAIL: return "previous send timed out (ARP)";
    default:                return "?";
    }
}
