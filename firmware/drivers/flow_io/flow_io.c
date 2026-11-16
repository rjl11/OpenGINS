#include "flow_io.h"

#include <string.h>

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"
#include "pico/time.h"

#include "flow_uart_rx.pio.h"

/* ---- events (trigger and strobe), written in the interrupts ------------ */

#define EV_RING 256u
static flow_event_t s_ev[EV_RING];
static volatile uint32_t s_ev_wr, s_ev_rd;
static volatile flow_io_stats_t s_st;

static __force_inline void ev_put(uint64_t t, uint32_t seq, uint8_t kind, uint8_t level)
{
    uint32_t w = s_ev_wr;
    if (w - s_ev_rd >= EV_RING) {
        s_st.event_overflows++;
        return;
    }
    flow_event_t *e = &s_ev[w & (EV_RING - 1u)];
    e->t_us = t;
    e->seq = seq;
    e->kind = kind;
    e->level = level;
    s_ev_wr = w + 1u;
}

/* ---- trigger: PWM, one pulse left out every FLOW_LINK_GAP_PERIOD -------- */

static int s_trig_slice = -1;
static int s_trig_pin = -1;
static unsigned s_trig_chan;
static uint16_t s_trig_level;
static uint32_t s_tick_q16;   /* one PWM count in us, Q16 */
static uint32_t s_period;     /* index of the period that just started */
static bool s_skip_now;       /* the period that just started has no pulse */

/* Wrap interrupt: the counter went back to 0, which is the rising edge of
 * this period's pulse (or would be, for the left-out one). The counter has
 * moved on by the interrupt latency; subtracting it gives the edge time. */
static void __not_in_flash_func(trig_irq)(void)
{
    if (s_trig_slice < 0 || !(pwm_get_irq_status_mask() & (1u << (unsigned)s_trig_slice))) {
        return;
    }
    uint64_t now = time_us_64();
    uint32_t ctr = pwm_get_counter((unsigned)s_trig_slice);
    pwm_clear_irq((unsigned)s_trig_slice);
    uint64_t t = now - (((uint64_t)ctr * s_tick_q16) >> 16);

    s_period++;
    if (s_skip_now) {
        s_st.gaps++;
        ev_put(t, s_period, FLOW_EV_GAP, 0);
    } else {
        s_st.triggers++;
        ev_put(t, s_period, FLOW_EV_TRIGGER, 1);
    }
    /* The compare value is double buffered: this one applies to the next
     * period. */
    s_skip_now = ((s_period + 1u) % FLOW_LINK_GAP_PERIOD) == 0u;
    pwm_set_chan_level((unsigned)s_trig_slice, s_trig_chan, s_skip_now ? 0u : s_trig_level);
}

static void trigger_init(unsigned pin, uint32_t hz, uint32_t width_us)
{
    uint32_t sys = clock_get_hz(clk_sys);
    uint32_t div = (uint32_t)(((uint64_t)sys + (uint64_t)hz * 65536u - 1u) / ((uint64_t)hz * 65536u));
    if (div < 1u) div = 1u;
    if (div > 255u) div = 255u;
    /* Prefer a divider that makes the period exact (150 MHz, 100 Hz: 25). */
    for (uint32_t d = div; d <= 255u; d++) {
        if (sys % (d * hz) == 0u) {
            div = d;
            break;
        }
    }
    uint32_t top = sys / (div * hz) - 1u;
    if (top > 65535u) top = 65535u;
    s_tick_q16 = (uint32_t)(((uint64_t)div * 1000000u << 16) / sys);
    uint64_t lvl = ((uint64_t)width_us << 16) / s_tick_q16;
    s_trig_level = (uint16_t)(lvl < 1u ? 1u : lvl > top ? top : lvl);

    s_trig_pin = (int)pin;
    s_trig_slice = (int)pwm_gpio_to_slice_num(pin);
    s_trig_chan = pwm_gpio_to_channel(pin);
    gpio_set_function(pin, GPIO_FUNC_PWM);
    gpio_set_drive_strength(pin, GPIO_DRIVE_STRENGTH_12MA);

    pwm_config c = pwm_get_default_config();
    pwm_config_set_clkdiv_int(&c, div);
    pwm_config_set_wrap(&c, (uint16_t)top);
    pwm_init((unsigned)s_trig_slice, &c, false);
    pwm_set_chan_level((unsigned)s_trig_slice, s_trig_chan, s_trig_level);
    s_period = 0;
    s_skip_now = false;

    pwm_clear_irq((unsigned)s_trig_slice);
    /* Exclusive: nothing else uses the PWM interrupt, and the SDK's shared
     * handler slots are all taken (USB, CYW43, GPIO). */
    irq_set_exclusive_handler(PWM_DEFAULT_IRQ_NUM(), trig_irq);
    pwm_set_irq_enabled((unsigned)s_trig_slice, true);
    irq_set_enabled(PWM_DEFAULT_IRQ_NUM(), true);
    pwm_set_enabled((unsigned)s_trig_slice, true);
}

/* ---- strobe: both edges ------------------------------------------------- */

static int s_strobe_pin = -1;
static uint32_t s_strobe_n;

static void __not_in_flash_func(strobe_irq)(void)
{
    uint64_t t = time_us_64();
    unsigned pin = (unsigned)s_strobe_pin;
    uint32_t ev = gpio_get_irq_event_mask(pin) & (GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL);
    if (!ev) {
        return;
    }
    gpio_acknowledge_irq(pin, ev);
    bool high = gpio_get(pin);
    if ((ev & GPIO_IRQ_EDGE_RISE) && (ev & GPIO_IRQ_EDGE_FALL)) {
        s_st.strobe_edges++;
        ev_put(t, ++s_strobe_n, FLOW_EV_STROBE, high ? 0 : 1); /* the edge before */
    }
    s_st.strobe_edges++;
    ev_put(t, ++s_strobe_n, FLOW_EV_STROBE, high ? 1 : 0);
}

static void strobe_init(unsigned pin)
{
    s_strobe_pin = (int)pin;
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_up(pin); /* open collector, active low */
    gpio_set_input_hysteresis_enabled(pin, true);
    gpio_acknowledge_irq(pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL);
    gpio_add_raw_irq_handler(pin, strobe_irq);
    gpio_set_irq_enabled(pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    irq_set_enabled(IO_IRQ_BANK0, true);
}

/* ---- link: PIO UART RX into a DMA ring, then the frame parser ----------- */

#define RX_BITS      10 /* 1 KB, about 90 ms at 115200 */
#define RX_SIZE      (1u << RX_BITS)
#define RX_DMA_COUNT (1u << 24)

static uint8_t s_rx[RX_SIZE] __attribute__((aligned(RX_SIZE)));
static int s_rx_dma = -1;
static uint32_t s_rx_count; /* DMA bytes seen so far, mod RX_DMA_COUNT */
static uint32_t s_rx_rd;    /* ring read index */

static struct {
    uint8_t  buf[FLOW_LINK_HEADER_LEN + FLOW_LINK_MAX_PAYLOAD + FLOW_LINK_CRC_LEN];
    uint32_t n;
    uint32_t need;
} s_fr;

static bool link_init(unsigned pin, uint32_t baud)
{
    PIO pio;
    uint sm, offset;
    if (!pio_claim_free_sm_and_add_program_for_gpio_range(&flow_uart_rx_program, &pio, &sm, &offset, pin, 1,
                                                          true)) {
        return false;
    }
    s_rx_dma = dma_claim_unused_channel(false);
    if (s_rx_dma < 0) {
        return false;
    }
    flow_uart_rx_program_init(pio, sm, offset, pin, baud);

    dma_channel_config c = dma_channel_get_default_config((unsigned)s_rx_dma);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_ring(&c, true, RX_BITS);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, false));
    /* The byte sits in the top 8 bits of the FIFO word (shift right). */
    const volatile uint8_t *src = (const volatile uint8_t *)&pio->rxf[sm] + 3;
    dma_channel_configure((unsigned)s_rx_dma, &c, s_rx, src,
                          dma_encode_transfer_count_with_self_trigger(RX_DMA_COUNT), true);
    return true;
}

static uint32_t rx_dma_count(void)
{
    uint32_t left = dma_hw->ch[s_rx_dma].transfer_count & DMA_CH0_TRANS_COUNT_COUNT_BITS;
    return (RX_DMA_COUNT - left) & (RX_DMA_COUNT - 1u);
}

/* One byte into the frame parser; true when a good frame is complete. */
static bool parse_byte(uint8_t b)
{
    if (s_fr.n == 0) {
        if (b == FLOW_LINK_SYNC0) {
            s_fr.buf[s_fr.n++] = b;
        }
        return false;
    }
    if (s_fr.n == 1) {
        if (b == FLOW_LINK_SYNC1) {
            s_fr.buf[s_fr.n++] = b;
        } else {
            s_fr.n = b == FLOW_LINK_SYNC0 ? 1u : 0u;
        }
        return false;
    }
    s_fr.buf[s_fr.n++] = b;
    if (s_fr.n == FLOW_LINK_HEADER_LEN) {
        uint8_t len = s_fr.buf[4];
        if (len > FLOW_LINK_MAX_PAYLOAD) {
            s_st.long_frames++;
            s_fr.n = 0;
            return false;
        }
        s_fr.need = FLOW_LINK_HEADER_LEN + len + FLOW_LINK_CRC_LEN;
        return false;
    }
    if (s_fr.n < FLOW_LINK_HEADER_LEN || s_fr.n < s_fr.need) {
        return false;
    }
    uint32_t len = s_fr.buf[4];
    uint16_t crc = flow_link_crc16(&s_fr.buf[2], 3u + len);
    uint16_t got = (uint16_t)(s_fr.buf[FLOW_LINK_HEADER_LEN + len] | (s_fr.buf[FLOW_LINK_HEADER_LEN + len + 1u] << 8));
    s_fr.n = 0;
    if (crc != got) {
        s_st.crc_errors++;
        return false;
    }
    if (s_fr.buf[2] != FLOW_LINK_VERSION) {
        s_st.bad_version++;
        return false;
    }
    s_st.frames++;
    return true;
}

bool flow_io_poll_frame(flow_frame_t *f)
{
    if (s_rx_dma < 0) {
        return false;
    }
    uint32_t count = rx_dma_count();
    uint32_t avail = (count - s_rx_count) & (RX_DMA_COUNT - 1u);
    if (avail > RX_SIZE) { /* lapped: drop what the ring holds */
        s_st.rx_overflows++;
        s_rx_count = count;
        s_rx_rd = count & (RX_SIZE - 1u);
        s_fr.n = 0;
        return false;
    }
    while (avail > 0) {
        uint8_t b = s_rx[s_rx_rd];
        s_rx_rd = (s_rx_rd + 1u) & (RX_SIZE - 1u);
        s_rx_count = (s_rx_count + 1u) & (RX_DMA_COUNT - 1u);
        avail--;
        s_st.rx_bytes++;
        if (parse_byte(b)) {
            f->t_arrival_us = time_us_64();
            f->type = s_fr.buf[3];
            f->len = s_fr.buf[4];
            memcpy(f->payload, &s_fr.buf[FLOW_LINK_HEADER_LEN], f->len);
            return true;
        }
    }
    return false;
}

/* ---- public ------------------------------------------------------------- */

bool flow_io_init(const flow_io_config_t *cfg)
{
    if (cfg->trigger_pin >= 0 && cfg->trigger_hz > 0) {
        trigger_init((unsigned)cfg->trigger_pin, cfg->trigger_hz, cfg->trigger_us);
    }
    if (cfg->strobe_pin >= 0) {
        strobe_init((unsigned)cfg->strobe_pin);
    }
    if (cfg->rx_pin >= 0) {
        return link_init((unsigned)cfg->rx_pin, cfg->baud);
    }
    return true;
}

void flow_io_set_enabled(bool on)
{
    if (s_trig_slice >= 0) {
        unsigned sl = (unsigned)s_trig_slice, pin = (unsigned)s_trig_pin;
        pwm_set_enabled(sl, false);
        pwm_set_counter(sl, 0);
        pwm_clear_irq(sl);
        if (on) {
            gpio_set_function(pin, GPIO_FUNC_PWM);
            pwm_set_enabled(sl, true);
        } else {
            /* A stopped PWM holds its level, which may be mid-pulse: hand the
             * pin to plain GPIO, low. */
            gpio_init(pin);
            gpio_put(pin, false);
            gpio_set_dir(pin, GPIO_OUT);
        }
    }
    if (s_strobe_pin >= 0) {
        unsigned p = (unsigned)s_strobe_pin;
        gpio_set_irq_enabled(p, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);
        gpio_acknowledge_irq(p, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL);
        if (on) {
            gpio_set_irq_enabled(p, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
        }
    }
}

bool flow_io_pop_event(flow_event_t *e)
{
    uint32_t r = s_ev_rd;
    if (r == s_ev_wr) {
        return false;
    }
    *e = s_ev[r & (EV_RING - 1u)];
    s_ev_rd = r + 1u;
    return true;
}

void flow_io_stats(flow_io_stats_t *out)
{
    uint32_t save = save_and_disable_interrupts();
    memcpy(out, (const void *)&s_st, sizeof *out);
    restore_interrupts(save);
}
