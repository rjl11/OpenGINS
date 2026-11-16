#include "pulse_in.h"

#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "pico/time.h"

static int s_pin[2] = { -1, -1 };
static uint32_t s_mask;

static pulse_edge_t s_ring[PULSE_IN_RING];
static volatile uint32_t s_wr, s_rd; /* free-running, index & (RING-1) */
static volatile pulse_in_stats_t s_st;

_Static_assert((PULSE_IN_RING & (PULSE_IN_RING - 1u)) == 0, "ring size must be a power of two");

static __force_inline void edge_on(int ch, uint32_t events, uint64_t t)
{
    unsigned pin = (unsigned)s_pin[ch];
    uint32_t all = gpio_get_all();
    uint8_t lv = (uint8_t)(((all >> (unsigned)s_pin[0]) & 1u) |
                           (s_pin[1] >= 0 ? ((all >> (unsigned)s_pin[1]) & 1u) << 1 : 0u));
    /* Both edges since the last interrupt: two entries, in the order the
     * level now implies (the one matching the present level came last). */
    uint32_t want[2];
    int n = 0;
    bool high = (all >> pin) & 1u;
    if ((events & GPIO_IRQ_EDGE_RISE) && (events & GPIO_IRQ_EDGE_FALL)) {
        want[n++] = high ? GPIO_IRQ_EDGE_FALL : GPIO_IRQ_EDGE_RISE;
        want[n++] = high ? GPIO_IRQ_EDGE_RISE : GPIO_IRQ_EDGE_FALL;
    } else {
        want[n++] = events;
    }
    for (int i = 0; i < n; i++) {
        uint8_t l = lv;
        if (i == 0 && n == 2) { /* the first of a pair: this input had the other level */
            l ^= (uint8_t)(1u << ch);
        }
        if (want[i] & GPIO_IRQ_EDGE_RISE) {
            s_st.rising[ch]++;
        }
        s_st.edges[ch]++;
        uint32_t w = s_wr;
        if (w - s_rd >= PULSE_IN_RING) {
            s_st.overflows++;
            continue;
        }
        pulse_edge_t *e = &s_ring[w & (PULSE_IN_RING - 1u)];
        e->t_us = t;
        e->levels = (uint8_t)(l | (ch ? PULSE_IN_EDGE_B : 0u));
        s_wr = w + 1u;
    }
}

/* In SRAM, so the timestamp doesn't wait on an XIP cache miss. */
static void __not_in_flash_func(pulse_irq)(void)
{
    uint64_t t = time_us_64();
    for (int ch = 0; ch < 2; ch++) {
        if (s_pin[ch] < 0) {
            continue;
        }
        uint32_t ev = gpio_get_irq_event_mask((unsigned)s_pin[ch]) & (GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL);
        if (ev) {
            gpio_acknowledge_irq((unsigned)s_pin[ch], ev);
            edge_on(ch, ev, t);
        }
    }
}

void pulse_in_init(const pulse_in_config_t *cfg)
{
    s_pin[0] = cfg->pin_a;
    s_pin[1] = cfg->pin_b;
    s_mask = 0;
    for (int ch = 0; ch < 2; ch++) {
        if (s_pin[ch] < 0) {
            continue;
        }
        unsigned p = (unsigned)s_pin[ch];
        gpio_init(p);
        gpio_set_dir(p, GPIO_IN);
        if (cfg->pull_up) {
            gpio_pull_up(p);
        } else {
            gpio_disable_pulls(p);
        }
        gpio_set_input_hysteresis_enabled(p, true);
        gpio_acknowledge_irq(p, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL);
        s_mask |= 1u << p;
    }
    if (s_mask == 0) {
        return;
    }
    gpio_add_raw_irq_handler_masked(s_mask, pulse_irq);
    for (int ch = 0; ch < 2; ch++) {
        if (s_pin[ch] >= 0) {
            gpio_set_irq_enabled((unsigned)s_pin[ch], GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
        }
    }
    irq_set_enabled(IO_IRQ_BANK0, true);
}

void pulse_in_set_enabled(bool on)
{
    for (int ch = 0; ch < 2; ch++) {
        if (s_pin[ch] >= 0) {
            unsigned p = (unsigned)s_pin[ch];
            gpio_set_irq_enabled(p, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);
            gpio_acknowledge_irq(p, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL);
            if (on) {
                gpio_set_irq_enabled(p, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
            }
        }
    }
}

bool pulse_in_pop(pulse_edge_t *e)
{
    uint32_t r = s_rd;
    if (r == s_wr) {
        return false;
    }
    *e = s_ring[r & (PULSE_IN_RING - 1u)];
    s_rd = r + 1u;
    return true;
}

void pulse_in_stats(pulse_in_stats_t *out)
{
    uint32_t save = save_and_disable_interrupts();
    out->edges[0] = s_st.edges[0];
    out->edges[1] = s_st.edges[1];
    out->rising[0] = s_st.rising[0];
    out->rising[1] = s_st.rising[1];
    out->overflows = s_st.overflows;
    restore_interrupts(save);
}
