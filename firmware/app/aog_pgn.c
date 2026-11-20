#include "aog_pgn.h"

uint8_t aog_pgn_crc(const uint8_t *f, size_t n)
{
    unsigned sum = 0;
    for (size_t i = 2; i < n; i++) {
        sum += f[i];
    }
    return (uint8_t)sum;
}

aog_req_t aog_pgn_parse(const uint8_t *b, size_t n)
{
    if (n < 6 || b[0] != 0x80 || b[1] != 0x81) {
        return AOG_REQ_NONE;
    }
    size_t len = b[4];
    if (n < 5u + len + 1u) {
        return AOG_REQ_NONE;
    }
    /* The CRC byte is not checked: these two requests are fixed messages,
     * and AgIO is remembered to send them with a constant 0x47 in place of
     * the sum (not verified against its source here). The fixed bytes are
     * checked instead. */
    if (b[3] == AOG_PGN_HELLO && b[2] == 0x7F) {
        return AOG_REQ_HELLO;
    }
    if (b[3] == AOG_PGN_SCAN && len >= 3 && b[5] == 202 && b[6] == 202 && b[7] == 5) {
        return AOG_REQ_SCAN;
    }
    return AOG_REQ_NONE;
}

size_t aog_pgn_reply(aog_req_t req, const uint8_t ip[4], uint8_t *out, size_t cap)
{
    size_t n;
    if (req == AOG_REQ_SCAN) {
        if (cap < 13) return 0;
        out[0] = 0x80; out[1] = 0x81; out[2] = AOG_SRC_GPS; out[3] = AOG_PGN_SCAN_REPLY; out[4] = 7;
        for (int i = 0; i < 4; i++) out[5 + i] = ip[i];
        for (int i = 0; i < 3; i++) out[9 + i] = ip[i];
        n = 12;
    } else if (req == AOG_REQ_HELLO) {
        if (cap < 11) return 0;
        out[0] = 0x80; out[1] = 0x81; out[2] = AOG_SRC_GPS; out[3] = AOG_SRC_GPS; out[4] = 5;
        for (int i = 0; i < 5; i++) out[5 + i] = 0;
        n = 10;
    } else {
        return 0;
    }
    out[n] = aog_pgn_crc(out, n);
    return n + 1;
}
