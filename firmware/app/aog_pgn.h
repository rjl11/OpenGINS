/*
 * AgOpenGPS / AgOpenWeb module PGNs that a GPS module answers on UDP 8888
 * (plans/ethernet-w5500-agopenweb.md, "Discovery"; layouts from AgOpenWeb's
 * Docs/PGN.md, and as the AiO v26 Teensy firmware's GPS module answers).
 *
 * Frame: 0x80 0x81, source, PGN, length, data[length], CRC. The CRC is the
 * low byte of the sum of the bytes from the source to the last data byte.
 * Our replies carry it; on the two requests it is not checked (aog_pgn.c).
 *
 *  - Scan request, PGN 202 (0xCA), data 202 202 5: answered with PGN 203
 *    (0xCB) from source 0x78 (the GPS module), data = our four IP bytes and
 *    the first three of them again as the subnet.
 *  - Hello, PGN 200 (0xC8): answered with source 0x78, PGN 0x78, five zero
 *    data bytes.
 *
 * No hardware calls: host-tested in firmware/tools/test_aog_pgn.c.
 */
#ifndef AOG_PGN_H
#define AOG_PGN_H

#include <stddef.h>
#include <stdint.h>

#define AOG_PGN_HELLO       200u
#define AOG_PGN_SCAN        202u
#define AOG_PGN_SCAN_REPLY  203u
#define AOG_SRC_GPS         0x78u
#define AOG_PGN_MAX         64u

typedef enum {
    AOG_REQ_NONE = 0, /* not a PGN, bad CRC, or one we don't answer */
    AOG_REQ_HELLO,
    AOG_REQ_SCAN,
} aog_req_t;

uint8_t aog_pgn_crc(const uint8_t *frame, size_t len_without_crc);

/* What a received datagram asks for. */
aog_req_t aog_pgn_parse(const uint8_t *b, size_t n);

/* The reply to req into out; returns its length, 0 for none. */
size_t aog_pgn_reply(aog_req_t req, const uint8_t ip[4], uint8_t *out, size_t cap);

#endif /* AOG_PGN_H */
