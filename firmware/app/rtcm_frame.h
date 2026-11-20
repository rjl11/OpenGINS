/*
 * RTCM 3 framer: whole CRC-checked frames out of a byte stream that arrives
 * in arbitrary chunks (AgOpenWeb's UDP datagrams are 256-byte pieces of the
 * caster stream, so a frame can span datagrams; plans/rtcm-over-bluetooth.md,
 * plans/ethernet-w5500-agopenweb.md, "RTCM over Ethernet").
 *
 * Frame: 0xD3, 6 reserved bits (0) and a 10-bit length, the message, then a
 * 24-bit CRC-24Q over everything before it. On a bad CRC the search restarts
 * one byte after the 0xD3, so a lost chunk costs only the frames it touched.
 *
 * No hardware calls: host-tested in firmware/tools/test_rtcm_frame.c.
 */
#ifndef RTCM_FRAME_H
#define RTCM_FRAME_H

#include <stddef.h>
#include <stdint.h>

#define RTCM_FRAME_MAX (3u + 1023u + 3u)

typedef struct {
    uint8_t  buf[RTCM_FRAME_MAX];
    size_t   n;
    uint32_t frames;     /* good frames delivered                        */
    uint32_t crc_errors;
    uint32_t skipped;    /* bytes dropped while searching for a frame    */
} rtcm_frame_t;

/* Called for every good frame, with the whole frame (header to CRC). */
typedef void (*rtcm_frame_cb)(void *ctx, const uint8_t *frame, size_t len);

void rtcm_frame_init(rtcm_frame_t *r);
void rtcm_frame_push(rtcm_frame_t *r, const uint8_t *data, size_t len, rtcm_frame_cb cb, void *ctx);

/* CRC-24Q (RTCM 10403, Qualcomm polynomial 0x1864CFB). */
uint32_t rtcm_crc24q(const uint8_t *p, size_t n);

/* Message number of a good frame (first 12 bits of the message), 0 if too short. */
unsigned rtcm_frame_msg_type(const uint8_t *frame, size_t len);

#endif /* RTCM_FRAME_H */
