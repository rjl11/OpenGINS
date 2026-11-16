/*
 * A small record kept in two 4 KB flash sectors, written alternately, so a
 * power cut during a save leaves the previous record (plans/web-config-server.md,
 * "Storing configuration"). Used by the settings and by the IMU calibration.
 *
 * On flash, each sector holds one record at its start: a 16-byte header
 * (magic, version, payload size, sequence number, CRC-32 of the payload),
 * then the payload, over as many 256-byte pages as it needs (at most
 * FLASH_REC_MAX_BYTES in all). The valid record with the higher sequence number wins; a
 * record with another magic, version or size, or a bad CRC, is ignored.
 *
 * Saving pauses core 1 (flash lockout, core 1 must have called
 * flash_safe_execute_core_init()) and core 0's interrupts for one sector
 * erase and the program of the record's pages.
 *
 * Core 0 only.
 */
#ifndef FLASH_REC_H
#define FLASH_REC_H

#include <stdbool.h>
#include <stdint.h>

#define FLASH_REC_HEADER_BYTES  16u
#define FLASH_REC_MAX_BYTES     4096u /* header + payload: up to one 4 KB sector */
#define FLASH_REC_MAX_PAYLOAD   (FLASH_REC_MAX_BYTES - FLASH_REC_HEADER_BYTES)

typedef struct {
    uint32_t sector_a;     /* flash offset of the first sector */
    uint32_t sector_b;     /* flash offset of the second sector */
    uint32_t magic;
    uint16_t version;
    uint16_t size;         /* payload bytes */
    uint32_t seq;          /* record in use, 0 = none */
    uint32_t last_sector;  /* offset of the record in use, 0 = none */
} flash_rec_t;

/* first_sector: flash offset of the first of two consecutive 4 KB sectors. */
void flash_rec_init(flash_rec_t *r, uint32_t first_sector, uint32_t magic, uint16_t version,
                    uint16_t size);

/* Copies the newest valid payload into payload. Returns false (payload
 * untouched) if there is none. */
bool flash_rec_load(flash_rec_t *r, void *payload);

/* The same for a record of an older version and size (a migration): copies
 * size bytes. A later flash_rec_save() writes the current version. */
bool flash_rec_load_as(flash_rec_t *r, uint16_t version, uint16_t size, void *payload);

/* Writes a new record into the older sector and reads it back. Returns 0 or
 * a negative error. */
int flash_rec_save(flash_rec_t *r, const void *payload);

/* Erases both sectors. */
int flash_rec_erase(flash_rec_t *r);

static inline uint32_t flash_rec_seq(const flash_rec_t *r) { return r->seq; }

#endif /* FLASH_REC_H */
