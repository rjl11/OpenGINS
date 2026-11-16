#include "flash_rec.h"

#include <string.h>

#include "hardware/flash.h"
#include "pico/flash.h"

#define FLASH_TIMEOUT_MS 200u

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;      /* payload bytes */
    uint32_t seq;
    uint32_t crc;       /* CRC-32 of the payload */
} header_t;

_Static_assert(sizeof(header_t) == FLASH_REC_HEADER_BYTES, "flash record header layout");
_Static_assert(FLASH_REC_MAX_BYTES % FLASH_PAGE_SIZE == 0 && FLASH_REC_MAX_BYTES <= FLASH_SECTOR_SIZE,
               "a flash record is whole pages inside one sector");

static uint32_t crc32(const void *data, size_t len)
{
    const uint8_t *p = data;
    uint32_t c = 0xFFFFFFFFu;
    while (len--) {
        c ^= *p++;
        for (int k = 0; k < 8; k++) {
            c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
        }
    }
    return ~c;
}

/* The header at offset if a valid record of this version and size is
 * there, else NULL. */
static const header_t *read_rec_as(const flash_rec_t *r, uint32_t offset, uint16_t version, uint16_t size)
{
    const header_t *h = (const header_t *)(XIP_BASE + offset);
    if (h->magic != r->magic || h->version != version || h->size != size || size > FLASH_REC_MAX_PAYLOAD) {
        return NULL;
    }
    if (crc32((const uint8_t *)h + FLASH_REC_HEADER_BYTES, size) != h->crc) {
        return NULL;
    }
    return h;
}

static const header_t *read_rec(const flash_rec_t *r, uint32_t offset)
{
    return read_rec_as(r, offset, r->version, r->size);
}

void flash_rec_init(flash_rec_t *r, uint32_t first_sector, uint32_t magic, uint16_t version,
                    uint16_t size)
{
    memset(r, 0, sizeof *r);
    r->sector_a = first_sector;
    r->sector_b = first_sector + FLASH_SECTOR_SIZE;
    r->magic = magic;
    r->version = version;
    r->size = size <= FLASH_REC_MAX_PAYLOAD ? size : (uint16_t)FLASH_REC_MAX_PAYLOAD;
}

bool flash_rec_load_as(flash_rec_t *r, uint16_t version, uint16_t size, void *payload)
{
    const header_t *a = read_rec_as(r, r->sector_a, version, size);
    const header_t *b = read_rec_as(r, r->sector_b, version, size);
    const header_t *h = a;
    if (b && (!a || b->seq > a->seq)) {
        h = b;
    }
    if (!h) {
        r->seq = 0;
        r->last_sector = 0;
        return false;
    }
    memcpy(payload, (const uint8_t *)h + FLASH_REC_HEADER_BYTES, size);
    /* The next save goes into the other sector with a higher number, so an
     * older-version record stays as the fallback until then. */
    r->seq = h->seq;
    r->last_sector = h == a ? r->sector_a : r->sector_b;
    return true;
}

bool flash_rec_load(flash_rec_t *r, void *payload)
{
    return flash_rec_load_as(r, r->version, r->size, payload);
}

typedef struct {
    uint32_t offset;
    const uint8_t *page; /* NULL = erase only */
    uint32_t len;        /* bytes to program, whole pages */
} flash_job_t;

/* Runs with core 1 locked out and core 0's interrupts off. */
static void flash_job(void *param)
{
    const flash_job_t *j = param;
    flash_range_erase(j->offset, FLASH_SECTOR_SIZE);
    if (j->page) {
        flash_range_program(j->offset, j->page, j->len);
    }
}

int flash_rec_save(flash_rec_t *r, const void *payload)
{
    static uint8_t page[FLASH_REC_MAX_BYTES];
    header_t h = {
        .magic = r->magic,
        .version = r->version,
        .size = r->size,
        .seq = r->seq + 1,
        .crc = crc32(payload, r->size),
    };
    const uint32_t used = FLASH_REC_HEADER_BYTES + r->size;
    const uint32_t len = (used + FLASH_PAGE_SIZE - 1u) / FLASH_PAGE_SIZE * FLASH_PAGE_SIZE;
    memset(page, 0xFF, len);
    memcpy(page, &h, sizeof h);
    memcpy(page + FLASH_REC_HEADER_BYTES, payload, r->size);

    flash_job_t job = { .offset = r->last_sector == r->sector_a ? r->sector_b : r->sector_a,
                        .page = page, .len = len };
    int rc = flash_safe_execute(flash_job, &job, FLASH_TIMEOUT_MS);
    if (rc != 0) {
        return rc;
    }
    const header_t *back = read_rec(r, job.offset);
    if (!back || back->seq != h.seq) {
        return -1; /* read-back failed */
    }
    r->seq = h.seq;
    r->last_sector = job.offset;
    return 0;
}

int flash_rec_erase(flash_rec_t *r)
{
    flash_job_t job = { .offset = r->sector_a, .page = NULL, .len = 0 };
    int rc = flash_safe_execute(flash_job, &job, FLASH_TIMEOUT_MS);
    if (rc == 0) {
        job.offset = r->sector_b;
        rc = flash_safe_execute(flash_job, &job, FLASH_TIMEOUT_MS);
    }
    if (rc == 0) {
        r->seq = 0;
        r->last_sector = 0;
    }
    return rc;
}
