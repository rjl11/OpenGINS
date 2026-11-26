/*
 * Web config API (plans/web-config-server.md, Phase 4).
 *
 * GET  /config.json  the settings in use, the build defaults, the float
 *                    fields' limits (range: key -> [lo, hi]), which fields
 *                    wait for a reboot, and the result of the last command.
 * POST /config       form-encoded (application/x-www-form-urlencoded):
 *                    action=save|defaults|reboot|reconfigure_gnss|reconfigure_imu|
 *                    calib_reset (IMU calibration back to factory, every
 *                    group, at once), calib (with group=gyr|acc|mag|gyr_m|all
 *                    and op=accept|hold|auto|clear|factory, imu_calib_action();
 *                    all only with factory),
 *                    and for save the fields named in config.html. A field
 *                    left out keeps its current value; sta_password is only
 *                    changed when sent. Optional rid=<1..2^32-1>: a POST
 *                    with the rid and the same fields (in any order) as one
 *                    of the last 8 accepted (kept through a reboot once it
 *                    has run) is not queued again but answered
 *                    {"ok":true,"repeat":"pending"|"done"}, with
 *                    "result_seq" (its last_result seq) when it was done in
 *                    this boot: a retry after a reply that never reached
 *                    the page (common.js). last_result.rid in /config.json
 *                    names the request a result belongs to.
 *
 * One command at a time: a POST is refused as busy from its acceptance until
 * the main loop has reported its result, so a save always starts from the
 * settings the one before it left.
 *
 * The POST handler runs in lwIP context: it parses and range-checks a save
 * and queues one command. The main loop takes it with config_web_take(),
 * writes flash, applies it, and reports through config_web_publish().
 */
#ifndef CONFIG_WEB_H
#define CONFIG_WEB_H

#include <stdbool.h>
#include <stdint.h>

#include "settings.h"

typedef enum {
    CFG_CMD_NONE = 0,
    CFG_CMD_SAVE,
    CFG_CMD_DEFAULTS,
    CFG_CMD_REBOOT,
    CFG_CMD_RECONFIGURE_GNSS,
    CFG_CMD_RECONFIGURE_IMU,
    CFG_CMD_CALIB_RESET,
    CFG_CMD_CALIB,
} config_cmd_kind_t;

typedef struct {
    config_cmd_kind_t kind;
    settings_t settings; /* CFG_CMD_SAVE */
    int calib_group;     /* CFG_CMD_CALIB: IMU_CALIB_* group and op */
    int calib_op;
} config_cmd_t;

/* Registers the POST handler with the web server. */
void config_web_init(void);

/* Main loop: the queued command, if any. */
bool config_web_take(config_cmd_t *cmd);

/* Main loop: rebuild /config.json. cur = settings in use, boot = what this
 * boot started with, result = text of the last command's outcome (NULL keeps
 * the previous one, and its counter). */
void config_web_publish(const settings_t *cur, const settings_t *boot, bool from_flash,
                        const char *result);

#endif /* CONFIG_WEB_H */
