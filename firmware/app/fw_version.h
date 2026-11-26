/*
 * Firmware version, generated at build time from git by
 * tools/fw_version.cmake (the fw_version target in firmware/CMakeLists.txt).
 */
#ifndef FW_VERSION_H
#define FW_VERSION_H

/* git describe --always --dirty, e.g. "7e30719" or "7e30719-dirty";
 * "unknown" when built outside a git checkout. */
extern const char app_fw_version[];
/* Date of that commit, YYYY-MM-DD, or "" without git. */
extern const char app_fw_date[];

#endif /* FW_VERSION_H */
