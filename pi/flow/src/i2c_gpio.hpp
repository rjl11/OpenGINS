// Linux I2C (/dev/i2c-N) register access and a GPIO input's rising edges
// (the GPIO character device, uAPI v2), for the TF-Luna in I2C mode.
//
// Edge times come from the kernel's interrupt handler on CLOCK_MONOTONIC
// (the uAPI v2 default), the clock the flow's frame times are on.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace flow {

class I2cDevice {
public:
    I2cDevice() = default;
    ~I2cDevice();
    I2cDevice(const I2cDevice&) = delete;
    I2cDevice& operator=(const I2cDevice&) = delete;

    void open(const std::string& path, uint8_t addr);  // throws std::runtime_error
    // Register reads and writes in one transfer each (a read is the register
    // address, a repeated start, then n bytes). false on a bus error (no
    // ACK, a timeout); errno is kept.
    bool read(uint8_t reg, uint8_t* p, size_t n);
    bool write(uint8_t reg, const uint8_t* p, size_t n);

private:
    int     fd_ = -1;
    uint8_t addr_ = 0;
};

class GpioEdge {
public:
    GpioEdge() = default;
    ~GpioEdge();
    GpioEdge(const GpioEdge&) = delete;
    GpioEdge& operator=(const GpioEdge&) = delete;

    // Requests the line as an input with rising-edge events. Throws.
    void open(const std::string& chip_path, unsigned line, const char* consumer);
    // Waits up to timeout_ms for edges and takes all that are queued: the
    // count, and the newest one's time. 0 on timeout, -1 on an error.
    int wait(int timeout_ms, int64_t& t_last_ns);

private:
    int fd_ = -1;  // the line request
};

}  // namespace flow
