// Raw serial port for the link to the Pico (8N1, no flow control).
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace flow {

class SerialPort {
public:
    SerialPort() = default;
    ~SerialPort();
    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    // Throws std::runtime_error on failure (unknown baud, can't open).
    void open(const std::string& path, unsigned baud);
    // Writes everything; returns false on an error (the caller counts it).
    bool write_all(const uint8_t* p, size_t n);
    // Waits up to timeout_ms for input; returns the bytes read, 0 on a
    // timeout, -1 on an error (the port went away).
    long read_some(uint8_t* p, size_t cap, int timeout_ms);

private:
    int fd_ = -1;
};

}  // namespace flow
