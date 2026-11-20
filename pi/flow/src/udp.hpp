// UDP link to the Pico's W5500 (front camera unit,
// plans/front-camera-unit.md): one flow_link frame per datagram, and the
// Pico's clock sync requests coming back on the same socket.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace flow {

class UdpSender {
public:
    UdpSender() = default;
    ~UdpSender();
    UdpSender(const UdpSender&) = delete;
    UdpSender& operator=(const UdpSender&) = delete;

    // "HOST" or "HOST:PORT" (IPv4 address or name); default_port when no
    // port is given. Throws std::runtime_error on failure.
    void open(const std::string& dest, uint16_t default_port);
    // One datagram; returns false on an error (the caller counts it).
    // Safe to call from two threads (one datagram is one syscall).
    bool send(const uint8_t* p, size_t n);
    // Waits up to timeout_ms for a datagram from the Pico; returns its
    // length, 0 on a timeout or a refused port, -1 on another error.
    // rx_mono_ns, if not null, gets the kernel's receive time on
    // CLOCK_MONOTONIC (from SO_TIMESTAMPNS, which is CLOCK_REALTIME,
    // moved over by the two clocks' difference read right after), so the
    // wake-up delay of the calling thread is not in it; 0 if the kernel
    // gave none.
    long recv(uint8_t* p, size_t cap, int timeout_ms, int64_t* rx_mono_ns = nullptr);

private:
    int fd_ = -1;
};

}  // namespace flow
