#include "udp.hpp"

#include <netdb.h>
#include <poll.h>
#include <sys/uio.h>
#include <time.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>

namespace flow {

UdpSender::~UdpSender()
{
    if (fd_ >= 0) ::close(fd_);
}

void UdpSender::open(const std::string& dest, uint16_t default_port)
{
    std::string host = dest;
    std::string port = std::to_string(default_port);
    const size_t colon = dest.rfind(':');
    if (colon != std::string::npos) {
        host = dest.substr(0, colon);
        port = dest.substr(colon + 1);
    }
    addrinfo hints{};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    addrinfo* res = nullptr;
    const int rc = ::getaddrinfo(host.c_str(), port.c_str(), &hints, &res);
    if (rc != 0) throw std::runtime_error("udp: " + dest + ": " + ::gai_strerror(rc));
    fd_ = ::socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (fd_ < 0) {
        ::freeaddrinfo(res);
        throw std::runtime_error(std::string("udp: socket: ") + std::strerror(errno));
    }
    const int one = 1;
    ::setsockopt(fd_, SOL_SOCKET, SO_BROADCAST, &one, sizeof one);
    ::setsockopt(fd_, SOL_SOCKET, SO_TIMESTAMPNS, &one, sizeof one);  // receive times for the clock sync
    // connect() fixes the destination, so send() needs no address.
    const int c = ::connect(fd_, res->ai_addr, res->ai_addrlen);
    ::freeaddrinfo(res);
    if (c != 0) throw std::runtime_error("udp: connect " + dest + ": " + std::strerror(errno));
}

bool UdpSender::send(const uint8_t* p, size_t n)
{
    // An ICMP port-unreachable from an earlier datagram shows as an error
    // here; the next send goes out regardless.
    return ::send(fd_, p, n, 0) == static_cast<ssize_t>(n);
}

static int64_t ns_of(const timespec& t) { return static_cast<int64_t>(t.tv_sec) * 1000000000 + t.tv_nsec; }

long UdpSender::recv(uint8_t* p, size_t cap, int timeout_ms, int64_t* rx_mono_ns)
{
    pollfd pfd{fd_, POLLIN, 0};
    const int r = ::poll(&pfd, 1, timeout_ms);
    if (r <= 0) return r == 0 || errno == EINTR ? 0 : -1;
    iovec   iov{p, cap};
    alignas(cmsghdr) char ctl[CMSG_SPACE(sizeof(timespec))];
    msghdr  mh{};
    mh.msg_iov = &iov;
    mh.msg_iovlen = 1;
    mh.msg_control = ctl;
    mh.msg_controllen = sizeof ctl;
    const ssize_t n = ::recvmsg(fd_, &mh, 0);
    if (n < 0) return errno == ECONNREFUSED || errno == EINTR ? 0 : -1;
    if (rx_mono_ns) {
        *rx_mono_ns = 0;
        for (cmsghdr* c = CMSG_FIRSTHDR(&mh); c; c = CMSG_NXTHDR(&mh, c)) {
            if (c->cmsg_level == SOL_SOCKET && c->cmsg_type == SCM_TIMESTAMPNS) {
                timespec k{}, rt{}, mono{};
                std::memcpy(&k, CMSG_DATA(c), sizeof k);
                ::clock_gettime(CLOCK_REALTIME, &rt);
                ::clock_gettime(CLOCK_MONOTONIC, &mono);
                const int64_t ago = ns_of(rt) - ns_of(k);  // kernel receive to now
                // An NTP step of the realtime clock in between would show as
                // a negative or absurd age: then use the time now.
                *rx_mono_ns = ns_of(mono) - (ago >= 0 && ago < 1000000000 ? ago : 0);
            }
        }
    }
    return n;
}

}  // namespace flow
