#include "serial.hpp"

#include <cerrno>
#include <cstring>
#include <stdexcept>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

namespace flow {

static speed_t to_speed(unsigned baud)
{
    switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default: throw std::runtime_error("unsupported baud rate " + std::to_string(baud));
    }
}

SerialPort::~SerialPort()
{
    if (fd_ >= 0) ::close(fd_);
}

void SerialPort::open(const std::string& path, unsigned baud)
{
    const speed_t sp = to_speed(baud);
    fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_CLOEXEC);
    if (fd_ < 0) throw std::runtime_error("can't open " + path + ": " + std::strerror(errno));
    termios t{};
    if (tcgetattr(fd_, &t) != 0) throw std::runtime_error("tcgetattr " + path + ": " + std::strerror(errno));
    cfmakeraw(&t);
    t.c_cflag |= CLOCAL | CREAD;
    t.c_cflag &= ~static_cast<tcflag_t>(CSTOPB | CRTSCTS);
    cfsetispeed(&t, sp);
    cfsetospeed(&t, sp);
    if (tcsetattr(fd_, TCSANOW, &t) != 0) throw std::runtime_error("tcsetattr " + path + ": " + std::strerror(errno));
}

bool SerialPort::write_all(const uint8_t* p, size_t n)
{
    while (n > 0) {
        const ssize_t k = ::write(fd_, p, n);
        if (k < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        p += k;
        n -= static_cast<size_t>(k);
    }
    return true;
}

long SerialPort::read_some(uint8_t* p, size_t cap, int timeout_ms)
{
    pollfd pf{fd_, POLLIN, 0};
    const int r = ::poll(&pf, 1, timeout_ms);
    if (r < 0) return errno == EINTR ? 0 : -1;
    if (r == 0) return 0;
    if (pf.revents & (POLLERR | POLLHUP | POLLNVAL)) return -1;
    const ssize_t k = ::read(fd_, p, cap);
    if (k < 0) return (errno == EINTR || errno == EAGAIN) ? 0 : -1;
    return k == 0 ? -1 : static_cast<long>(k);
}

}  // namespace flow
