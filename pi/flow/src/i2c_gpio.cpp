#include "i2c_gpio.hpp"

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <vector>

#include <fcntl.h>
#include <linux/gpio.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace flow {

I2cDevice::~I2cDevice()
{
    if (fd_ >= 0) ::close(fd_);
}

void I2cDevice::open(const std::string& path, uint8_t addr)
{
    fd_ = ::open(path.c_str(), O_RDWR | O_CLOEXEC);
    if (fd_ < 0) throw std::runtime_error("can't open " + path + ": " + std::strerror(errno));
    addr_ = addr;
}

bool I2cDevice::read(uint8_t reg, uint8_t* p, size_t n)
{
    i2c_msg msgs[2] = {};
    msgs[0].addr = addr_;
    msgs[0].flags = 0;
    msgs[0].len = 1;
    msgs[0].buf = &reg;
    msgs[1].addr = addr_;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = static_cast<uint16_t>(n);
    msgs[1].buf = p;
    i2c_rdwr_ioctl_data x{msgs, 2};
    return ::ioctl(fd_, I2C_RDWR, &x) == 2;
}

bool I2cDevice::write(uint8_t reg, const uint8_t* p, size_t n)
{
    std::vector<uint8_t> b(n + 1);
    b[0] = reg;
    std::memcpy(b.data() + 1, p, n);
    i2c_msg msg{};
    msg.addr = addr_;
    msg.flags = 0;
    msg.len = static_cast<uint16_t>(b.size());
    msg.buf = b.data();
    i2c_rdwr_ioctl_data x{&msg, 1};
    return ::ioctl(fd_, I2C_RDWR, &x) == 1;
}

GpioEdge::~GpioEdge()
{
    if (fd_ >= 0) ::close(fd_);
}

void GpioEdge::open(const std::string& chip_path, unsigned line, const char* consumer)
{
    const int chip = ::open(chip_path.c_str(), O_RDWR | O_CLOEXEC);
    if (chip < 0) throw std::runtime_error("can't open " + chip_path + ": " + std::strerror(errno));
    gpio_v2_line_request req{};
    req.offsets[0] = line;
    req.num_lines = 1;
    std::strncpy(req.consumer, consumer, sizeof req.consumer - 1);
    // Pulled down, so a wire that came off reads low instead of floating
    // into false edges.
    req.config.flags = GPIO_V2_LINE_FLAG_INPUT | GPIO_V2_LINE_FLAG_EDGE_RISING | GPIO_V2_LINE_FLAG_BIAS_PULL_DOWN;
    req.event_buffer_size = 16;
    const int r = ::ioctl(chip, GPIO_V2_GET_LINE_IOCTL, &req);
    const int e = errno;
    ::close(chip);
    if (r < 0)
        throw std::runtime_error("can't request line " + std::to_string(line) + " of " + chip_path + ": " +
                                 std::strerror(e));
    fd_ = req.fd;
}

int GpioEdge::wait(int timeout_ms, int64_t& t_last_ns)
{
    pollfd pf{fd_, POLLIN, 0};
    const int r = ::poll(&pf, 1, timeout_ms);
    if (r < 0) return errno == EINTR ? 0 : -1;
    if (r == 0) return 0;
    if (pf.revents & (POLLERR | POLLHUP | POLLNVAL)) return -1;
    gpio_v2_line_event ev[16];
    const ssize_t      n = ::read(fd_, ev, sizeof ev);
    if (n < static_cast<ssize_t>(sizeof ev[0])) return n < 0 && errno == EAGAIN ? 0 : -1;
    const int k = static_cast<int>(static_cast<size_t>(n) / sizeof ev[0]);
    t_last_ns = static_cast<int64_t>(ev[k - 1].timestamp_ns);
    return k;
}

}  // namespace flow
