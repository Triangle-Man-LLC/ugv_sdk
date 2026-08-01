#include "ugv_sdk/details/async_port/async_serial.hpp"

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/epoll.h>
#include <sys/ioctl.h>
#include <cstring>
#include <iostream>

namespace westonrobot {

AsyncSerial::AsyncSerial(std::string port_name, uint32_t baud_rate)
    : port_(std::move(port_name)),
      baud_rate_(baud_rate),
      hwflow_(false),
      serial_fd_(-1),
      epoll_fd_(-1),
      running_(false),
      tx_in_progress_(false) {}

AsyncSerial::~AsyncSerial() {
    Close();
}

void AsyncSerial::SetBaudRate(unsigned baudrate) {
    baud_rate_ = baudrate;
}

void AsyncSerial::SetHardwareFlowControl(bool enabled) {
    hwflow_ = enabled;
}

bool AsyncSerial::Open() {
    if (port_opened_) return true;

    // open serial port
    serial_fd_ = ::open(port_.c_str(), O_RDWR | O_NONBLOCK | O_NOCTTY);
    if (serial_fd_ < 0) {
        std::cerr << "Failed to open serial port\n";
        return false;
    }

    // configure termios
    struct termios tio {};
    tcgetattr(serial_fd_, &tio);

    cfmakeraw(&tio);

    speed_t speed = B115200;
    switch (baud_rate_) {
        case 9600: speed = B9600; break;
        case 19200: speed = B19200; break;
        case 38400: speed = B38400; break;
        case 57600: speed = B57600; break;
        case 115200: speed = B115200; break;
        case 230400: speed = B230400; break;
        default: speed = B115200; break;
    }

    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);

    if (hwflow_) tio.c_cflag |= CRTSCTS;
    else tio.c_cflag &= ~CRTSCTS;

    tcsetattr(serial_fd_, TCSANOW, &tio);

#if defined(__linux__)
    // enable low latency mode
    struct serial_struct ser {};
    if (ioctl(serial_fd_, TIOCGSERIAL, &ser) == 0) {
        ser.flags |= ASYNC_LOW_LATENCY;
        ioctl(serial_fd_, TIOCSSERIAL, &ser);
    }
#endif

    // create epoll
    epoll_fd_ = epoll_create1(0);
    if (epoll_fd_ < 0) {
        std::cerr << "Failed to create epoll\n";
        ::close(serial_fd_);
        serial_fd_ = -1;
        return false;
    }

    struct epoll_event ev {};
    ev.events = EPOLLIN;
    ev.data.fd = serial_fd_;
    epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, serial_fd_, &ev);

    running_ = true;
    port_opened_ = true;

    io_thread_ = std::thread(&AsyncSerial::IoThreadFunc, this);

    std::cout << "AsyncSerial listening on " << port_ << "@" << baud_rate_ << "\n";
    return true;
}

void AsyncSerial::Close() {
    running_ = false;

    if (io_thread_.joinable())
        io_thread_.join();

    if (epoll_fd_ >= 0) {
        ::close(epoll_fd_);
        epoll_fd_ = -1;
    }

    if (serial_fd_ >= 0) {
        ::close(serial_fd_);
        serial_fd_ = -1;
    }

    port_opened_ = false;
}

bool AsyncSerial::IsOpened() const {
    return port_opened_;
}

void AsyncSerial::DefaultReceiveCallback(uint8_t *data, const size_t bufsize, size_t len) {
    // default: do nothing
}

void AsyncSerial::IoThreadFunc() {
    struct epoll_event events[4];

    while (running_) {
        int n = epoll_wait(epoll_fd_, events, 4, 50);

        if (n <= 0) {
            WriteToPort(true);
            continue;
        }

        for (int i = 0; i < n; i++) {
            if (events[i].data.fd == serial_fd_ &&
                (events[i].events & EPOLLIN)) {
                ReadFromPort();
            }
        }

        WriteToPort(true);
    }
}

void AsyncSerial::ReadFromPort() {
    ssize_t len = ::read(serial_fd_, rx_buf_.data(), rx_buf_.size());
    if (len > 0) {
        if (rcv_cb_)
            rcv_cb_(rx_buf_.data(), rx_buf_.size(), len);
        else
            DefaultReceiveCallback(rx_buf_.data(), rx_buf_.size(), len);
    }
}

void AsyncSerial::WriteToPort(bool check_if_busy) {
    if (check_if_busy && tx_in_progress_)
        return;

    std::lock_guard<std::recursive_mutex> lock(tx_mutex_);

    if (tx_rbuf_.IsEmpty()) {
        tx_in_progress_ = false;
        return;
    }

    size_t count = tx_rbuf_.Read(tx_buf_, tx_rbuf_.GetOccupiedSize());

    ssize_t written = ::write(serial_fd_, tx_buf_, count);
    if (written < 0) {
        tx_in_progress_ = false;
        return;
    }

    tx_in_progress_ = (written < (ssize_t)count);
}

void AsyncSerial::SendBytes(const uint8_t *bytes, size_t length) {
    if (!IsOpened()) {
        std::cerr << "Port closed, cannot send\n";
        return;
    }

    if (length >= rxtx_buffer_size) {
        throw std::length_error("SendBytes: length exceeds buffer size");
    }

    std::lock_guard<std::recursive_mutex> lock(tx_mutex_);

    if (tx_rbuf_.GetFreeSize() < length) {
        throw std::length_error("SendBytes: TX buffer overflow");
    }

    tx_rbuf_.Write(bytes, length);
}

} // namespace westonrobot
