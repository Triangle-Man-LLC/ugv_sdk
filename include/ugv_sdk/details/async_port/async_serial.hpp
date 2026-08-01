#ifndef ASYNC_SERIAL_HPP
#define ASYNC_SERIAL_HPP

#include <cstdint>
#include <memory>
#include <atomic> 
#include <array>
#include <mutex>
#include <thread>
#include <functional>
#include <string>
#include <linux/serial.h>

#include "ugv_sdk/details/async_port/ring_buffer.hpp"

namespace westonrobot {

class AsyncSerial : public std::enable_shared_from_this<AsyncSerial> {
public:
    using ReceiveCallback =
        std::function<void(uint8_t *data, const size_t bufsize, size_t len)>;

    explicit AsyncSerial(std::string port_name, uint32_t baud_rate = 115200);
    ~AsyncSerial();

    AsyncSerial(const AsyncSerial&) = delete;
    AsyncSerial& operator=(const AsyncSerial&) = delete;

    void SetBaudRate(unsigned baudrate);
    void SetHardwareFlowControl(bool enabled);

    bool Open();
    void Close();
    bool IsOpened() const;

    void SetReceiveCallback(ReceiveCallback cb) { rcv_cb_ = cb; }
    void SendBytes(const uint8_t *bytes, size_t length);

private:
    // config
    std::string port_;
    uint32_t baud_rate_;
    bool hwflow_;

    // state
    bool port_opened_{false};
    int serial_fd_{-1};
    int epoll_fd_{-1};

    std::thread io_thread_;
    std::atomic<bool> running_{false};

    // rx buffer
    static constexpr uint32_t rxtx_buffer_size = 1024 * 8;
    std::array<uint8_t, rxtx_buffer_size> rx_buf_;

    // tx buffer
    uint8_t tx_buf_[rxtx_buffer_size];
    RingBuffer<uint8_t, rxtx_buffer_size> tx_rbuf_;
    std::recursive_mutex tx_mutex_;
    bool tx_in_progress_{false};

    ReceiveCallback rcv_cb_ = nullptr;

    // internal helpers
    void DefaultReceiveCallback(uint8_t *data, const size_t bufsize, size_t len);
    void IoThreadFunc();
    void ReadFromPort();
    void WriteToPort(bool check_if_busy);
};

} // namespace westonrobot

#endif /* ASYNC_SERIAL_HPP */
