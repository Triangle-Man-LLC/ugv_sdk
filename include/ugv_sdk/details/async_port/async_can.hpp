#ifndef ASYNC_CAN_HPP
#define ASYNC_CAN_HPP

#include <linux/can.h>

#include <atomic>
#include <memory>
#include <thread>
#include <functional>
#include <string>

namespace westonrobot {

class AsyncCAN : public std::enable_shared_from_this<AsyncCAN> {
public:
    using ReceiveCallback = std::function<void(can_frame *rx_frame)>;

    explicit AsyncCAN(std::string can_port = "can0");
    ~AsyncCAN();

    AsyncCAN(const AsyncCAN&) = delete;
    AsyncCAN& operator=(const AsyncCAN&) = delete;

    bool Open();
    void Close();
    bool IsOpened() const;

    void SetReceiveCallback(ReceiveCallback cb) { rcv_cb_ = cb; }
    void SendFrame(const struct can_frame &frame);

private:
    // configuration
    std::string port_;
    std::atomic<bool> port_opened_{false};

    // raw CAN socket
    int can_fd_{-1};

    // epoll for async I/O
    int epoll_fd_{-1};

    // background thread
    std::thread io_thread_;
    std::atomic<bool> running_{false};

    // receive buffer + callback
    struct can_frame rcv_frame_;
    ReceiveCallback rcv_cb_ = nullptr;

    // internal helpers
    void DefaultReceiveCallback(can_frame *rx_frame);
    void IoThreadFunc();
};

} // namespace westonrobot

#endif /* ASYNC_CAN_HPP */
