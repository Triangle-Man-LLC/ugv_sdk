/*
 * async_can.cpp
 *
 * Created on: Sep 10, 2020 13:23
 * Description:
 *
 * Copyright (c) 2020 Weston Robot Pte. Ltd.
 * Copyright (c) 2026 Triangle Man LLC
 * Rewritten to remove ASIO and use epoll + raw SocketCAN
 */

#include "ugv_sdk/details/async_port/async_can.hpp"

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/epoll.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

namespace westonrobot {

AsyncCAN::AsyncCAN(std::string can_port)
    : port_(std::move(can_port)),
      can_fd_(-1),
      epoll_fd_(-1),
      running_(false) {}

AsyncCAN::~AsyncCAN() {
    Close();
}

bool AsyncCAN::Open() {
    if (port_opened_) return true;

    // Create CAN socket
    can_fd_ = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
    if (can_fd_ < 0) {
        std::cerr << "Failed to create CAN socket\n";
        return false;
    }

    // Resolve interface index
    struct ifreq ifr {};
    std::strncpy(ifr.ifr_name, port_.c_str(), IFNAMSIZ);
    if (::ioctl(can_fd_, SIOCGIFINDEX, &ifr) < 0) {
        std::cerr << "Failed to get CAN interface index\n";
        ::close(can_fd_);
        can_fd_ = -1;
        return false;
    }

    // Bind to CAN interface
    struct sockaddr_can addr {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (::bind(can_fd_, reinterpret_cast<struct sockaddr*>(&addr),
               sizeof(addr)) < 0) {
        std::cerr << "Failed to bind CAN socket\n";
        ::close(can_fd_);
        can_fd_ = -1;
        return false;
    }

    // Create epoll instance
    epoll_fd_ = ::epoll_create1(0);
    if (epoll_fd_ < 0) {
        std::cerr << "Failed to create epoll\n";
        ::close(can_fd_);
        can_fd_ = -1;
        return false;
    }

    // Register CAN FD for EPOLLIN
    struct epoll_event ev {};
    ev.events = EPOLLIN;
    ev.data.fd = can_fd_;

    if (::epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, can_fd_, &ev) < 0) {
        std::cerr << "Failed to add CAN FD to epoll\n";
        ::close(epoll_fd_);
        epoll_fd_ = -1;
        ::close(can_fd_);
        can_fd_ = -1;
        return false;
    }

    running_ = true;
    port_opened_ = true;

    // Start background thread
    io_thread_ = std::thread(&AsyncCAN::IoThreadFunc, this);

    std::cout << "AsyncCAN listening on " << port_ << "\n";
    return true;
}

void AsyncCAN::Close() {
    running_ = false;

    if (io_thread_.joinable())
        io_thread_.join();

    if (epoll_fd_ >= 0) {
        ::close(epoll_fd_);
        epoll_fd_ = -1;
    }

    if (can_fd_ >= 0) {
        ::close(can_fd_);
        can_fd_ = -1;
    }

    port_opened_ = false;
}

bool AsyncCAN::IsOpened() const {
    return port_opened_;
}

void AsyncCAN::DefaultReceiveCallback(can_frame *rx_frame) {
    std::cout << std::hex << rx_frame->can_id << "  ";
    for (int i = 0; i < rx_frame->can_dlc; i++)
        std::cout << int(rx_frame->data[i]) << " ";
    std::cout << std::dec << "\n";
}

void AsyncCAN::IoThreadFunc() {
    struct epoll_event events[4];

    while (running_) {
        int n = ::epoll_wait(epoll_fd_, events, 4, 100); // 100ms timeout

        if (n <= 0)
            continue;

        for (int i = 0; i < n; i++) {
            if (events[i].data.fd == can_fd_ &&
                (events[i].events & EPOLLIN)) {

                ssize_t bytes = ::read(can_fd_, &rcv_frame_, sizeof(rcv_frame_));
                if (bytes == sizeof(rcv_frame_)) {
                    if (rcv_cb_)
                        rcv_cb_(&rcv_frame_);
                    else
                        DefaultReceiveCallback(&rcv_frame_);
                }
            }
        }
    }
}

void AsyncCAN::SendFrame(const struct can_frame &frame) {
    if (!port_opened_) return;

    ssize_t written = ::write(can_fd_, &frame, sizeof(frame));
    if (written < 0) {
        std::cerr << "Failed to send CAN frame\n";
    }
}

} // namespace westonrobot
