#include "recorder_streamer.h"
#include <boost/system/error_code.hpp>
#include <iostream>

template <typename PayloadType>
RecorderStreamer<PayloadType>::RecorderStreamer(const std::string &remote_host, int send_port, int recv_port)
    : remote_host(remote_host), send_port(send_port), recv_port(recv_port), stop_threads(false),
      new_data_available(false), 
      send_socket(io_context, boost::asio::ip::udp::v4()),
      recv_socket(io_context) // Initialize but don't bind yet
{
    // 1. Always start the sender
    send_thread = std::thread(&RecorderStreamer::sendLoop, this);

    // 2. Conditionally start the receiver
    if (recv_port != -1) {
        boost::system::error_code ec;
        recv_socket.open(boost::asio::ip::udp::v4(), ec);
        recv_socket.bind(boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), recv_port), ec);
        
        if (ec) {
            // Handle bind error (e.g., port already in use)
            std::cerr << "Failed to bind recv_port " << recv_port << ": " << ec.message() << std::endl;
        } else {
            recv_thread = std::thread(&RecorderStreamer::receiveLoop, this);
        }
    }
}

template <typename PayloadType> RecorderStreamer<PayloadType>::~RecorderStreamer() { stop(); }

template <typename PayloadType> void RecorderStreamer<PayloadType>::stop() {
    stop_threads = true;
    io_context.stop();
    send_condition.notify_all();

    try {
        recv_socket.cancel();
        recv_socket.shutdown(boost::asio::ip::udp::socket::shutdown_both);
        recv_socket.close();
    } catch (...) {
        // Ignore exceptions during socket cleanup
    }

    if (recv_thread.joinable())
        recv_thread.join();
    if (send_thread.joinable())
        send_thread.join();
}

template <typename PayloadType>
boost::optional<typename RecorderStreamer<PayloadType>::ReceivedData>
RecorderStreamer<PayloadType>::getLatestReceived() {
    std::lock_guard<std::mutex> lock(state_mutex);
    return latest_received;
}

template <typename PayloadType> void RecorderStreamer<PayloadType>::send(const PayloadType &data) {
    {
        std::lock_guard<std::mutex> lock(send_mutex);
        pending_send_data = data;
        new_data_available = true;
    }
    send_condition.notify_one();
}

template <typename PayloadType> void RecorderStreamer<PayloadType>::receiveLoop() {
    boost::asio::ip::udp::endpoint sender_endpoint;
    char buffer[sizeof(PayloadType)];

    while (!stop_threads) {
        boost::system::error_code ec;
        size_t len = recv_socket.receive_from(boost::asio::buffer(buffer, sizeof(buffer)), sender_endpoint, 0, ec);

        // If the socket was closed or we received a malformed packet, ignore it
        if (ec == boost::asio::error::operation_aborted || len != sizeof(PayloadType))
            continue;

        PayloadType received_payload;
        std::memcpy(&received_payload, buffer, sizeof(PayloadType));

        {
            std::lock_guard<std::mutex> lock(state_mutex);
            latest_received = ReceivedData{received_payload, std::chrono::steady_clock::now()};
        }
    }

    try {
        if (recv_socket.is_open())
            recv_socket.close();
    } catch (...) {
    }
}

template <typename PayloadType> void RecorderStreamer<PayloadType>::sendLoop() {
    boost::asio::ip::udp::endpoint remote_endpoint(boost::asio::ip::make_address(remote_host), send_port);

    while (!stop_threads) {
        std::unique_lock<std::mutex> lock(send_mutex);
        send_condition.wait(lock, [this] { return new_data_available || stop_threads; });

        if (stop_threads)
            break;

        PayloadType data_to_send = pending_send_data;
        new_data_available = false;
        lock.unlock();

        char buffer[sizeof(PayloadType)];
        std::memcpy(buffer, &data_to_send, sizeof(PayloadType));

        boost::system::error_code ec;
        send_socket.send_to(boost::asio::buffer(buffer, sizeof(buffer)), remote_endpoint, 0, ec);
    }

    try {
        if (send_socket.is_open())
            send_socket.close();
    } catch (...) {
    }
}

// ---------------------------------------------------------
// EXPLICIT TEMPLATE INSTANTIATIONS
// ---------------------------------------------------------
// Add any other DOF configurations you plan to record here
template class RecorderStreamer<RecorderPayload<7>>;