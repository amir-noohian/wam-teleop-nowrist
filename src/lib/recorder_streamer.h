#ifndef RECORDER_STREAMER_H
#define RECORDER_STREAMER_H

#include <boost/asio.hpp>
#include <boost/optional.hpp>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>

// ---------------------------------------------------------
// PAYLOAD DEFINITION
// ---------------------------------------------------------
// #pragma pack ensures 1-byte alignment so the memory footprint
// perfectly matches across C++, network boundaries, and Python.
#pragma pack(push, 1)
template <size_t DOF> struct RecorderPayload {
    double joint_positions[DOF];
    double joint_velocities[DOF];
    double external_torques[DOF];
    double measured_torques[DOF];
    double gripper_position;
    uint64_t timestamp_us;
};
#pragma pack(pop)

// ---------------------------------------------------------
// TIME UTILITY
// ---------------------------------------------------------
inline uint64_t getCurrentTimeMicroseconds() {
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
}

// ---------------------------------------------------------
// STREAMER CLASS DECLARATION
// ---------------------------------------------------------
template <typename PayloadType> class RecorderStreamer {
  public:
    struct ReceivedData {
        PayloadType payload;
        std::chrono::steady_clock::time_point rx_time;
    };

    RecorderStreamer(const std::string &remote_host, int send_port, int recv_port = -1);
    ~RecorderStreamer();

    void stop();
    void send(const PayloadType &data);
    boost::optional<ReceivedData> getLatestReceived();

  private:
    void receiveLoop();
    void sendLoop();

    std::string remote_host;
    int send_port;
    int recv_port;
    bool stop_threads;
    bool new_data_available;

    boost::asio::io_context io_context;
    boost::asio::ip::udp::socket send_socket;
    boost::asio::ip::udp::socket recv_socket;

    std::thread recv_thread;
    std::thread send_thread;

    std::mutex send_mutex;
    std::mutex state_mutex;
    std::condition_variable send_condition;

    PayloadType pending_send_data;
    boost::optional<ReceivedData> latest_received;
};

#endif // RECORDER_STREAMER_H