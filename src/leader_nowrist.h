#pragma once

#include <boost/asio.hpp>

#include "udp_handler.h"
#include <barrett/detail/ca_macro.h>
#include <barrett/systems/abstract/single_io.h>
#include <barrett/thread/abstract/mutex.h>
#include <barrett/units.h>

template <size_t DOF>
class Leader : public barrett::systems::System {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  public:
    Input<jp_type> wamJPIn;
    Input<jv_type> wamJVIn;
    Output<jt_type> wamJPOutput;

    enum class State { INIT, LINKED, UNLINKED };

    explicit Leader(barrett::systems::ExecutionManager* em, char* remoteHost, int rec_port = 5555,
                      int send_port = 5554, const std::string& sysName = "Leader Nowrist")
        : System(sysName)
        , theirJp(0.0)
        , theirJv(0.0)
        , control(0.0)
        , wamJPIn(this)
        , wamJVIn(this)
        , wamJPOutput(this, &jtOutputValue)
        , udp_handler(remoteHost, send_port, rec_port)
        , state(State::INIT) {

        kp << 600, 700, 250, 120;
        kd << 30, 25, 15, 10;

        if (em != NULL) {
            em->startManaging(*this);
        }
    }

    virtual ~Leader() {
        this->mandatoryCleanUp();
    }

    bool isLinked() const {
        return state == State::LINKED;
    }
    void tryLink() {
        BARRETT_SCOPED_LOCK(this->getEmMutex());
        state = State::LINKED;
    }
    void unlink() {
        BARRETT_SCOPED_LOCK(this->getEmMutex());
        state = State::UNLINKED;
    }

  protected:
    typename Output<jt_type>::Value* jtOutputValue;
    jp_type wamJP;
    jv_type wamJV;
    Eigen::Matrix<double, DOF + 3, 1> sendJpMsg;
    Eigen::Matrix<double, DOF + 3, 1> sendJvMsg;

    using ReceivedData = typename UDPHandler<DOF + 3>::ReceivedData;

    virtual void operate() {

        wamJP = wamJPIn.getValue();
        wamJV = wamJVIn.getValue();
        sendJpMsg << wamJP, 0.0, 0.0, 0.0; // added zero to send zero joint positions to the wrist part of the 7-dof follower
        sendJvMsg << wamJV, 0.0, 0.0, 0.0;

        udp_handler.send(sendJpMsg, sendJvMsg);

        boost::optional<ReceivedData> received_data = udp_handler.getLatestReceived();
        auto now = std::chrono::steady_clock::now();
        if (received_data && (now - received_data->timestamp <= TIMEOUT_DURATION)) {

            // theirJp = received_data->jp;
            // theirJv = received_data->jv;

            theirJp = received_data->jp.template head<DOF>();
            theirJv = received_data->jv.template head<DOF>();

        } else {
            if (state == State::LINKED) {
                std::cout << "lost link" << std::endl;
                state = State::UNLINKED;
            }
        }

        switch (state) {
            case State::INIT:
                control.setZero();
                jtOutputValue->setData(&control);
                break;
            case State::LINKED:
                // Active teleop. Only the callee can transition to LINKED
                control = compute_control(theirJp, theirJv, wamJP, wamJV);
                jtOutputValue->setData(&control);
                break;
            case State::UNLINKED:
                // Changed to unlinked with either timeout or callee.
                control.setZero();
                jtOutputValue->setData(&control);
                break;
        }
    }

    jp_type theirJp;
    jp_type theirJv;
    jt_type control;

  private:
    DISALLOW_COPY_AND_ASSIGN(Leader);
    std::mutex state_mutex;
    jp_type joint_positions;
    UDPHandler<DOF + 3> udp_handler;
    const std::chrono::milliseconds TIMEOUT_DURATION = std::chrono::milliseconds(20);
    State state;
    Eigen::Matrix<double, DOF, 1> kp;
    Eigen::Matrix<double, DOF, 1> kd;

    jt_type compute_control(const jp_type& ref_pos, const jv_type& ref_vel, const jp_type& cur_pos,
                            const jv_type& cur_vel) {
        jt_type pos_term = kp.asDiagonal() * (ref_pos - cur_pos);
        jt_type vel_term = kd.asDiagonal() * (ref_vel - cur_vel);
        return pos_term + vel_term;
    };
};
