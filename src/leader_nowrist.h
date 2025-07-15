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
    Input<jt_type> extTorqueIn;
    Input<jt_type> wamGravIn;
    Input<jt_type> wamDynIn;
    Output<jt_type> wamJPOutput;
    Output<jp_type> theirJPOutput;

    enum class State { INIT, LINKED, UNLINKED };

    explicit Leader(barrett::systems::ExecutionManager* em, const std::string& remoteHost, int rec_port = 5555,
                      int send_port = 5554, const std::string& sysName = "Leader")
        : System(sysName)
        , theirJp(0.0)
        , theirJv(0.0)
        , theirExtTorque(0.0)
        , control(0.0)
        , wamJPIn(this)
        , wamJVIn(this)
        , extTorqueIn(this)
        , wamGravIn(this)
        , wamDynIn(this)
        , wamJPOutput(this, &jtOutputValue)
        , theirJPOutput(this, &theirJPOutputValue)
        , udp_handler(remoteHost, send_port, rec_port)
        , state(State::INIT) {

        kp << 750, 1000, 400, 200;
        kd << 8.3, 8, 3.3, 0.8;


        if (em != NULL) {
            em->startManaging(*this);
        }
    }

    virtual ~Leader() {
        this->mandatoryCleanUp();
    }

    virtual bool inputsValid() {return true;}


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
    typename Output<jp_type>::Value* theirJPOutputValue;
    jp_type wamJP;
    jv_type wamJV;
    jt_type extTorque;
    jt_type wamGrav;
    jt_type wamDyn;
    Eigen::Matrix<double, DOF + 3, 1> sendJpMsg;
    Eigen::Matrix<double, DOF + 3, 1> sendJvMsg;
    Eigen::Matrix<double, DOF + 3, 1> sendExtTorqueMsg;

    using ReceivedData = typename UDPHandler<DOF + 3>::ReceivedData;

    virtual void operate() {

        wamJP = wamJPIn.getValue();
        wamJV = wamJVIn.getValue();
        wamGrav = wamGravIn.getValue();
        wamDyn = wamDynIn.getValue();

        if (extTorqueIn.valueDefined()) {
            extTorque = extTorqueIn.getValue();
            // std::cout << "defined" << std::endl;

        } else {
            // std::cout << "not defined" << std::endl;
            extTorque << 0.0, 0.0, 0,0, 0.0;
        }

        sendJpMsg << wamJP, 0.0, 0.0, 0.0;
        sendJvMsg << wamJV, 0.0, 0.0, 0.0;
        sendExtTorqueMsg << extTorque, 0.0, 0.0, 0.0;


        udp_handler.send(sendJpMsg, sendJvMsg, sendExtTorqueMsg);

        boost::optional<ReceivedData> received_data = udp_handler.getLatestReceived();
        auto now = std::chrono::steady_clock::now();
        if (received_data && (now - received_data->timestamp <= TIMEOUT_DURATION)) {

            theirJp = received_data->jp.template head<DOF>();
            theirJv = received_data->jv.template head<DOF>();
            theirExtTorque = received_data->extTorque.template head<DOF>();

            theirJPOutputValue->setData(&theirJp);


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
                control = compute_control(theirJp, theirJv, theirExtTorque, wamJP, wamJV, extTorque, wamGrav, wamDyn);
                jtOutputValue->setData(&control);
                break;
            case State::UNLINKED:
                // Changed to unlinked with either timeout or callee.
                control.setZero();
                jtOutputValue->setData(&control);
                break;
        }

        // sendExtTorqueMsg << control;


    }

    jp_type theirJp;
    jv_type theirJv;
    jt_type theirExtTorque;
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

    jt_type compute_control(const jp_type& ref_pos, const jv_type& ref_vel, const jt_type& ref_extTorque,
                            const jp_type& cur_pos, const jv_type& cur_vel, const jt_type& cur_extTorque,
                            const jt_type& cur_grav, const jt_type& cur_dyn) {
        // jt_type pos_term = kp.asDiagonal() * (ref_pos - cur_pos);
        // jt_type vel_term = kd.asDiagonal() * (ref_vel - cur_vel);
        // jt_type cur_extTorque_term = 1 * cur_extTorque;

        // jt_type u1 = pos_term + vel_term; // p-p control with PD
        // jt_type u2 = pos_term + vel_term + cur_extTorque; // p-p control with PD and extorqe compensation (it vibrates and becomes unstable)
        // jt_type u3 = 0.0 * cur_extTorque; // zero feedforward

        jt_type u1 = 0.0 * cur_extTorque; // zero feedforward

        jt_type u4 = 0.5 * cur_extTorque; // only compensating external torque

        jt_type u5; // only a controller on force
        u5.fill(0.0);
        u5[1] = -0.5 * (cur_extTorque[1] + ref_extTorque[1]);

        jt_type u6; // both feedforward and force controller
        u6.fill(0.0);
        u6[1] = -0.5 * (cur_extTorque[1] + ref_extTorque[1]) + 0.5 * cur_extTorque[1];

        jt_type u7 = 0.5 * cur_extTorque + cur_dyn - cur_grav; // external torque comp and dynamic compensation

        jt_type u8 = -0.5 * (cur_extTorque + ref_extTorque) + 0.5 * cur_extTorque + cur_dyn - cur_grav; // external torque comp and dynamic compensation

        jt_type u9; // torque controller and dynamic comp
        u9.fill(0.0);
        u9[1] = -0.5 * (cur_extTorque[1] + ref_extTorque[1]) + cur_dyn[1] - cur_grav[1];

        return u8;
    };
};