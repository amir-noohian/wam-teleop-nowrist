#pragma once

#include <boost/asio.hpp>

#include "udp_handler.h"
#include <barrett/detail/ca_macro.h>
#include <barrett/systems/abstract/single_io.h>
#include <barrett/thread/abstract/mutex.h>
#include <barrett/units.h>

template <size_t DOF> class FollowerPolicy : public barrett::systems::System {
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
  Input<jp_type> wamJPIn;
  Input<jv_type> wamJVIn;
  Input<jt_type> extTorqueIn;
  Input<jt_type> wamGravIn;
  Input<jt_type> wamDynIn;
  Output<jt_type> wamJPOutput;
  Output<jp_type> policyOutput;
  Output<jp_type> theirJPOutput;

  enum class State { INIT, LINKED, UNLINKED, ROLLOUTS, NEED_RESET };

  explicit FollowerPolicy(barrett::systems::ExecutionManager *em,
                          const std::string &remoteHost, int rec_port = 5554,
                          int send_port = 5555, int policy_rec_port = 5556,
                          int policy_send_port = 5557,
                          const std::string &sysName = "FollowerPolicy")
      : System(sysName), theirJp(0.0), theirJv(0.0), theirExtTorque(0.0),
        control(0.0), wamJPIn(this), wamJVIn(this), extTorqueIn(this),
        wamGravIn(this), wamDynIn(this), wamJPOutput(this, &jtOutputValue),
        policyOutput(this, &policyOutputValue),
        theirJPOutput(this, &theirJPOutputValue),
        udp_handler(remoteHost, send_port, rec_port),
        policy_handler(remoteHost, policy_send_port, policy_rec_port),
        state(State::INIT) {

    kp << 750, 1000, 400, 200, 10, 10, 2.5;
    kd << 8.3, 8, 3.3, 0.8, 0.5, 0.5, 0.05;
    cf << 0.375, 0.4, 0.2, 0.1, 0.01, 0.01, 0.01;

    if (em != NULL) {
      em->startManaging(*this);
    }
  }

  virtual ~FollowerPolicy() { this->mandatoryCleanUp(); }

  virtual bool inputsValid() { return true; }

  bool isLinked() const { return state == State::LINKED; }
  void tryLink() {
    BARRETT_SCOPED_LOCK(this->getEmMutex());
    state = State::LINKED;
  }
  void unlink() {
    BARRETT_SCOPED_LOCK(this->getEmMutex());
    state = State::UNLINKED;
  }

  bool isRollingOut() const { return state == State::ROLLOUTS; }

  bool needReset() const { return state == State::NEED_RESET; }
  void switchToPolicyRollouts() {
    BARRETT_SCOPED_LOCK(this->getEmMutex());
    state = State::ROLLOUTS;
    tmp_jp[0] = wamJP[0];
    tmp_jp[1] = wamJP[1];
    tmp_jp[2] = wamJP[2];
    tmp_jp[3] = wamJP[3];
    tmp_jp[4] = wamJP[4];
    tmp_jp[5] = wamJP[5];
    tmp_jp[6] = wamJP[6];

    policyOutputValue->setData(
        &tmp_jp); // initialize the policy output to the current position to
                 // avoid jumps when switching.
  }
  void disablePolicyRollouts() {
    BARRETT_SCOPED_LOCK(this->getEmMutex());
    state = State::LINKED;
  }

  bool arePositionsEqual(jp_type pos_1, jp_type pos_2, float epsilon) {
      if (pos_1.size() != pos_2.size()) return false;

      float diff = 0.0;
      for (size_t i = 0; i < pos_1.size(); ++i) {
          diff += std::abs(pos_1[i] - pos_2[i]);
      }

      return diff < epsilon;
  }

  jp_type theirJp;


protected:
  typename Output<jt_type>::Value *jtOutputValue;
  typename Output<jp_type>::Value *theirJPOutputValue;
  typename Output<jp_type>::Value *policyOutputValue;
  jp_type wamJP;
  jv_type wamJV;
  jt_type extTorque;
  jt_type wamGrav;
  jt_type wamDyn;
  jp_type tmp_jp;
  Eigen::Matrix<double, DOF, 1> sendJpMsg;
  Eigen::Matrix<double, DOF, 1> sendJvMsg;
  Eigen::Matrix<double, DOF, 1> sendExtTorqueMsg;

  using ReceivedData = typename UDPHandler<DOF>::ReceivedData;

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
      extTorque << 0.0, 0.0, 0, 0, 0.0;
    }

    sendJpMsg << wamJP;
    sendJvMsg << wamJV;
    sendExtTorqueMsg << extTorque;

    udp_handler.send(sendJpMsg, sendJvMsg, sendExtTorqueMsg);

    boost::optional<ReceivedData> received_data =
        udp_handler.getLatestReceived();
    auto now = std::chrono::steady_clock::now();
    if (received_data && (now - received_data->timestamp <= TIMEOUT_DURATION)) {

      theirJp = received_data->jp;
      theirJv = received_data->jv;
      theirExtTorque = received_data->extTorque;
      theirJPOutputValue->setData(&theirJp);

    } else {
      if (state == State::LINKED) {
        std::cout << "lost link" << std::endl;
        state = State::UNLINKED;
      }
    }

    boost::optional<ReceivedData> policy_received_data =
        policy_handler.getLatestReceived();

    if (policy_received_data && state == State::ROLLOUTS &&
        (now - policy_received_data->timestamp <= POLICY_TIMEOUT_DURATION)) {
    // if (policy_received_data && state == State::ROLLOUTS) {
      // Update the policy information
      policyJp = policy_received_data->jp;
      policyJv = policy_received_data->jv;
      policyTorque = policy_received_data->extTorque;

      policyOutputValue->setData(&policyJp);
    } else {
      // NOTE comment this out if you just want to test position
        // if (state == State::ROLLOUTS) {
        //   std::cout << "lost policy link, press p to switch back to teleop control"
        //             << std::endl;
        //   // Can't go to "linked", because the main loop needs to switch us back
        //   // to following the right reference signal first.
        //   state = State::NEED_RESET; 
        // }
    }

    switch (state) {
    case State::INIT:
      control.setZero();
      jtOutputValue->setData(&control);
      break;
    case State::LINKED:
      // Active teleop. Only the callee can transition to LINKED
      control = compute_control(theirJp, theirJv, theirExtTorque, wamJP, wamJV,
                                extTorque, wamGrav, wamDyn);
      jtOutputValue->setData(&control);
      break;
    case State::ROLLOUTS:
      // Policy has control over our position; the forces from the leader allow
      // for corrections.
      // NOTE: we add the policy info here though it is not used asof now.
      control = compute_control_finetune(
          policyJp, policyJv, policyTorque, theirJp, theirJv, theirExtTorque,
          wamJP, wamJV, extTorque, wamGrav, wamDyn);
      jtOutputValue->setData(&control);
      break;
    case State::UNLINKED:
      // Changed to unlinked with either timeout or callee.
      control.setZero();
      jtOutputValue->setData(&control);
      break;
    }

    // sendExtTorqueMsg << control;

    // udp_handler.send(sendJpMsg, sendJvMsg, sendExtTorqueMsg);
  }

  jv_type theirJv;
  jt_type theirExtTorque;
  jt_type control;
  jp_type policyJp;
  jv_type policyJv;
  jt_type policyTorque;

private:
  DISALLOW_COPY_AND_ASSIGN(FollowerPolicy);
  std::mutex state_mutex;
  jp_type joint_positions;
  UDPHandler<DOF> udp_handler;
  UDPHandler<DOF> policy_handler;
  const std::chrono::milliseconds TIMEOUT_DURATION =
      std::chrono::milliseconds(20);
  const std::chrono::milliseconds POLICY_TIMEOUT_DURATION =
      std::chrono::milliseconds(100);
  State state;
  Eigen::Matrix<double, DOF, 1> kp;
  Eigen::Matrix<double, DOF, 1> kd;
  Eigen::Matrix<double, DOF, 1> cf;

  jt_type
  compute_control_finetune(const jp_type &policy_pos, const jv_type &policy_vel,
                           const jt_type &policy_torque, const jp_type &ref_pos,
                           const jv_type &ref_vel, const jt_type &ref_extTorque,
                           const jp_type &cur_pos, const jv_type &cur_vel,
                           const jt_type &cur_extTorque,
                           const jt_type &cur_grav, const jt_type &cur_dyn) {
    /**
      NOTE we pass in policy information, as well as leader information, in case
      either is needed for control law.
    */

    jt_type u0 = 0.0 * cur_extTorque; // zero feedforward (equal to default P-P
                                      // with gravity compensation)
    // PP with external force from the leader (haptic corrections)
    jt_type u1 = -1.0 * ref_extTorque;

    // PP with external force from the leader (haptic corrections)
    // AND dynamic compensation
    jt_type u2 = -0.5 * ref_extTorque + cur_dyn - cur_grav;

    // for now p-p only. LATER SHOULD BE u1!!!
    return u1;
  }

  jt_type compute_control(const jp_type &ref_pos, const jv_type &ref_vel,
                          const jt_type &ref_extTorque, const jp_type &cur_pos,
                          const jv_type &cur_vel, const jt_type &cur_extTorque,
                          const jt_type &cur_grav, const jt_type &cur_dyn) {

    // cases where the follower and leader have the same control law

    jt_type u1 = 0.0 * cur_extTorque; // zero feedforward (equal to default P-P
                                      // with gravity compensation)

    jt_type u2 = cur_dyn - cur_grav; // P-P with dynamic compensation

    jt_type u3 =
        -0.5 * ref_extTorque; // PF-PF with ref external torque feedback

    jt_type u4 = -0.5 * ref_extTorque + cur_dyn -
                 cur_grav; // PF-PF with ref external torque feedback and
                           // dynamic compensation (Lawrence's perfect
                           // transparency architecture);

    jt_type u5 = -0.5 * ref_extTorque -
                 0.15 * (ref_extTorque +
                         cur_extTorque); // PF-PF with ref external torque and
                                         // cur external torque feedback

    jt_type u6 = -0.5 * ref_extTorque - 0.15 * (ref_extTorque + cur_extTorque) +
                 cur_dyn - cur_grav; // it has the best performance

    // cases that the leader side has differnt controller that the follower

    jt_type u7 = -0.0 * cur_extTorque; // zero

    jt_type u8 = -0.0 * (ref_extTorque + cur_extTorque); // zero

    // jt_type u9 = -0.0 * cur_extTorque - 0.0 * (ref_extTorque +
    // cur_extTorque);

    jt_type u = u1;

    for (size_t i = 4; i < 7; ++i) {
      u[i] = 0.0;
    }
    return u;
  };
};