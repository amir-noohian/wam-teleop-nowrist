/*
 * ex11_master_master.cpp
 *
 *  Created on: Feb 22, 2010
 *      Author: Christopher Dellin
 *      Author: Dan Cody
 *      Author: Brian Zenowich
 */

// A version of 7-DOF follower.

#include "lib/external_torque.h"
#include <iostream>
#include <string>

#include <boost/thread.hpp>

#include <barrett/detail/stl_utils.h>
#include <barrett/os.h>
#include <barrett/products/product_manager.h>
#include <barrett/systems.h>
#include <barrett/units.h>

#define BARRETT_SMF_VALIDATE_ARGS
#include <barrett/standard_main_function.h>

#include "lib/background_state_publisher.h"
#include "lib/dynamic_external_torque.h"
#include "lib/follower_dynamics.h"
#include "lib/follower_pi0.h"

using namespace barrett;
using detail::waitForEnter;

void printUsage(const std::string &programName, const std::string &remoteHost, int recPort, int sendPort) {
    std::cout << "Usage: " << programName << " [remoteHost] [recPort] [sendPort]" << std::endl;
    std::cout << "       Defaults: remoteHost=" << remoteHost << ", recPort=" << recPort << ", sendPort=" << sendPort
              << std::endl;
    std::cout << "       -h or --help: Display this help message." << std::endl;
}

bool validate_args(int argc, char **argv) {

    if ((argc == 2 && (std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help")) || (argc > 4)) {
        printUsage(argv[0], "127.0.0.1", 5554, 5555);
        return 0;
    }

    return true;
}

template <size_t DOF>
typename units::JointTorques<DOF>::type saturateJt(const typename units::JointTorques<DOF>::type &x,
                                                   const typename units::JointTorques<DOF>::type &limit) {
    int index;
    double minRatio;

    minRatio = (limit.array() / (x.cwiseAbs()).array()).minCoeff(&index);
    if (minRatio < 1.0) {
        return minRatio * x;
    } else {
        return x;
    }
}

template <size_t DOF> int wam_main(int argc, char **argv, ProductManager &pm, systems::Wam<DOF> &wam) {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

    jp_type DEMO_POS; // the position each WAM should move to before starting policy control
    jp_type SYNC_POS; // the position each WAM should move to before linking
    if (DOF == 7) {
        SYNC_POS[0] = 0.0;
        SYNC_POS[1] = -1.95;
        SYNC_POS[2] = 0.0;
        SYNC_POS[3] = 2.97;
        SYNC_POS[4] = 0.0;
        SYNC_POS[5] = 0.0;
        SYNC_POS[6] = 0.0;

        // TODO SET DEMO POS
        DEMO_POS[0] = 0.0;
        DEMO_POS[1] = 0.25;
        DEMO_POS[2] = 0.0;
        DEMO_POS[3] = 2.5;
        DEMO_POS[4] = 0.0;
        DEMO_POS[5] = 0.0;
        DEMO_POS[6] = 0.0;

    } else {
        printf("Error: 7 DOF supported\n");
        return false;
    }

    std::string remoteHost = "127.0.0.1";
    int rec_port = 5554;
    int send_port = 5555;
    int policy_rec_port = 5556;
    int policy_send_port = 5557;
    bool use_dynamics_for_ext_torque = false;

    if (argc >= 2) {
        remoteHost = std::string(argv[1]);
    }
    if (argc >= 3) {
        rec_port = std::atoi(argv[2]);
    }
    if (argc >= 4) {
        send_port = std::atoi(argv[3]);
    }
    if (argc >= 5) {
        policy_rec_port = std::atoi(argv[4]);
    }
    if (argc >= 6) {
        policy_send_port = std::atoi(argv[5]);
    }

    ros::init(argc, argv, "follower");
    BackgroundStatePublisher<DOF> state_publisher(pm.getExecutionManager(), wam);

    barrett::systems::Summer<jt_type, 3> customjtSum;
    pm.getExecutionManager()->startManaging(customjtSum);

    FollowerDynamics<DOF> followerDynamics(pm.getExecutionManager());

    ExternalTorque<DOF> externalTorque(pm.getExecutionManager());

    DynamicExternalTorque<DOF> dynamicExternalTorque(pm.getExecutionManager());

    jt_type jtLimits;
    // TODO: these limits are from the wam follower conf. Should try to find better ones
    // These make sure we don't apply too high a force from the human
    jtLimits[0] = 25.0;
    jtLimits[1] = 20.0;
    jtLimits[2] = 15.0;
    jtLimits[3] = 15.0;
    jtLimits[4] = 5.0;
    jtLimits[5] = 5.0;
    jtLimits[6] = 5.0;
    systems::Callback<jt_type> saturateCallback(boost::bind(saturateJt<DOF>, _1, jtLimits));

    v_type initial_p_gains = wam.jpController.getKp();
    v_type initial_d_gains = wam.jpController.getKd();

    // NOTE kinda randomly chosen number for now. The more we decrease the follower gains
    // during policy rollout, the larger range we can easily move on the leader side without
    // feeling heavy resistance. 0.125 seems like a decent spot, but this will also be affected
    // by the scale we put on external torque from leader
    v_type policy_p_gains = initial_p_gains * 0.1;
    v_type policy_d_gains = initial_d_gains;

    barrett::systems::FirstOrderFilter<jt_type> extFilter;
    jt_type omega_p(180.0);
    extFilter.setLowPass(omega_p);
    pm.getExecutionManager()->startManaging(extFilter);

    barrett::systems::FirstOrderFilter<jt_type> dynamicExtFilter;
    dynamicExtFilter.setLowPass(omega_p);
    pm.getExecutionManager()->startManaging(dynamicExtFilter);

    ja_type ja;
    ja.setConstant(0.0);
    systems::Constant<ja_type> zeroAcceleration(ja);
    pm.getExecutionManager()->startManaging(zeroAcceleration);

    FollowerPi0<DOF> follower(pm.getExecutionManager(), remoteHost, rec_port, send_port, policy_rec_port,
                              policy_send_port);

    jt_type maxRate; // Nm · s-1 per joint
    maxRate << 50, 50, 50, 50;
    systems::RateLimiter<jt_type> wamJPOutputRamp(maxRate, "ffRamp");

    systems::PrintToStream<jt_type> printdynamicextTorque(pm.getExecutionManager(), "dynamicextTorque: ");
    systems::PrintToStream<jt_type> printSC(pm.getExecutionManager(), "SC: ");
    systems::PrintToStream<jp_type> printPolicy(pm.getExecutionManager(), "Policy: ");
    systems::PrintToStream<jt_type> printSaturateJt(pm.getExecutionManager(), "SatJt (FF TORQ): ");
    systems::PrintToStream<jp_type> printTheirJp(pm.getExecutionManager(), "TheirJP: ");
    systems::PrintToStream<jt_type> printWamInput(pm.getExecutionManager(), "WAM.input: ");

    // systems::PrintToStream<jt_type> printcustomjtSum(pm.getExecutionManager(),
    // "customjtSum: ");

    double h_omega_p = 25.0;
    barrett::systems::FirstOrderFilter<jv_type> hp1;
    hp1.setHighPass(jv_type(h_omega_p), jv_type(h_omega_p));
    systems::Gain<jv_type, double, ja_type> jaWAM(1.0);
    pm.getExecutionManager()->startManaging(hp1);

    barrett::systems::FirstOrderFilter<ja_type> jaFilter;
    ja_type l_omega_p = ja_type::Constant(50.0);
    jaFilter.setLowPass(l_omega_p);
    pm.getExecutionManager()->startManaging(jaFilter);

    systems::connect(wam.jvOutput, hp1.input);
    systems::connect(hp1.output, jaWAM.input);
    systems::connect(jaWAM.output, jaFilter.input);
    systems::connect(jaFilter.output, followerDynamics.jaInputDynamics);

    systems::connect(wam.jpOutput, follower.wamJPIn);
    systems::connect(wam.jvOutput, follower.wamJVIn);
    systems::connect(customjtSum.output, follower.jtSumIn);

    if (use_dynamics_for_ext_torque) {
        systems::connect(dynamicExternalTorque.wamExternalTorqueOut, follower.extTorqueIn);
        // systems::connect(dynamicExtFilter.output, follower.extTorqueIn);
    } else {
        systems::connect(externalTorque.wamExternalTorqueOut, follower.extTorqueIn);
        // systems::connect(extFilter.output, follower.extTorqueIn);
    }
    systems::connect(wam.jpOutput, followerDynamics.jpInputDynamics);
    systems::connect(wam.jvOutput, followerDynamics.jvInputDynamics);
    // systems::connect(zeroAcceleration.output,
    // followerDynamics.jaInputDynamics);

    systems::connect(follower.wamJPOutput, customjtSum.getInput(0));
    systems::connect(wam.gravity.output, customjtSum.getInput(1));
    systems::connect(wam.supervisoryController.output, customjtSum.getInput(2));

    if (use_dynamics_for_ext_torque) {
        systems::connect(customjtSum.output, dynamicExternalTorque.wamTorqueSumIn);
        systems::connect(followerDynamics.dynamicsFeedFWD, dynamicExternalTorque.wamDynamicsIn);
        systems::connect(dynamicExternalTorque.wamExternalTorqueOut, dynamicExtFilter.input);
    } else {
        systems::connect(wam.gravity.output, externalTorque.wamGravityIn);
        systems::connect(customjtSum.output, externalTorque.wamTorqueSumIn);
        systems::connect(externalTorque.wamExternalTorqueOut, extFilter.input);
    }

    systems::connect(wam.gravity.output, follower.wamGravIn);
    systems::connect(followerDynamics.dynamicsFeedFWD, follower.wamDynIn);

    // systems::connect(dynamicExternalTorque.wamExternalTorqueOut, printdynamicextTorque.input);
    // systems::connect(dynamicExternalTorque.wamExternalTorqueOut,
    // printdynamicextTorque.input);
    // systems::connect(wam.supervisoryController.output, printSC.input);
    // systems::connect(dynamicExtFilter.output, printcustomjtSum.input);
    // systems::connect(follower.policyOutput, printPolicy.input);

    if (use_dynamics_for_ext_torque) {
        systems::connect(dynamicExternalTorque.wamExternalTorqueOut, state_publisher.exposedExternalTorque.input);
        // systems::connect(dynamicExtFilter.output, state_publisher.exposedExternalTorque.input);
    } else {
        systems::connect(externalTorque.wamExternalTorqueOut, state_publisher.exposedExternalTorque.input);
        // systems::connect(extFilter.output, state_publisher.exposedExternalTorque.input);
    }

    wam.gravityCompensate();

    std::string line;
    v_type gainTmp;

    bool going = true;
    bool set_demo_start = false;

    while (going) {
        printf(">>> ");
        std::getline(std::cin, line);

        switch (line[0]) {

        case 'p':
            if (follower.isRollingOut() || follower.needReset()) {
                // If already rolling out, disable policy rollouts and switch back to
                // following the leader

                // The FF term does go back to 0. Follower policy also looks fine.
                // When we are in policy mode, there is a bit of a "desync" of sorts when we apply external forces
                // And maybe that somehow leads to this.
                // Is there a way to somehow "re-link" force re-link them?

                // Reset gains to their default values
                wam.jpController.setKp(initial_p_gains);
                wam.jpController.setKd(initial_d_gains);

                wam.supervisoryController.disconnectInput();
                systems::disconnect(wam.input);

                // Try to force re-link them?
                wam.jpController.resetIntegrator();
                // wam.moveTo(follower.theirJp, true);
                // btsleep(0.5);
                // printf("Press [Enter] to re-link with the other WAM.");
                // waitForEnter();
                // wam.idle();

                // wam.trackReferenceSignal(follower.theirJPOutput);

                follower.disablePolicyRollouts();
                printf("Online policy tuning disabled - rollout stopped.\n");
            } else {
                // If linked, switch to policy rollouts
                follower.switchToPolicyRollouts();

                // adjust gains
                wam.jpController.setKp(policy_p_gains);
                wam.jpController.setKd(policy_d_gains);

                wam.supervisoryController.disconnectInput();
                // TODO uncomment these to actually follow the policy.
                // Don't need saturate callback for now tho
                // systems::connect(saturateCallback.output, wam.input);
                wam.trackReferenceSignal(follower.policyOutput);
                printf("Online policy tuning enabled - rollout starting.\n");
            }
            break;

        case 's':
            if (!follower.isRollingOut()) {
                DEMO_POS = wam.getJointPositions();
                set_demo_start = true;
                printf("saved joint positions state: %f %f %f %f %f %f %f", DEMO_POS[0], DEMO_POS[1], DEMO_POS[2],
                       DEMO_POS[3], DEMO_POS[4], DEMO_POS[5], DEMO_POS[6]);
            } else {
                printf("Rolling out, stop with r first.");
            }

            break;
        case 'h':
            if (!follower.isRollingOut()) {
                wam.moveHome(true);
                btsleep(0.1); // wait an execution cycle or two
                printf("moved to home.\n");
            } else {
                printf("Rolling out, stop with r first.");
            }
            break;

        case 'g':
            if (!follower.isRollingOut()) {
                // we rely on previous leader to follower linking to move them together
                wam.moveTo(DEMO_POS, true);

                btsleep(0.1); // wait an execution cycle or two
                printf("moved to demo start.\n");

            } else {
                printf("Rolling out, stop with r first.");
            }

            break;

        case 't':
            size_t jointIndex;
            {
                size_t jointNumber;
                std::cout << "\tJoint: ";
                std::cin >> jointNumber;
                jointIndex = jointNumber - 1;

                if (jointIndex >= DOF) {
                    std::cout << "\tBad joint number: " << jointNumber;
                    break;
                }
            }

            char gainId;
            std::cout << "\tGain identifier (p, i, or d): ";
            std::cin >> line;
            gainId = line[0];

            std::cout << "\tCurrent value: ";
            switch (gainId) {
            case 'p':
                gainTmp = wam.jpController.getKp();
                break;
            case 'i':
                gainTmp = wam.jpController.getKi();
                break;
            case 'd':
                gainTmp = wam.jpController.getKd();
                break;

            default:
                std::cout << "\tBad gain identifier.";
            }
            std::cout << gainTmp[jointIndex] << std::endl;

            std::cout << "\tNew value: ";
            std::cin >> gainTmp[jointIndex];
            switch (gainId) {
            case 'p':
                wam.jpController.setKp(gainTmp);
                break;
            case 'i':
                wam.jpController.setKi(gainTmp);
                break;
            case 'd':
                wam.jpController.setKd(gainTmp);
                break;

            default:
                std::cout << "\tBad gain identifier.";
            }

            break;
        case 'x':
            going = false;
            break;

        default:
            printf("\n");
            printf("    'l' to toggle linking with other WAM\n");
            printf("    't' to tune control gains\n");
            printf("    'p' to start moving with the policy. (policy needs to already be running)\n");
            printf("    's' to save current position as demo start\n");
            printf("    'g' go to saved start position\n");
            printf("    'x' to exit\n");

            break;
        }
    }

    pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);

    return 0;
}