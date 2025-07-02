/*
 * ex11_master_master.cpp
 *
 *  Created on: Feb 22, 2010
 *      Author: Christopher Dellin
 *      Author: Dan Cody
 *      Author: Brian Zenowich
 */

#include "external_torque.h"
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

#include "leader_nowrist.h"
#include "background_state_publisher.h"
#include "leader_dynamics.h"
#include "dynamic_external_torque.h"

using namespace barrett;
using detail::waitForEnter;

void printUsage(const std::string& programName, const std::string& remoteHost, int recPort, int sendPort) {
    std::cout << "Usage: " << programName << " [remoteHost] [recPort] [sendPort]" << std::endl;
    std::cout << "       Defaults: remoteHost=" << remoteHost << ", recPort=" << recPort << ", sendPort=" << sendPort
              << std::endl;
    std::cout << "       -h or --help: Display this help message." << std::endl;
}

bool validate_args(int argc, char** argv) {

    if ((argc == 2 && (std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help")) || (argc > 4)) {
        printUsage(argv[0], "127.0.0.1", 5555, 5554);
        return 0;
    }

    return true;
}

template <size_t DOF> int wam_main(int argc, char **argv, ProductManager &pm, systems::Wam<DOF> &wam) {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

    jp_type SYNC_POS; // the position each WAM should move to before linking
    if (DOF == 4) {
        SYNC_POS[0] = 0.0;
        SYNC_POS[1] = -1.95;
        SYNC_POS[2] = 0.0;
        SYNC_POS[3] = 2.7;
        // SYNC_POS[4] = 0.0;
        // SYNC_POS[5] = 0.0;
        // SYNC_POS[6] = 0.0;

    } else {
        printf("Error: 4 DOF supported\n");
        return false;
    }

    std::string remoteHost = "127.0.0.1";
    int rec_port = 5555;
    int send_port = 5554;

    if (argc >= 2) {
        remoteHost = std::string(argv[1]);
    }
    if (argc >= 3) {
        rec_port = std::atoi(argv[2]);
    }
    if (argc >= 4) {
        send_port = std::atoi(argv[3]);
    }

    ros::init(argc, argv, "leader_nowrist");
    BackgroundStatePublisher<DOF> state_publisher(pm.getExecutionManager(), wam);

    barrett::systems::Summer<jt_type, 3> customjtSum;
    pm.getExecutionManager()->startManaging(customjtSum);

    LeaderDynamics<DOF> leaderDynamics(pm.getExecutionManager());

    ExternalTorque<DOF> externalTorque(pm.getExecutionManager());

    DynamicExternalTorque<DOF> dynamicExternalTorque(pm.getExecutionManager());
    
    barrett::systems::FirstOrderFilter<jt_type> extFilter;
    jt_type omega_p(180.0);
    extFilter.setLowPass(omega_p);
    pm.getExecutionManager()->startManaging(extFilter);

    barrett::systems::FirstOrderFilter<jt_type> dynamicExtFilter;
    // jt_type omega_p(180.0);
    dynamicExtFilter.setLowPass(omega_p);
    pm.getExecutionManager()->startManaging(dynamicExtFilter);

    ja_type ja;
    ja.setConstant(0.0);
    systems::Constant<ja_type> zeroAcceleration(ja);
    pm.getExecutionManager()->startManaging(zeroAcceleration);

    Leader<DOF> leader(pm.getExecutionManager(), remoteHost, rec_port, send_port);

    jt_type maxRate; // Nm · s-1 per joint
    maxRate << 50, 50, 50, 50;
    systems::RateLimiter<jt_type> wamJPOutputRamp(maxRate, "ffRamp");

    systems::PrintToStream<jt_type> printdynamicextTorque(pm.getExecutionManager(), "dynamicextTorque: ");
    systems::PrintToStream<jt_type> printextTorque(pm.getExecutionManager(), "extTorque: ");
    systems::PrintToStream<jt_type> printdynamicoutput(pm.getExecutionManager(), "dynamicoutput: ");
    systems::PrintToStream<jt_type> printSC(pm.getExecutionManager(), "SC: ");
    // systems::PrintToStream<jt_type> printjtSum(pm.getExecutionManager(), "jtSum: ");
    // systems::PrintToStream<jt_type> printcustomjtSum(pm.getExecutionManager(), "customjtSum: ");

    systems::connect(wam.jpOutput, leader.wamJPIn);
    systems::connect(wam.jvOutput, leader.wamJVIn);
    systems::connect(dynamicExtFilter.output, leader.extTorqueIn);

    systems::connect(wam.jpOutput, leaderDynamics.jpInputDynamics);
    systems::connect(wam.jvOutput, leaderDynamics.jvInputDynamics);
    systems::connect(zeroAcceleration.output, leaderDynamics.jaInputDynamics);

    systems::connect(leader.wamJPOutput, customjtSum.getInput(0));
    systems::connect(wam.gravity.output, customjtSum.getInput(1));
    systems::connect(wam.supervisoryController.output, customjtSum.getInput(2));

    systems::connect(wam.gravity.output, externalTorque.wamGravityIn);
    systems::connect(customjtSum.output, externalTorque.wamTorqueSumIn);
    systems::connect(externalTorque.wamExternalTorqueOut, extFilter.input);

    systems::connect(wam.gravity.output, dynamicExternalTorque.wamGravityIn);
    systems::connect(customjtSum.output, dynamicExternalTorque.wamTorqueSumIn);
    systems::connect(leaderDynamics.dynamicsFeedFWD, dynamicExternalTorque.wamDynamicsIn);
    systems::connect(dynamicExternalTorque.wamExternalTorqueOut, dynamicExtFilter.input);

    systems::connect(dynamicExtFilter.output, printdynamicextTorque.input);
    systems::connect(extFilter.output, printextTorque.input);
    systems::connect(wam.supervisoryController.output, printSC.input);
    systems::connect(leaderDynamics.dynamicsFeedFWD, printdynamicoutput.input);

    // systems::connect(extFilter.output, printjtSum.input);
    // systems::connect(extFilter.output, printcustomjtSum.input);


    wam.gravityCompensate();


    std::string line;
    v_type gainTmp;

    bool going = true;

    while (going) {
        printf(">>> ");
        std::getline(std::cin, line);

        switch (line[0]) {
        case 'l':
            if (leader.isLinked()) {
                leader.unlink();
            } else {
                wam.moveTo(SYNC_POS, true);

                printf("Press [Enter] to link with the other WAM.");
                waitForEnter();
                leader.tryLink();
                wam.trackReferenceSignal(leader.theirJPOutput);
                // connect(leader.wamJPOutput, wam.input);
                connect(leader.wamJPOutput, wamJPOutputRamp.input); // one of the problem with the joint limiter is that it adds delay in applying external torque to the robot.
                connect(wamJPOutputRamp.output, wam.input);
                // systems::forceConnect(wam.jtSum.output, externalTorque.wamTorqueSumIn);

                btsleep(0.1); // wait an execution cycle or two
                if (leader.isLinked()) {
                    printf("Linked with remote WAM.\n");
                } else {
                    printf("WARNING: Linking was unsuccessful.\n");
                }
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
            printf("    'x' to exit\n");

            break;
        }
    }


    pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);

    return 0;
}