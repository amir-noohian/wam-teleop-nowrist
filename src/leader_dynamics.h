// This files calculate the dynamics without gravity for 2DOF WAM

#ifndef LEADER_DYNAMICS_H_
#define LEADER_DYNAMICS_H_

#include <barrett/math/traits.h>
#include <list>
#include <barrett/units.h>
#include <barrett/detail/ca_macro.h>
#include <barrett/systems/abstract/system.h>

#include <eigen3/Eigen/Core>
#include <libconfig.h++>

#include <barrett/detail/ca_macro.h>
#include <barrett/math/traits.h>
#include <barrett/systems/abstract/execution_manager.h>
#include <barrett/systems/abstract/controller.h>

#include <math.h>
#include <Eigen/Dense>
#include <iostream>


Eigen::Matrix<double, 8, 1> initialize_pi() {
    Eigen::Matrix<double, 8, 1> pi;
    pi <<      -0.0164,
   -0.1713,
   -0.1909,
   -1.1084,
   -2.0976,
   -0.1389,
   -0.3147,
   -0.4007;

    return pi;
}

// Function to calculate the Y matrix
Eigen::Matrix<double,2 ,8> calculate_Y_matrix(const Eigen::Vector4d theta, Eigen::Vector4d thetad, Eigen::Vector4d thetadd) {
    
    Eigen::Matrix<double, 2, 8> Y;
    Y.setZero(); // Initialize the matrix with zeros
    Y(0, 0) = 2.0 * cos(theta[3]) * thetad[3] * thetad[1] + cos(theta[3]) * pow(thetad[3], 2) + 2.0 * sin(theta[3]) * thetadd[1] + sin(theta[3]) * thetadd[3];
    Y(0, 1) = 2.0 * cos(theta[3]) * thetadd[1] - sin(theta[3]) * pow(thetad[3], 2) - 2.0 * sin(theta[3]) * thetad[3] * thetad[1] + cos(theta[3]) * thetadd[3];
    Y(0, 2) = thetadd[3];
    Y(0, 3) = thetadd[1];
    Y(0, 4) = tanh(20 * thetad[1]);
    Y(0, 5) = thetad[1];
    Y(0, 6) = 0.0;
    Y(0, 7) = 0.0;

    Y(1, 0) = sin(theta[3]) * thetadd[1] - cos(theta[3]) * pow(thetad[1], 2);
    Y(1, 1) = sin(theta[3]) * pow(thetad[1], 2) + cos(theta[3]) * thetadd[1];
    Y(1, 2) = thetadd[1] + thetadd[3];
    Y(1, 3) = 0.0;
    Y(1, 4) = 0.0;
    Y(1, 5) = 0.0;
    Y(1, 6) = tanh(20 * thetad[3]);
    Y(1, 7) = thetad[3];

    return Y;
}

using namespace barrett;

template<size_t DOF>
class LeaderDynamics: public systems::System {

	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	/* Torque*/
public:
	Input<jp_type> jpInputDynamics;
	Input<jv_type> jvInputDynamics;
    Input<ja_type> jaInputDynamics;

public:
	Output<jt_type> dynamicsFeedFWD;
protected:
	typename Output<jt_type>::Value* dynamicsFeedFWDValue;

public:
	explicit LeaderDynamics(systems::ExecutionManager* em) :
			jpInputDynamics(this), jvInputDynamics(this), jaInputDynamics(this), dynamicsFeedFWD(this,
					&dynamicsFeedFWDValue){
			      em->startManaging(*this);
		    }

	virtual ~LeaderDynamics() {
		this->mandatoryCleanUp();
	}

protected:
	jp_type tmp_theta_pos;
	jv_type tmp_theta_vel;
	ja_type tmp_theta_acc;
	Eigen::Matrix<double,2 ,8> Y;
	Eigen::Matrix<double, 8, 1> P;
	Eigen::Vector4d ThetaInput;
	Eigen::Vector4d ThetadotInput;
	Eigen::Vector4d ThetaddotInput;
	Eigen::Vector2d FeedFwd;
	jt_type dynFeedFWD;

	virtual void operate() {
		tmp_theta_pos = this->jpInputDynamics.getValue();
		ThetaInput << tmp_theta_pos[0], tmp_theta_pos[1], tmp_theta_pos[2], tmp_theta_pos[3];
		tmp_theta_vel = this->jvInputDynamics.getValue();
		ThetadotInput << tmp_theta_vel[0], tmp_theta_vel[1], tmp_theta_vel[2], tmp_theta_vel[3];
		tmp_theta_acc = this->jaInputDynamics.getValue();
		ThetaddotInput << tmp_theta_acc[0], tmp_theta_acc[1], tmp_theta_acc[2], tmp_theta_acc[3];
		Y = calculate_Y_matrix(ThetaInput, ThetadotInput, ThetaddotInput);
		P = initialize_pi();
		
		FeedFwd = Y * P;
	
		dynFeedFWD[1] = FeedFwd[0];
		dynFeedFWD[3] = FeedFwd[1];

		this->dynamicsFeedFWDValue->setData(&dynFeedFWD);

	}
private:
	DISALLOW_COPY_AND_ASSIGN(LeaderDynamics);
};
#endif /* LEADER_DYNAMICS_H_ */