#pragma once

#include <barrett/detail/ca_macro.h>
#include <barrett/systems.h>
#include <barrett/units.h>

template <size_t DOF>
class ExternalTorque : public barrett::systems::System {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  public:
    Input<jt_type> wamTorqueSumIn;
    Input<jt_type> wamGravityIn;
    Output<jt_type> wamExternalTorqueOut;

    explicit ExternalTorque(barrett::systems::ExecutionManager* em, const std::string& sysName = "ExternalTorque")
        : System(sysName)
        , wamTorqueSumIn(this)
        , wamGravityIn(this)
        , wamExternalTorqueOut(this, &jtOutputValue) {

        if (em != NULL) {
            em->startManaging(*this);
        }
    }

    virtual ~ExternalTorque() {
        this->mandatoryCleanUp();
    }

  protected:
    typename Output<jt_type>::Value* jtOutputValue;
    jt_type jtSum;
    jt_type gravity;
    jt_type externalTorque;

    virtual void operate() {

        jtSum = wamTorqueSumIn.getValue();
        gravity = wamGravityIn.getValue();
        std::cout << "is connected: " << wamTorqueSumIn.isConnected() << std::endl;
        externalTorque = jtSum + gravity - gravity;
        jtOutputValue->setData(&externalTorque);

        // std::cout << "gravity_inextTorque = [" << gravity << "]" << std::endl;
        std::cout << "jtsum_inextTorque = [" << jtSum << "]" << std::endl;
        // std::cout << "extTorque_inextTorque = [" << externalTorque << "]" << std::endl;

        // std::cout << "jtSum ptr: " << &jtSum << std::endl;
        // std::cout << "gravity ptr: " << &gravity << std::endl;
        

    }

  private:
    DISALLOW_COPY_AND_ASSIGN(ExternalTorque);
};