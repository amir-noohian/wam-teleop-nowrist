#pragma once

#include <barrett/detail/ca_macro.h>
#include <barrett/systems.h>
#include <barrett/units.h>

template <size_t DOF>
class OneStepDelay : public barrett::systems::System {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
  public:
    Input<jt_type> input;  // not templated!
    Output<jt_type> output;

    explicit OneStepDelay(barrett::systems::ExecutionManager* em, const std::string& sysName = "OneStepDelay")
        : System(sysName)
        , input(this)
        , output(this, &outValue)  // now outValue is a member (see below)
    {
        prev_value.setZero();
        stored_value.setZero();
        if (em != NULL) {
            em->startManaging(*this);
        }
    }

    virtual ~OneStepDelay() {
        this->mandatoryCleanUp();
    }

  protected:
    typename Output<jt_type>::Value* outValue;
    jt_type prev_value;

    virtual void operate() override {
        outValue->setData(&stored_value);
        stored_value = prev_value;
        prev_value = input.getValue();

        std::cout << "jtSum defined: " << input.valueDefined() << std::endl;
    }

  private:
    DISALLOW_COPY_AND_ASSIGN(OneStepDelay);
    jt_type stored_value; 

};
