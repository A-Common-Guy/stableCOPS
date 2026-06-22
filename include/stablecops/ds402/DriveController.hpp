#pragma once

#include <cstdint>

#include "stablecops/ds402/ObjectAccess.hpp"
#include "stablecops/ds402/State.hpp"

namespace stablecops::ds402 {

struct Feedback {
    uint16_t statusword{0};
    State state{State::Unknown};
    OperationMode mode{OperationMode::CyclicSynchronousPosition};
    int32_t position{0};
    int32_t velocity{0};
    int16_t torque{0};
    uint16_t error_code{0};
};

struct MitCommandRaw {
    uint32_t parameter_0{0};
    uint32_t parameter_1{0};
    uint32_t parameter_2{0};
    uint32_t parameter_3{0};
};

class DriveController {
public:
    explicit DriveController(ObjectAccess& object_access);

    Feedback readFeedback();

    void resetFault();
    void shutdown();
    void switchOn();
    void enableOperation();
    void disableVoltage();
    void quickStop();
    void halt();

    void switchModeSafely(OperationMode mode, int32_t stationary_velocity_threshold = 0);
    OperationMode readMode();

    void setProfilePosition(int32_t target_position,
                            uint32_t velocity,
                            uint32_t acceleration,
                            uint32_t deceleration);
    void triggerAbsoluteProfilePosition();
    void triggerRelativeProfilePosition();

    void setCspTargetPosition(int32_t target_position);
    void setCsvTargetVelocity(int32_t target_velocity);
    void setCstTargetTorque(int16_t target_torque);
    void setMitCommandRaw(const MitCommandRaw& command);
    void setCurrentPositionAsZero();

private:
    ObjectAccess& object_access_;

    uint16_t readStatusword();
    void writeControlword(uint16_t value);
    void setModeUnsafe(OperationMode mode);
};

}  // namespace stablecops::ds402
