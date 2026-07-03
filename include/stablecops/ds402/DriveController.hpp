#pragma once

#include <chrono>
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

struct HomingConfig {
    int32_t search_velocity{25000};
    int32_t approach_velocity{24000};
    int32_t center_velocity{24000};
    int32_t center_final_velocity{4000};
    int32_t center_slowdown_distance{1000};
    int32_t backoff_distance{2000};
    int32_t center_tolerance{50};
    int32_t center_settle_tolerance{200};
    int32_t min_travel{1000};
    int32_t max_travel{2000000};
    int32_t home_offset{0};
    int16_t threshold_torque{90};
    int32_t stopped_velocity{200};
    std::chrono::milliseconds contact_dwell{20};
    std::chrono::milliseconds settle_time{200};
    std::chrono::milliseconds timeout{30000};
    bool save_zero_to_nvm{true};
};

struct HomingResult {
    int32_t lower_limit_position{0};
    int32_t upper_limit_position{0};
    int32_t center_position{0};
    int32_t travel{0};
    bool success{false};
};

enum class HomingPhase : uint8_t {
    Idle,
    SearchNegative,
    BackoffNegative,
    SearchPositive,
    MoveToCenter,
    WaitAtCenter,
    ZeroAtCenter,
    RestoreDisable,
    RestoreMode,
    RestoreEnable,
    Done,
    Failed,
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

    Feedback waitForState(State expected, std::chrono::milliseconds timeout,
                          std::chrono::milliseconds poll_interval = std::chrono::milliseconds{20});
    Feedback enableOperationSafely(std::chrono::milliseconds timeout = std::chrono::milliseconds{
                                       2000});
    int32_t primeCspTargetToCurrentPosition();
    uint32_t readSupportedModes();
    void requestMode(OperationMode mode);

    void switchModeSafely(OperationMode mode, int32_t stationary_velocity_threshold = 0);
    OperationMode readMode();

    void setProfilePosition(int32_t target_position, uint32_t velocity, uint32_t acceleration,
                            uint32_t deceleration);
    void triggerAbsoluteProfilePosition();
    void triggerRelativeProfilePosition();

    void setCspTargetPosition(int32_t target_position);
    void setCsvTargetVelocity(int32_t target_velocity);
    void setCstTargetTorque(int16_t target_torque);
    void setCurrentPositionAsZero();
    void storeApplicationParameters();
    void stopCsvMotion();

private:
    ObjectAccess& object_access_;

    uint16_t readStatusword();
    void writeControlword(uint16_t value);
};

}  // namespace stablecops::ds402
