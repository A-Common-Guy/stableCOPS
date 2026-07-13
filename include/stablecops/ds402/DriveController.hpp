#pragma once

#include <array>
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
    // DS402 error code (0x603F), streamed in the cyclic TPDO image.
    uint16_t error_code{0};
    // Last EMCY announced by the drive over the dedicated emergency channel
    // (COB 0x080+id): the emergency error code, the error register (0x1001) and
    // the five manufacturer-specific bytes. Surfaced independently of the TPDO
    // so an internally-latched fault is visible even while cyclic feedback keeps
    // flowing. Cleared when the drive announces "no error" (EMCY 0x0000) or on a
    // fault reset.
    uint16_t emergency_error_code{0};
    uint8_t error_register{0};
    std::array<uint8_t, 5> vendor_error{};
    // False once the master's consumer heartbeat (or node guarding) has lost the
    // node, independently of TPDO staleness.
    bool node_alive{true};
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
    // Contact detection: torque at/above threshold_torque while |velocity| is
    // at/below stopped_velocity (i.e. the axis is stalled, not just loaded).
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

// Thin, transport-independent facade over the DS402 object dictionary.
//
// This owns only what the runtime actually uses: reading a coherent feedback
// snapshot and staging the cyclic setpoints / zero / store commands. The DS402
// state machine itself (fault reset, the shutdown -> switch-on -> enable ladder,
// mode changes, and the Profile Position handshake) is driven non-blocking from
// the cyclic path in stablecops::lely::MotorDriver, not here, so no blocking
// controlword transitions live on this class.
class DriveController {
public:
    explicit DriveController(ObjectAccess& object_access);

    Feedback readFeedback();
    OperationMode readMode();

    void setCspTargetPosition(int32_t target_position);
    void setCsvTargetVelocity(int32_t target_velocity);
    void setCstTargetTorque(int16_t target_torque);
    void setCurrentPositionAsZero();
    void storeApplicationParameters();

private:
    ObjectAccess& object_access_;

    uint16_t readStatusword();
};

}  // namespace stablecops::ds402
