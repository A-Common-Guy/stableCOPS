#pragma once

#include <cstdint>
#include <string>

namespace stablecops::ds402 {

enum class State {
    NotReadyToSwitchOn,
    SwitchOnDisabled,
    ReadyToSwitchOn,
    SwitchedOn,
    OperationEnabled,
    QuickStopActive,
    FaultReactionActive,
    Fault,
    Unknown,
};

enum class OperationMode : int8_t {
    ProfilePosition = 1,
    ProfileVelocity = 3,
    ProfileTorque = 4,
    CyclicSynchronousPosition = 8,
    CyclicSynchronousVelocity = 9,
    CyclicSynchronousTorque = 10,
};

namespace controlword {

constexpr uint16_t shutdown = 0x0006;
constexpr uint16_t switch_on = 0x0007;
constexpr uint16_t enable_operation = 0x000F;
constexpr uint16_t disable_voltage = 0x0000;
constexpr uint16_t quick_stop = 0x0002;
constexpr uint16_t fault_reset = 0x0080;
constexpr uint16_t halt = 0x010F;

constexpr uint16_t new_setpoint = 0x0010;
constexpr uint16_t change_immediately = 0x0020;
constexpr uint16_t relative_position = 0x0040;

}  // namespace controlword

State decodeState(uint16_t statusword);
bool isFault(uint16_t statusword);
bool isOperationEnabled(uint16_t statusword);
std::string toString(State state);
std::string toString(OperationMode mode);

}  // namespace stablecops::ds402
