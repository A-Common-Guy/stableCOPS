#include "stablecops/ds402/State.hpp"

namespace stablecops::ds402 {

State decodeState(uint16_t statusword) {
    if ((statusword & 0x004F) == 0x0000) {
        return State::NotReadyToSwitchOn;
    }
    if ((statusword & 0x004F) == 0x0040) {
        return State::SwitchOnDisabled;
    }
    if ((statusword & 0x006F) == 0x0021) {
        return State::ReadyToSwitchOn;
    }
    if ((statusword & 0x006F) == 0x0023) {
        return State::SwitchedOn;
    }
    if ((statusword & 0x006F) == 0x0027) {
        return State::OperationEnabled;
    }
    if ((statusword & 0x006F) == 0x0007) {
        return State::QuickStopActive;
    }
    if ((statusword & 0x004F) == 0x000F) {
        return State::FaultReactionActive;
    }
    if ((statusword & 0x004F) == 0x0008) {
        return State::Fault;
    }
    return State::Unknown;
}

bool isFault(uint16_t statusword) {
    const auto state = decodeState(statusword);
    return state == State::Fault || state == State::FaultReactionActive;
}

bool isOperationEnabled(uint16_t statusword) {
    return decodeState(statusword) == State::OperationEnabled;
}

std::string toString(State state) {
    switch (state) {
        case State::NotReadyToSwitchOn:
            return "not ready to switch on";
        case State::SwitchOnDisabled:
            return "switch on disabled";
        case State::ReadyToSwitchOn:
            return "ready to switch on";
        case State::SwitchedOn:
            return "switched on";
        case State::OperationEnabled:
            return "operation enabled";
        case State::QuickStopActive:
            return "quick stop active";
        case State::FaultReactionActive:
            return "fault reaction active";
        case State::Fault:
            return "fault";
        case State::Unknown:
            return "unknown";
    }
    return "unknown";
}

std::string toString(OperationMode mode) {
    switch (mode) {
        case OperationMode::ProfilePosition:
            return "profile position";
        case OperationMode::ProfileVelocity:
            return "profile velocity";
        case OperationMode::ProfileTorque:
            return "profile torque";
        case OperationMode::CyclicSynchronousPosition:
            return "cyclic synchronous position";
        case OperationMode::CyclicSynchronousVelocity:
            return "cyclic synchronous velocity";
        case OperationMode::CyclicSynchronousTorque:
            return "cyclic synchronous torque";
    }
    return "unknown";
}

}  // namespace stablecops::ds402
