#include "stablecops/ds402/DriveController.hpp"

#include <chrono>
#include <cstdlib>
#include <sstream>
#include <stdexcept>
#include <thread>

#include "stablecops/ds402/ObjectDictionary.hpp"

namespace stablecops::ds402 {

DriveController::DriveController(ObjectAccess& object_access)
    : object_access_(object_access) {}

Feedback DriveController::readFeedback() {
    Feedback feedback;
    feedback.statusword = readStatusword();
    feedback.state = decodeState(feedback.statusword);
    feedback.mode = readMode();
    feedback.position = object_access_.readI32(od::position_actual_value, od::default_subindex);
    feedback.velocity = object_access_.readI32(od::velocity_actual_value, od::default_subindex);
    feedback.torque = static_cast<int16_t>(
        object_access_.readU16(od::torque_actual_value, od::default_subindex));
    feedback.error_code = object_access_.readU16(od::error_code, od::default_subindex);
    return feedback;
}

void DriveController::resetFault() {
    writeControlword(controlword::fault_reset);
}

void DriveController::shutdown() {
    writeControlword(controlword::shutdown);
}

void DriveController::switchOn() {
    writeControlword(controlword::switch_on);
}

void DriveController::enableOperation() {
    writeControlword(controlword::enable_operation);
}

void DriveController::disableVoltage() {
    writeControlword(controlword::disable_voltage);
}

void DriveController::quickStop() {
    writeControlword(controlword::quick_stop);
}

void DriveController::halt() {
    writeControlword(controlword::halt);
}

Feedback DriveController::waitForState(State expected,
                                       std::chrono::milliseconds timeout,
                                       std::chrono::milliseconds poll_interval) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    Feedback feedback = readFeedback();

    while (true) {
        if (feedback.state == expected) {
            return feedback;
        }
        if (feedback.state == State::Fault || feedback.state == State::FaultReactionActive) {
            throw std::runtime_error("drive entered fault state while waiting for " +
                                     toString(expected));
        }
        if (feedback.error_code != 0) {
            std::ostringstream message;
            message << "drive reported error code 0x" << std::hex << feedback.error_code
                    << " while waiting for " << toString(expected);
            throw std::runtime_error(message.str());
        }
        if (std::chrono::steady_clock::now() >= deadline) {
            throw std::runtime_error("timed out waiting for " + toString(expected) +
                                     "; last state was " + toString(feedback.state));
        }

        std::this_thread::sleep_for(poll_interval);
        feedback = readFeedback();
    }
}

Feedback DriveController::enableOperationSafely(std::chrono::milliseconds timeout) {
    auto feedback = readFeedback();
    if (feedback.state == State::Fault || feedback.state == State::FaultReactionActive) {
        throw std::runtime_error("refusing to enable while the drive is in fault");
    }
    if (feedback.error_code != 0) {
        std::ostringstream message;
        message << "refusing to enable with drive error code 0x" << std::hex
                << feedback.error_code;
        throw std::runtime_error(message.str());
    }
    if (feedback.state == State::OperationEnabled) {
        return feedback;
    }

    shutdown();
    feedback = waitForState(State::ReadyToSwitchOn, timeout);

    switchOn();
    feedback = waitForState(State::SwitchedOn, timeout);

    enableOperation();
    return waitForState(State::OperationEnabled, timeout);
}

int32_t DriveController::primeCspTargetToCurrentPosition() {
    const auto position =
        object_access_.readI32(od::position_actual_value, od::default_subindex);
    setCspTargetPosition(position);
    return position;
}

uint32_t DriveController::readSupportedModes() {
    return object_access_.readU32(0x6502, od::default_subindex);
}

void DriveController::requestMode(OperationMode mode) {
    object_access_.writeU8(
        od::modes_of_operation,
        od::default_subindex,
        static_cast<uint8_t>(mode));
}

OperationMode DriveController::readMode() {
    return static_cast<OperationMode>(
        static_cast<int8_t>(object_access_.readU8(
            od::modes_of_operation_display,
            od::default_subindex)));
}

void DriveController::switchModeSafely(OperationMode mode, int32_t stationary_velocity_threshold) {
    const auto feedback = readFeedback();
    if (feedback.state != State::OperationEnabled) {
        throw std::logic_error("mode changes are allowed only while operation is enabled");
    }
    if (std::abs(feedback.velocity) > stationary_velocity_threshold) {
        throw std::logic_error("mode changes are allowed only while the drive is stationary");
    }

    if (mode == OperationMode::ProfilePosition ||
        mode == OperationMode::CyclicSynchronousPosition) {
        object_access_.writeI32(od::target_position, od::default_subindex, feedback.position);
    }

    requestMode(mode);
}

void DriveController::setProfilePosition(int32_t target_position,
                                         uint32_t velocity,
                                         uint32_t acceleration,
                                         uint32_t deceleration) {
    object_access_.writeI32(od::target_position, od::default_subindex, target_position);
    object_access_.writeU32(
        od::profile_velocity,
        od::default_subindex,
        velocity);
    object_access_.writeU32(
        od::profile_acceleration,
        od::default_subindex,
        acceleration);
    object_access_.writeU32(
        od::profile_deceleration,
        od::default_subindex,
        deceleration);
}

void DriveController::triggerAbsoluteProfilePosition() {
    writeControlword(controlword::enable_operation);
    writeControlword(controlword::enable_operation | controlword::new_setpoint);
}

void DriveController::triggerRelativeProfilePosition() {
    writeControlword(controlword::enable_operation | controlword::relative_position);
    writeControlword(controlword::enable_operation |
                     controlword::relative_position |
                     controlword::new_setpoint);
}

void DriveController::setCspTargetPosition(int32_t target_position) {
    object_access_.writeI32(od::target_position, od::default_subindex, target_position);
}

void DriveController::setCsvTargetVelocity(int32_t target_velocity) {
    object_access_.writeI32(od::target_velocity, od::default_subindex, target_velocity);
}

void DriveController::setCstTargetTorque(int16_t target_torque) {
    object_access_.writeU16(
        od::target_torque,
        od::default_subindex,
        static_cast<uint16_t>(target_torque));
}

void DriveController::setCurrentPositionAsZero() {
    object_access_.writeI32(od::set_current_position_zero, od::default_subindex, 1);
}

void DriveController::storeApplicationParameters() {
    object_access_.writeU32(od::store_parameters,
                            od::store_application_parameters_subindex,
                            od::store_parameters_signature);
}

void DriveController::stopCsvMotion() {
    setCsvTargetVelocity(0);
}

uint16_t DriveController::readStatusword() {
    return object_access_.readU16(od::statusword, od::default_subindex);
}

void DriveController::writeControlword(uint16_t value) {
    object_access_.writeU16(od::controlword, od::default_subindex, value);
}

}  // namespace stablecops::ds402
