#include "stablecops/lely/MotorDriver.hpp"

#include <cstdlib>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <ostream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>

#include "stablecops/ds402/ObjectDictionary.hpp"

namespace stablecops::lely {

namespace {

std::ostream& writeHex(std::ostream& stream, uint32_t value, int width) {
    const auto flags = stream.flags();
    const auto fill = stream.fill();
    stream << "0x" << std::uppercase << std::hex
           << std::setw(width) << std::setfill('0') << value;
    stream.flags(flags);
    stream.fill(fill);
    return stream;
}

void writeObjectName(std::ostream& stream,
                     const char* name,
                     uint16_t index,
                     uint8_t subindex) {
    const auto flags = stream.flags();
    const auto fill = stream.fill();
    stream << "  " << name << " ";
    writeHex(stream, index, 4)
        << ':' << std::setw(2) << std::setfill('0') << std::uppercase
        << std::hex << static_cast<int>(subindex) << std::dec << " = ";
    stream.flags(flags);
    stream.fill(fill);
}

}  // namespace

MotorDriver::MotorDriver(::lely::canopen::AsyncMaster& master,
                         uint8_t node_id,
                         BootActionConfig boot_actions)
    : ::lely::canopen::FiberDriver(master, node_id),
      drive_(*this),
      boot_actions_(std::move(boot_actions)) {}

ds402::DriveController& MotorDriver::drive() {
    return drive_;
}

const ds402::DriveController& MotorDriver::drive() const {
    return drive_;
}

uint8_t MotorDriver::readU8(uint16_t index, uint8_t subindex) {
    return Wait(AsyncRead<uint8_t>(index, subindex));
}

uint16_t MotorDriver::readU16(uint16_t index, uint8_t subindex) {
    return Wait(AsyncRead<uint16_t>(index, subindex));
}

uint32_t MotorDriver::readU32(uint16_t index, uint8_t subindex) {
    return Wait(AsyncRead<uint32_t>(index, subindex));
}

int32_t MotorDriver::readI32(uint16_t index, uint8_t subindex) {
    return Wait(AsyncRead<int32_t>(index, subindex));
}

void MotorDriver::writeU8(uint16_t index, uint8_t subindex, uint8_t value) {
    Wait(AsyncWrite(index, subindex, value));
}

void MotorDriver::writeU16(uint16_t index, uint8_t subindex, uint16_t value) {
    Wait(AsyncWrite(index, subindex, value));
}

void MotorDriver::writeU32(uint16_t index, uint8_t subindex, uint32_t value) {
    Wait(AsyncWrite(index, subindex, value));
}

void MotorDriver::writeI32(uint16_t index, uint8_t subindex, int32_t value) {
    Wait(AsyncWrite(index, subindex, value));
}

void MotorDriver::OnBoot(::lely::canopen::NmtState state,
                         char error,
                         const std::string& reason) noexcept {
    if (error != 0) {
        std::cerr << "node " << static_cast<int>(id())
                  << " boot failed: " << reason << " (" << error << ")\n";
        return;
    }

    std::cout << "node " << static_cast<int>(id())
              << " booted, NMT state " << static_cast<int>(state) << '\n';

    if (boot_actions_.inspect) {
        inspectNode();
    }
    runBootActions();
}

void MotorDriver::inspectNode() noexcept {
    std::cout << "inspecting CANopen/DS402 objects\n";

    auto read_u8 = [this](uint16_t index, uint8_t subindex, const char* name) {
        std::error_code ec;
        const auto value = Wait(AsyncRead<uint8_t>(index, subindex), ec);
        writeObjectName(std::cout, name, index, subindex);
        if (ec) {
            std::cout << "FAILED: " << ec.message() << '\n';
        } else {
            writeHex(std::cout, value, 2) << " (" << static_cast<int>(value) << ")\n";
        }
        return std::pair<uint8_t, bool>{value, !ec};
    };

    auto read_u16 = [this](uint16_t index, uint8_t subindex, const char* name) {
        std::error_code ec;
        const auto value = Wait(AsyncRead<uint16_t>(index, subindex), ec);
        writeObjectName(std::cout, name, index, subindex);
        if (ec) {
            std::cout << "FAILED: " << ec.message() << '\n';
        } else {
            writeHex(std::cout, value, 4) << " (" << value << ")\n";
        }
        return std::pair<uint16_t, bool>{value, !ec};
    };

    auto read_u32 = [this](uint16_t index, uint8_t subindex, const char* name) {
        std::error_code ec;
        const auto value = Wait(AsyncRead<uint32_t>(index, subindex), ec);
        writeObjectName(std::cout, name, index, subindex);
        if (ec) {
            std::cout << "FAILED: " << ec.message() << '\n';
        } else {
            writeHex(std::cout, value, 8) << " (" << value << ")\n";
        }
        return std::pair<uint32_t, bool>{value, !ec};
    };

    auto read_i32 = [this](uint16_t index, uint8_t subindex, const char* name) {
        std::error_code ec;
        const auto value = Wait(AsyncRead<int32_t>(index, subindex), ec);
        writeObjectName(std::cout, name, index, subindex);
        if (ec) {
            std::cout << "FAILED: " << ec.message() << '\n';
        } else {
            writeHex(std::cout, static_cast<uint32_t>(value), 8) << " (" << value << ")\n";
        }
        return std::pair<int32_t, bool>{value, !ec};
    };

    std::cout << " identity\n";
    read_u32(0x1018, 0x01, "vendor id");
    read_u32(0x1018, 0x02, "product code");
    read_u32(0x1018, 0x03, "revision");
    read_u32(0x1018, 0x04, "serial number");

    std::cout << " DS402\n";
    const auto [statusword, status_ok] =
        read_u16(ds402::od::statusword, ds402::od::default_subindex, "statusword");
    if (status_ok) {
        std::cout << "  decoded state = "
                  << ds402::toString(ds402::decodeState(statusword)) << '\n';
    }

    read_u32(0x6502, 0x00, "supported modes");
    read_u8(ds402::od::modes_of_operation, ds402::od::default_subindex, "commanded mode");
    const auto [display_mode, display_mode_ok] = read_u8(
        ds402::od::modes_of_operation_display,
        ds402::od::default_subindex,
        "displayed mode");
    if (display_mode_ok) {
        const auto mode = static_cast<ds402::OperationMode>(
            static_cast<int8_t>(display_mode));
        std::cout << "  decoded mode = " << ds402::toString(mode) << '\n';
    }

    read_i32(ds402::od::position_actual_value, ds402::od::default_subindex, "position");
    read_i32(ds402::od::velocity_actual_value, ds402::od::default_subindex, "velocity");
    read_u16(ds402::od::torque_actual_value, ds402::od::default_subindex, "torque");
    read_u16(ds402::od::error_code, ds402::od::default_subindex, "error code");
}

void MotorDriver::runBootActions() noexcept {
    const bool wants_motion_action =
        boot_actions_.enable ||
        boot_actions_.hold_position ||
        boot_actions_.csp_target_position.has_value() ||
        boot_actions_.csp_relative_move.has_value();

    if (!wants_motion_action) {
        return;
    }

    try {
        enableDrive(boot_actions_.hold_position ||
                    boot_actions_.csp_target_position.has_value() ||
                    boot_actions_.csp_relative_move.has_value());
        applyCspTarget();
    } catch (const std::exception& exception) {
        std::cerr << "boot action failed: " << exception.what() << '\n';
    }
}

void MotorDriver::enableDrive(bool prime_csp_target) {
    auto feedback = drive_.readFeedback();
    std::cout << "enabling DS402 operation from state "
              << ds402::toString(feedback.state)
              << ", mode " << ds402::toString(feedback.mode) << '\n';

    if (prime_csp_target &&
        feedback.mode != ds402::OperationMode::CyclicSynchronousPosition) {
        throw std::runtime_error("CSP hold/target requires cyclic synchronous position mode");
    }

    if (prime_csp_target) {
        const auto position = drive_.primeCspTargetToCurrentPosition();
        std::cout << "  primed CSP target position to current position "
                  << position << '\n';
    }

    feedback = drive_.enableOperationSafely(boot_actions_.state_transition_timeout);
    std::cout << "  enabled operation, statusword=";
    writeHex(std::cout, feedback.statusword, 4)
        << " state=" << ds402::toString(feedback.state) << '\n';

    if (prime_csp_target) {
        const auto position = drive_.primeCspTargetToCurrentPosition();
        std::cout << "  holding current position at " << position << '\n';
    }
}

void MotorDriver::applyCspTarget() {
    if (!boot_actions_.csp_target_position && !boot_actions_.csp_relative_move) {
        return;
    }

    const auto feedback = drive_.readFeedback();
    if (feedback.state != ds402::State::OperationEnabled) {
        throw std::runtime_error("CSP target requires operation enabled");
    }
    if (feedback.mode != ds402::OperationMode::CyclicSynchronousPosition) {
        throw std::runtime_error("CSP target requires cyclic synchronous position mode");
    }

    int64_t target = feedback.position;
    if (boot_actions_.csp_target_position) {
        target = *boot_actions_.csp_target_position;
    }
    if (boot_actions_.csp_relative_move) {
        target += *boot_actions_.csp_relative_move;
    }

    if (target < std::numeric_limits<int32_t>::min() ||
        target > std::numeric_limits<int32_t>::max()) {
        throw std::runtime_error("requested CSP target is outside int32 range");
    }

    const auto delta = target - feedback.position;
    if (std::llabs(delta) > boot_actions_.max_position_step) {
        throw std::runtime_error("requested CSP step exceeds --max-position-step");
    }

    drive_.setCspTargetPosition(static_cast<int32_t>(target));
    std::cout << "  CSP target position set to " << target
              << " (delta " << delta << " counts)\n";
}

}  // namespace stablecops::lely
