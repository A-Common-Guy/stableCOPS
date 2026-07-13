#include "stablecops/lely/MotorDriver.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>

#include "stablecops/ds402/Diagnostics.hpp"
#include "stablecops/ds402/ObjectDictionary.hpp"
#include "stablecops/log/Log.hpp"

namespace stablecops::lely {

namespace {

// PDO mapping entry word as written to 0x16xx/0x1Axx sub-indices:
// index<<16 | subindex<<8 | bit length.
uint32_t mappingEntryWord(const config::PdoMappedObject& object) {
    return (static_cast<uint32_t>(object.index) << 16) |
           (static_cast<uint32_t>(object.subindex) << 8) | static_cast<uint32_t>(object.bit_length);
}

constexpr uint32_t kCobIdValidMask = 0x80000000u;

// Objects whose CANopen type is signed. Everything else PDO-mappable here
// (controlword, statusword, error code, profile/torque limits) is unsigned. Used
// together with the mapped bit length to pick the wire type.
bool isSignedObject(uint16_t index) {
    switch (index) {
        case ds402::od::modes_of_operation:
        case ds402::od::modes_of_operation_display:
        case ds402::od::position_actual_value:
        case ds402::od::velocity_actual_value:
        case ds402::od::target_position:
        case ds402::od::target_velocity:
        case ds402::od::target_torque:
        case ds402::od::torque_actual_value:
            return true;
        default:
            return false;
    }
}

ds402::ObjectWidth objectTypeFor(uint16_t index, uint8_t bit_length) {
    using ds402::ObjectWidth;
    const bool is_signed = isSignedObject(index);
    switch (bit_length) {
        case 8:
            return is_signed ? ObjectWidth::I8 : ObjectWidth::U8;
        case 16:
            return is_signed ? ObjectWidth::I16 : ObjectWidth::U16;
        case 32:
        default:
            return is_signed ? ObjectWidth::I32 : ObjectWidth::U32;
    }
}

// How often to log progress while waiting for the drive to confirm it has
// dropped its power stage during shutdown.
constexpr auto kPowerOffRetryLogInterval = std::chrono::milliseconds(200);

// The manual (3.3) only allows a mode switch in the enabled *stationary* state.
// Encoder noise means the measured velocity is rarely exactly zero, so
// "stationary" is anything below this threshold (drive velocity units).
constexpr int32_t kModeSwitchStationaryVelocity = 200;

std::ostream& writeHex(std::ostream& stream, uint32_t value, int width) {
    const auto flags = stream.flags();
    const auto fill = stream.fill();
    stream << "0x" << std::uppercase << std::hex << std::setw(width) << std::setfill('0') << value;
    stream.flags(flags);
    stream.fill(fill);
    return stream;
}

void writeObjectName(std::ostream& stream, const char* name, uint16_t index, uint8_t subindex) {
    const auto flags = stream.flags();
    const auto fill = stream.fill();
    stream << "  " << name << " ";
    writeHex(stream, index, 4) << ':' << std::setw(2) << std::setfill('0') << std::uppercase
                               << std::hex << static_cast<int>(subindex) << std::dec << " = ";
    stream.flags(flags);
    stream.fill(fill);
}

}  // namespace

MotorDriver::MotorDriver(::lely::canopen::AsyncMaster& master, uint8_t node_id,
                         BootActionConfig boot_actions, config::PdoMap pdo_map)
    : ::lely::canopen::FiberDriver(master, node_id),
      drive_(*this),
      boot_actions_(std::move(boot_actions)),
      pdo_map_(std::move(pdo_map)) {
    buildCyclicObjects();
}

void MotorDriver::buildCyclicObjects() {
    // Snapshot the active RxPDO / TxPDO objects once so the cyclic path is a
    // plain iteration with no per-cycle allocation or PDO-layout knowledge.
    const auto collect = [](const std::vector<config::PdoChannel>& channels,
                            std::vector<CyclicObject>& out) {
        for (const auto& channel : channels) {
            if (!channel.active()) {
                continue;
            }
            for (const auto& entry : channel.entries) {
                CyclicObject object;
                object.index = entry.index;
                object.subindex = entry.subindex;
                object.type = objectTypeFor(entry.index, entry.bit_length);
                out.push_back(object);
            }
        }
    };
    collect(pdo_map_.rpdo, command_objects_);
    collect(pdo_map_.tpdo, feedback_objects_);
}

CyclicObject* MotorDriver::findCommand(uint16_t index) {
    for (auto& object : command_objects_) {
        if (object.index == index) {
            return &object;
        }
    }
    return nullptr;
}

const CyclicObject* MotorDriver::findCommand(uint16_t index) const {
    for (const auto& object : command_objects_) {
        if (object.index == index) {
            return &object;
        }
    }
    return nullptr;
}

bool MotorDriver::hasCommand(uint16_t index) const {
    return findCommand(index) != nullptr;
}

int64_t MotorDriver::commandValue(uint16_t index) const {
    const auto* object = findCommand(index);
    return object ? object->value : 0;
}

void MotorDriver::setCommandValue(uint16_t index, int64_t value) {
    if (auto* object = findCommand(index)) {
        object->value = value;
    }
}

int64_t MotorDriver::readMappedObject(const CyclicObject& object, std::error_code& ec) {
    // Feedback objects are the drive's TxPDOs, which Lely surfaces to the master
    // through rpdo_mapped. Reading here never hits the bus; it returns the value
    // from the last received frame.
    auto sub = rpdo_mapped[object.index][object.subindex];
    switch (object.type) {
        case ds402::ObjectWidth::U8:
            return sub.Read<uint8_t>(ec);
        case ds402::ObjectWidth::I8:
            return sub.Read<int8_t>(ec);
        case ds402::ObjectWidth::U16:
            return sub.Read<uint16_t>(ec);
        case ds402::ObjectWidth::I16:
            return sub.Read<int16_t>(ec);
        case ds402::ObjectWidth::U32:
            return sub.Read<uint32_t>(ec);
        case ds402::ObjectWidth::I32:
            return sub.Read<int32_t>(ec);
    }
    return 0;
}

void MotorDriver::writeMappedObject(const CyclicObject& object, std::error_code& ec) {
    // Command objects are the drive's RxPDOs, which the master transmits through
    // tpdo_mapped. Writing here stages the value into the outgoing frame emitted
    // on this SYNC.
    auto sub = tpdo_mapped[object.index][object.subindex];
    switch (object.type) {
        case ds402::ObjectWidth::U8:
            sub.Write(static_cast<uint8_t>(object.value), ec);
            return;
        case ds402::ObjectWidth::I8:
            sub.Write(static_cast<int8_t>(object.value), ec);
            return;
        case ds402::ObjectWidth::U16:
            sub.Write(static_cast<uint16_t>(object.value), ec);
            return;
        case ds402::ObjectWidth::I16:
            sub.Write(static_cast<int16_t>(object.value), ec);
            return;
        case ds402::ObjectWidth::U32:
            sub.Write(static_cast<uint32_t>(object.value), ec);
            return;
        case ds402::ObjectWidth::I32:
            sub.Write(static_cast<int32_t>(object.value), ec);
            return;
    }
}

void MotorDriver::decodeFeedbackObject(uint16_t index, int64_t raw) {
    switch (index) {
        case ds402::od::statusword:
            feedback_.statusword = static_cast<uint16_t>(raw);
            feedback_.state = ds402::decodeState(feedback_.statusword);
            break;
        case ds402::od::modes_of_operation_display:
            feedback_.mode = static_cast<ds402::OperationMode>(static_cast<int8_t>(raw));
            break;
        case ds402::od::position_actual_value:
            feedback_.position = static_cast<int32_t>(raw);
            break;
        case ds402::od::velocity_actual_value:
            feedback_.velocity = static_cast<int32_t>(raw);
            break;
        case ds402::od::torque_actual_value:
            feedback_.torque = static_cast<int16_t>(raw);
            break;
        case ds402::od::error_code:
            feedback_.error_code = static_cast<uint16_t>(raw);
            break;
        default:
            break;  // mapped but not a named field of ds402::Feedback
    }
}

void MotorDriver::publishFeedback() {
    // One lock publishes both snapshots so readers always see a coherent pair
    // and the cyclic path takes the mutex only once per event.
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    feedback_published_ = feedback_;
    stats_published_ = stats_;
}

bool MotorDriver::feedbackStale(std::chrono::steady_clock::time_point now) const {
    // Disabled, or no feedback received yet (nothing to be stale about).
    if (boot_actions_.feedback_timeout.count() <= 0 || !rpdo_seen_) {
        return false;
    }
    return (now - last_feedback_time_) > boot_actions_.feedback_timeout;
}

void MotorDriver::logFaultTransition() {
    const bool in_fault = feedback_.state == ds402::State::Fault ||
                          feedback_.state == ds402::State::FaultReactionActive ||
                          feedback_.error_code != 0 || feedback_.emergency_error_code != 0;
    if (in_fault && !fault_active_logged_) {
        // Prefer the code that is actually set: the EMCY emergency code and the
        // TPDO 0x603F share the manual's fault-code space (Table 4-2).
        const uint16_t fault_code =
            feedback_.error_code != 0 ? feedback_.error_code : feedback_.emergency_error_code;
        stablecops::log::err() << "drive FAULT: state=" << ds402::toString(feedback_.state)
                               << " statusword=";
        writeHex(stablecops::log::err(), feedback_.statusword, 4) << " error_code=";
        writeHex(stablecops::log::err(), feedback_.error_code, 4) << " emcy=";
        writeHex(stablecops::log::err(), feedback_.emergency_error_code, 4) << " error_register=";
        writeHex(stablecops::log::err(), feedback_.error_register, 2)
            << " (" << ds402::describeDeviceFault(fault_code)
            << "; register: " << ds402::describeErrorRegister(feedback_.error_register) << ")\n";
        fault_active_logged_ = true;
    } else if (!in_fault && fault_active_logged_) {
        stablecops::log::out() << "drive fault cleared (state=" << ds402::toString(feedback_.state)
                               << ")\n";
        fault_active_logged_ = false;
    }
}

ds402::Feedback MotorDriver::feedbackSnapshot() const {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    return feedback_published_;
}

bool MotorDriver::feedbackLive() const {
    return feedback_live_.load(std::memory_order_acquire);
}

ds402::HomingPhase MotorDriver::homingPhase() const {
    return homing_phase_;
}

ds402::HomingResult MotorDriver::homingResult() const {
    return homing_result_;
}

CyclicStats MotorDriver::cyclicStats() const {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    return stats_published_;
}

void MotorDriver::updateCyclicStats(std::chrono::steady_clock::time_point now) {
    // The first SYNC only seeds the reference timestamp; intervals start at the
    // second one.
    if (last_sync_time_.time_since_epoch().count() != 0) {
        const double interval_us =
            std::chrono::duration<double, std::micro>(now - last_sync_time_).count();
        stats_.cycles += 1;
        stats_.last_us = interval_us;
        if (stats_.cycles == 1) {
            stats_.min_us = interval_us;
            stats_.max_us = interval_us;
        } else {
            stats_.min_us = std::min(stats_.min_us, interval_us);
            stats_.max_us = std::max(stats_.max_us, interval_us);
        }
        interval_sum_us_ += interval_us;
        stats_.mean_us = interval_sum_us_ / static_cast<double>(stats_.cycles);
        const double nominal_us = static_cast<double>(boot_actions_.sync_period_us);
        if (nominal_us > 0.0) {
            stats_.max_jitter_us =
                std::max(stats_.max_jitter_us, std::abs(interval_us - nominal_us));
        }
    }
    last_sync_time_ = now;
}

ds402::DriveController& MotorDriver::drive() {
    return drive_;
}

const ds402::DriveController& MotorDriver::drive() const {
    return drive_;
}

bool MotorDriver::isPdoOnlyCommand(uint16_t index) const {
    // Only the controlword and the motion targets abort SDO downloads on this
    // firmware. Profile parameters (0x6081/0x6083/0x6084/0x6087) are ordinary
    // configuration objects and must reach the drive over SDO when unmapped.
    switch (index) {
        case ds402::od::controlword:
        case ds402::od::target_position:
        case ds402::od::target_velocity:
        case ds402::od::target_torque:
            return true;
        default:
            return false;
    }
}

bool MotorDriver::isFeedbackObject(uint16_t index) const {
    switch (index) {
        case ds402::od::statusword:
        case ds402::od::modes_of_operation_display:
        case ds402::od::position_actual_value:
        case ds402::od::velocity_actual_value:
        case ds402::od::torque_actual_value:
            return true;
        default:
            return false;
    }
}

bool MotorDriver::feedbackMapped(uint16_t index) const {
    return std::any_of(feedback_objects_.begin(), feedback_objects_.end(),
                       [index](const CyclicObject& object) { return object.index == index; });
}

// Reads of objects that ride in an active TxPDO are served from the cached
// snapshot refreshed every SYNC (no bus round trip); everything else, and any
// object not actually mapped, falls back to a blocking SDO read.
uint8_t MotorDriver::readU8(uint16_t index, uint8_t subindex) {
    if (rpdo_seen_ && index == ds402::od::modes_of_operation_display && feedbackMapped(index)) {
        return static_cast<uint8_t>(static_cast<int8_t>(feedback_.mode));
    }
    return Wait(AsyncRead<uint8_t>(index, subindex));
}

uint16_t MotorDriver::readU16(uint16_t index, uint8_t subindex) {
    if (rpdo_seen_ && feedbackMapped(index)) {
        if (index == ds402::od::statusword) {
            return feedback_.statusword;
        }
        if (index == ds402::od::torque_actual_value) {
            return static_cast<uint16_t>(feedback_.torque);
        }
        // error_code (0x603F) is refreshed from the TPDO by decodeFeedbackObject
        // when the profile maps it, so serve it from the snapshot too instead of
        // an SDO read on every feedback poll.
        if (index == ds402::od::error_code) {
            return feedback_.error_code;
        }
    }
    return Wait(AsyncRead<uint16_t>(index, subindex));
}

uint32_t MotorDriver::readU32(uint16_t index, uint8_t subindex) {
    return Wait(AsyncRead<uint32_t>(index, subindex));
}

int32_t MotorDriver::readI32(uint16_t index, uint8_t subindex) {
    if (rpdo_seen_ && feedbackMapped(index)) {
        if (index == ds402::od::position_actual_value) {
            return feedback_.position;
        }
        if (index == ds402::od::velocity_actual_value) {
            return feedback_.velocity;
        }
    }
    return Wait(AsyncRead<int32_t>(index, subindex));
}

// Command writes are staged, not sent immediately: if the object rides in an
// active RxPDO its value is buffered and streamed on the next SYNC; if it is a
// PDO-only command object that is not currently mapped it is dropped with a
// warning (an SDO download would abort with 0x00000002); anything else falls
// back to a blocking SDO write (configuration objects).
template <typename T>
void MotorDriver::writeStagedOrSdo(uint16_t index, uint8_t subindex, T value) {
    if (auto* object = findCommand(index)) {
        object->value = static_cast<int64_t>(value);
        return;
    }
    if (isPdoOnlyCommand(index)) {
        stablecops::log::warn() << "write to PDO-only object ";
        writeHex(stablecops::log::warn(), index, 4)
            << " dropped: it is not mapped into an active RxPDO\n";
        return;
    }
    Wait(AsyncWrite(index, subindex, value));
}

void MotorDriver::writeU8(uint16_t index, uint8_t subindex, uint8_t value) {
    writeStagedOrSdo(index, subindex, value);
}

void MotorDriver::writeU16(uint16_t index, uint8_t subindex, uint16_t value) {
    writeStagedOrSdo(index, subindex, value);
}

void MotorDriver::writeU32(uint16_t index, uint8_t subindex, uint32_t value) {
    writeStagedOrSdo(index, subindex, value);
}

void MotorDriver::writeI32(uint16_t index, uint8_t subindex, int32_t value) {
    writeStagedOrSdo(index, subindex, value);
}

void MotorDriver::OnBoot(::lely::canopen::NmtState state, char error,
                         const std::string& reason) noexcept {
    if (error != 0) {
        stablecops::log::err() << "node " << static_cast<int>(id()) << " boot failed: " << reason
                               << " (" << error << ")\n";
        return;
    }

    stablecops::log::out() << "node " << static_cast<int>(id()) << " booted, NMT state "
                           << static_cast<int>(state) << '\n';

    if (boot_actions_.inspect) {
        inspectNode();
    }
    runBootActions();
}

void MotorDriver::OnConfig(std::function<void(std::error_code)> result) noexcept {
    // The 'update configuration' step runs while the node is still
    // pre-operational, which is the only window in which this drive accepts SDO
    // writes to its command objects and PDO parameters (writing them once the
    // node is operational aborts with 0x00000002). We only reconfigure when a
    // motion action is requested so that --inspect stays read-only.
    //
    // Lely requires OnConfig() itself to be non-blocking. The SDO helpers below
    // use FiberDriver::Wait(), so run them on this driver's fiber executor and
    // report completion asynchronously.
    Defer([this, result = std::move(result)]() mutable {
        std::error_code ec;
        if (wantsCyclicConfig()) {
            ec = configurePdos();
            if (!ec) {
                ec = selectOperationMode();
            }
            if (!ec) {
                ec = configureProfileParameters();
            }
            if (ec) {
                stablecops::log::err() << "node " << static_cast<int>(id())
                                       << " configuration failed: " << ec.message() << '\n';
            }
        }
        result(ec);
    });
}

std::error_code MotorDriver::selectOperationMode() noexcept {
    // Select the operation mode by writing 0x6060 over SDO while the node is
    // still pre-operational (manual: 0x6060 is SDO-writable in this state; the
    // PDO-only abort 0x00000002 applies to the target/controlword objects, not
    // to mode selection). One fixed PDO layout serves CSP/CSV/CST, so this is
    // the only object that changes between cyclic modes. Left untouched when no
    // mode is requested, preserving the drive's persisted mode.
    if (!boot_actions_.mode) {
        return {};
    }

    const auto mode = *boot_actions_.mode;
    std::error_code ec;
    Wait(AsyncWrite(ds402::od::modes_of_operation, ds402::od::default_subindex,
                    static_cast<int8_t>(mode)),
         ec);
    if (ec) {
        stablecops::log::err() << "  SDO write to modes of operation (0x6060:00) failed: "
                               << ec.message() << '\n';
        return ec;
    }
    stablecops::log::out() << "  operation mode set to " << ds402::toString(mode)
                           << " (0x6060=" << static_cast<int>(static_cast<int8_t>(mode)) << ")\n";
    return {};
}

std::error_code MotorDriver::configureProfileParameters() noexcept {
    // Profile-mode parameters are ordinary configuration objects (unlike the
    // PDO-only target/controlword), so they are SDO-writable while the node is
    // pre-operational. Each is written only when the caller provided it; the
    // drive's persisted value is otherwise left in place.
    std::error_code ec;
    const auto write = [&](const std::optional<uint32_t>& value, uint16_t index,
                           const char* what) -> bool {
        if (!value) {
            return true;
        }
        Wait(AsyncWrite(index, ds402::od::default_subindex, *value), ec);
        if (ec) {
            stablecops::log::err() << "  SDO write to " << what << " (";
            writeHex(stablecops::log::err(), index, 4) << ":00) failed: " << ec.message() << '\n';
            return false;
        }
        stablecops::log::out() << "  " << what << " set to " << *value << '\n';
        return true;
    };

    if (!write(boot_actions_.profile_velocity, ds402::od::profile_velocity, "profile velocity")) {
        return ec;
    }
    if (!write(boot_actions_.profile_acceleration, ds402::od::profile_acceleration,
               "profile acceleration")) {
        return ec;
    }
    if (!write(boot_actions_.profile_deceleration, ds402::od::profile_deceleration,
               "profile deceleration")) {
        return ec;
    }
    if (!write(boot_actions_.torque_slope, ds402::od::torque_slope, "torque slope")) {
        return ec;
    }

    // Vendor "Disable Mode" (0x2103) is a single-byte object, so it must be
    // written as u8 (a u32 download aborts on length mismatch). Selects the
    // power-stage behaviour on disable (coast vs short-circuit braking).
    if (boot_actions_.disable_mode) {
        Wait(AsyncWrite(ds402::od::disable_mode, ds402::od::default_subindex,
                        static_cast<uint8_t>(*boot_actions_.disable_mode)),
             ec);
        if (ec) {
            stablecops::log::err()
                << "  SDO write to disable mode (0x2103:00) failed: " << ec.message() << '\n';
            return ec;
        }
        stablecops::log::out() << "  disable mode (0x2103) set to "
                               << static_cast<int>(*boot_actions_.disable_mode) << '\n';
    }

    // Ad-hoc raw object writes (experimentation / drive configuration). Each is
    // downloaded with the requested CANopen width so single-byte objects are not
    // rejected by a wrong-length download.
    for (const auto& object : boot_actions_.object_writes) {
        switch (object.width) {
            case ds402::ObjectWidth::U8:
                Wait(AsyncWrite(object.index, object.subindex, static_cast<uint8_t>(object.value)),
                     ec);
                break;
            case ds402::ObjectWidth::U16:
                Wait(AsyncWrite(object.index, object.subindex, static_cast<uint16_t>(object.value)),
                     ec);
                break;
            case ds402::ObjectWidth::U32:
                Wait(AsyncWrite(object.index, object.subindex, static_cast<uint32_t>(object.value)),
                     ec);
                break;
            case ds402::ObjectWidth::I8:
                Wait(AsyncWrite(object.index, object.subindex, static_cast<int8_t>(object.value)),
                     ec);
                break;
            case ds402::ObjectWidth::I16:
                Wait(AsyncWrite(object.index, object.subindex, static_cast<int16_t>(object.value)),
                     ec);
                break;
            case ds402::ObjectWidth::I32:
                Wait(AsyncWrite(object.index, object.subindex, static_cast<int32_t>(object.value)),
                     ec);
                break;
        }
        if (ec) {
            stablecops::log::err() << "  SDO write to ";
            writeHex(stablecops::log::err(), object.index, 4)
                << ':' << std::setw(2) << std::setfill('0') << std::uppercase << std::hex
                << static_cast<int>(object.subindex) << std::dec << " failed: " << ec.message()
                << '\n';
            return ec;
        }
        stablecops::log::out() << "  object ";
        writeHex(stablecops::log::out(), object.index, 4)
            << ':' << std::setw(2) << std::setfill('0') << std::uppercase << std::hex
            << static_cast<int>(object.subindex) << std::dec << " set to " << object.value << '\n';
    }

    // Persist the objects just written so a value that the drive only honours out
    // of NVM (or that must survive a power cycle) takes effect. Saves all
    // application parameters (0x1010:03).
    if (boot_actions_.save_params) {
        try {
            drive_.storeApplicationParameters();
        } catch (const std::exception& exception) {
            stablecops::log::err()
                << "  saving application parameters to NVM failed: " << exception.what() << '\n';
            return std::make_error_code(std::errc::io_error);
        }
        stablecops::log::out() << "  application parameters saved to NVM (0x1010:03)\n";
    }
    return {};
}

std::error_code MotorDriver::configurePdos() noexcept {
    stablecops::log::out() << "node " << static_cast<int>(id())
                           << ": configuring drive PDOs for cyclic synchronous operation\n";

    std::error_code ec;
    const auto download = [&](uint16_t index, uint8_t subindex, auto value,
                              const char* what) -> bool {
        Wait(AsyncWrite(index, subindex, value), ec);
        if (ec) {
            stablecops::log::err() << "  SDO write to " << what << " (";
            writeHex(stablecops::log::err(), index, 4)
                << ':' << std::setw(2) << std::setfill('0') << std::uppercase << std::hex
                << static_cast<int>(subindex) << std::dec << ") failed: " << ec.message() << '\n';
        }
        return !ec;
    };

    // Out of NVM the drive does not run the cyclic exchange (per the manual its
    // TxPDOs default to disabled COB-IDs, and this firmware additionally reports
    // stale transmission types/mappings), so every active PDO is rewritten here.
    // PDO parameters are only reliably SDO-writable while the node is
    // pre-operational (manual Table 5-5 configures them between Reset Node and
    // NMT Start). The layout comes entirely from the generated summary.json
    // (pdo_map_), derived from the same profile as dcf/master.dcf, so both ends
    // of the bus stay coherent.
    //
    // Note: the DS402 command objects (0x6040/0x6060/0x607A/...) are NOT written
    // here. On this firmware they reject SDO downloads with vendor abort
    // 0x00000002 ("PDO mapped only"); only the PDO communication/mapping
    // parameters are SDO-writable.
    const auto program = [&](const char* role, const config::PdoChannel& pdo) -> bool {
        // Disable the PDO before touching its parameters, set the transmission
        // type, rewrite the mapping from scratch, then re-enable it. Rewriting
        // the mapping (rather than only the transmission type) defends against
        // the stale/oversized mapping the drive reports straight out of NVM.
        const uint32_t base_cob_id = pdo.baseCobId();
        if (!download(pdo.comm_index, 0x01, base_cob_id | kCobIdValidMask,
                      "PDO COB-ID (disable)")) {
            return false;
        }
        if (!download(pdo.comm_index, 0x02, static_cast<uint8_t>(pdo.transmission_type),
                      "PDO transmission type")) {
            return false;
        }
        if (!download(pdo.map_index, 0x00, static_cast<uint8_t>(0), "PDO mapping count (clear)")) {
            return false;
        }
        uint8_t sub = 0;
        for (const auto& object : pdo.entries) {
            ++sub;
            if (!download(pdo.map_index, sub, mappingEntryWord(object), "PDO mapping entry")) {
                return false;
            }
        }
        if (!download(pdo.map_index, 0x00, static_cast<uint8_t>(sub), "PDO mapping count")) {
            return false;
        }
        if (!download(pdo.comm_index, 0x01, base_cob_id, "PDO COB-ID (enable)")) {
            return false;
        }
        stablecops::log::out() << "node " << static_cast<int>(id()) << ": " << role << " 0x"
                               << std::hex << pdo.comm_index << std::dec << " COB-ID 0x" << std::hex
                               << base_cob_id << std::dec << " set to transmission type "
                               << static_cast<int>(pdo.transmission_type) << ", "
                               << pdo.entries.size() << " mapped objects\n";
        return true;
    };

    // Active channels (COB-ID valid bit clear, at least one mapped object) are
    // programmed for cyclic synchronous transfer. Inactive RxPDOs are explicitly
    // disabled so the drive never acts on stale frames the master no longer
    // sends; inactive TxPDOs are left untouched (an unexpected TxPDO would only
    // be ignored by the master).
    for (const auto& pdo : pdo_map_.rpdo) {
        if (pdo.active() && !pdo.entries.empty()) {
            if (!program("RxPDO", pdo)) {
                return ec;
            }
        } else {
            if (!download(pdo.comm_index, 0x01, pdo.baseCobId() | kCobIdValidMask,
                          "PDO COB-ID (disable)")) {
                return ec;
            }
            stablecops::log::out() << "node " << static_cast<int>(id()) << ": RxPDO 0x" << std::hex
                                   << pdo.comm_index << std::dec << " disabled (unused)\n";
        }
    }

    for (const auto& pdo : pdo_map_.tpdo) {
        if (pdo.active() && !pdo.entries.empty()) {
            if (!program("TxPDO", pdo)) {
                return ec;
            }
        }
    }

    return {};
}

void MotorDriver::OnRpdoWrite(uint16_t index, uint8_t /*subindex*/) noexcept {
    if (isFeedbackObject(index)) {
        rpdo_seen_ = true;
        // Stamp every received feedback frame so the staleness watchdog can tell
        // when the drive has stopped talking on the bus.
        last_feedback_time_ = std::chrono::steady_clock::now();
    }
}

void MotorDriver::OnSync(uint8_t /*counter*/, const time_point& /*time*/) noexcept {
    ++sync_count_;

    std::error_code ec;

    const auto now = std::chrono::steady_clock::now();

    // Measure the achieved cyclic cadence (interval/jitter vs the nominal SYNC
    // period). Cheap and allocation-free; published with the feedback snapshot.
    updateCyclicStats(now);

    // Keep the node-liveness flag on the cached image current every cycle so the
    // published snapshot always reflects the latest heartbeat/node-guarding view.
    feedback_.node_alive = !node_loss_;

    // Refresh the cached feedback from every object the drive maps into its
    // TPDOs. These reads never hit the bus; they snapshot the last received PDO.
    // The set of objects is whatever the profile mapped, not a fixed list.
    if (rpdo_seen_) {
        for (auto& object : feedback_objects_) {
            const auto raw = readMappedObject(object, ec);
            if (!ec) {
                object.value = raw;
                decodeFeedbackObject(object.index, raw);
            }
        }
        logFaultTransition();
    }

    // Liveness reflects staleness rather than latching: it is true only while
    // fresh feedback keeps arriving, so readers (and the facade) can trust it.
    // A node the heartbeat/node-guarding channel has lost is never live, even if
    // stale TPDOs would otherwise still be within the window.
    const bool stale = feedbackStale(now);
    feedback_live_.store(rpdo_seen_ && !stale && !node_loss_, std::memory_order_release);
    publishFeedback();

    // Safety watchdog: if the drive stops talking while energised, drop the
    // power stage. requestGracefulStop is one-shot, so this logs/acts once.
    const bool homing_active = homing_phase_ != ds402::HomingPhase::Idle &&
                               homing_phase_ != ds402::HomingPhase::Done &&
                               homing_phase_ != ds402::HomingPhase::Failed;
    if (cyclic_active_ && stop_phase_ == StopPhase::None && stale) {
        if (homing_active) {
            stablecops::log::err() << "feedback watchdog: no drive feedback for >"
                                   << boot_actions_.feedback_timeout.count()
                                   << " ms during homing; commanding zero velocity\n";
            failHoming("feedback watchdog expired during homing");
        } else if (homing_phase_ == ds402::HomingPhase::Failed) {
            if (hasCommand(ds402::od::target_velocity)) {
                setCommandValue(ds402::od::target_velocity, 0);
            }
        } else {
            stablecops::log::err()
                << "feedback watchdog: no drive feedback for >"
                << boot_actions_.feedback_timeout.count() << " ms; de-energising\n";
            requestGracefulStop();
        }
    }

    if (!cyclic_active_) {
        return;
    }

    // While bringing a position mode up (boot enable or fault recovery), keep
    // the commanded target glued to the measured position so the drive never
    // sees a step. Only meaningful when target position is in the stream.
    if (csp_track_actual_ && rpdo_seen_ && hasCommand(ds402::od::target_position)) {
        setCommandValue(ds402::od::target_position, feedback_.position);
    }

    // Exactly one controlword driver runs per cycle, in priority order: a
    // controlled shutdown always wins, then fault recovery, then the Profile
    // Position handshake.
    if (stop_phase_ != StopPhase::None && stop_phase_ != StopPhase::Done) {
        advanceGracefulStop();
    } else if (enable_phase_ != EnablePhase::Idle) {
        advanceEnableLadder();
    } else if (homing_phase_ != ds402::HomingPhase::Idle &&
               homing_phase_ != ds402::HomingPhase::Done &&
               homing_phase_ != ds402::HomingPhase::Failed) {
        advanceHoming();
    } else if (setpoint_phase_ != SetpointPhase::Idle) {
        advanceProfileSetpoint();
    }

    // Emit the whole command image every cycle. A PDO is transmitted as one
    // frame, so streaming every mapped command object together keeps each frame
    // coherent and never zeros a neighbour that shares the PDO.
    for (const auto& object : command_objects_) {
        writeMappedObject(object, ec);
    }
}

void MotorDriver::OnEmcy(uint16_t eec, uint8_t er, uint8_t msef[5]) noexcept {
    // The drive announces faults on the dedicated emergency channel (COB
    // 0x080+id) as well as through statusword/0x603F in the TPDO. Fold the EMCY
    // into the cached feedback so an internally-latched fault is visible even
    // when the cyclic stream keeps flowing, and publish it for other threads.
    feedback_.emergency_error_code = eec;
    feedback_.error_register = er;
    for (std::size_t i = 0; i < feedback_.vendor_error.size(); ++i) {
        feedback_.vendor_error[i] = msef[i];
    }

    if (eec != 0) {
        stablecops::log::err() << "node " << static_cast<int>(id()) << " EMCY: error_code=";
        writeHex(stablecops::log::err(), eec, 4) << " error_register=";
        writeHex(stablecops::log::err(), er, 2) << " vendor=";
        for (std::size_t i = 0; i < feedback_.vendor_error.size(); ++i) {
            writeHex(stablecops::log::err(), feedback_.vendor_error[i], 2)
                << (i + 1 < feedback_.vendor_error.size() ? " " : "");
        }
        stablecops::log::err() << " (" << ds402::describeDeviceFault(eec)
                               << "; register: " << ds402::describeErrorRegister(er) << ")\n";
    } else {
        // EMCY with error code 0x0000 is the standard "error reset / no error"
        // notification; the drive has cleared its emergency state.
        stablecops::log::out() << "node " << static_cast<int>(id()) << " EMCY cleared (no error)\n";
    }

    publishFeedback();
}

void MotorDriver::OnState(::lely::canopen::NmtState state) noexcept {
    // NMT state change (or boot-up) of the remote node, detected by the
    // heartbeat protocol. Purely informational here; node loss is handled by
    // OnHeartbeat/OnNodeGuarding.
    stablecops::log::out() << "node " << static_cast<int>(id()) << " NMT state -> "
                           << static_cast<int>(state) << '\n';
}

void MotorDriver::OnHeartbeat(bool occurred) noexcept {
    // occurred == true: the master's consumer heartbeat timed out (node lost);
    // occurred == false: heartbeat resumed.
    handleNodeLoss("heartbeat", occurred);
}

void MotorDriver::OnNodeGuarding(bool occurred) noexcept {
    handleNodeLoss("node guarding", occurred);
}

void MotorDriver::handleNodeLoss(const char* channel, bool lost) {
    if (lost == node_loss_) {
        return;  // already in this state; react once per transition
    }
    node_loss_ = lost;
    feedback_.node_alive = !lost;

    if (lost) {
        stablecops::log::err() << "node " << static_cast<int>(id()) << ' ' << channel
                               << " lost; the node stopped its error-control traffic\n";
        // The node is gone on an independent channel, so any cyclic feedback is
        // no longer trustworthy regardless of TPDO cadence.
        feedback_live_.store(false, std::memory_order_release);
        // De-energise if we were driving it; harmless (and a no-op) otherwise.
        if (cyclic_active_ && stop_phase_ == StopPhase::None) {
            stablecops::log::err()
                << "node " << static_cast<int>(id()) << ": de-energising after node loss\n";
            requestGracefulStop();
        }
    } else {
        stablecops::log::out() << "node " << static_cast<int>(id()) << ' ' << channel
                               << " restored\n";
    }

    publishFeedback();
}

void MotorDriver::inspectNode() noexcept {
    stablecops::log::out() << "inspecting CANopen/DS402 objects\n";

    // One typed SDO read + log line; the width tag selects the CANopen type.
    const auto read = [this](auto width_tag, uint16_t index, uint8_t subindex, const char* name) {
        using T = decltype(width_tag);
        std::error_code ec;
        const auto value = Wait(AsyncRead<T>(index, subindex), ec);
        writeObjectName(stablecops::log::out(), name, index, subindex);
        if (ec) {
            stablecops::log::out() << "FAILED: " << ec.message() << '\n';
        } else {
            writeHex(stablecops::log::out(), static_cast<uint32_t>(value), sizeof(T) * 2)
                << " (" << static_cast<int64_t>(value) << ")\n";
        }
        return std::pair<T, bool>{value, !ec};
    };
    const auto read_u8 = [&](uint16_t index, uint8_t subindex, const char* name) {
        return read(uint8_t{}, index, subindex, name);
    };
    const auto read_u16 = [&](uint16_t index, uint8_t subindex, const char* name) {
        return read(uint16_t{}, index, subindex, name);
    };
    const auto read_u32 = [&](uint16_t index, uint8_t subindex, const char* name) {
        return read(uint32_t{}, index, subindex, name);
    };
    const auto read_i32 = [&](uint16_t index, uint8_t subindex, const char* name) {
        return read(int32_t{}, index, subindex, name);
    };

    stablecops::log::out() << " identity\n";
    read_u32(0x1018, 0x01, "vendor id");
    read_u32(0x1018, 0x02, "product code");
    read_u32(0x1018, 0x03, "revision");
    read_u32(0x1018, 0x04, "serial number");

    stablecops::log::out() << " DS402\n";
    const auto [statusword, status_ok] =
        read_u16(ds402::od::statusword, ds402::od::default_subindex, "statusword");
    if (status_ok) {
        stablecops::log::out() << "  decoded state = "
                               << ds402::toString(ds402::decodeState(statusword)) << '\n';
    }

    read_u32(0x6502, 0x00, "supported modes");
    read_u8(ds402::od::modes_of_operation, ds402::od::default_subindex, "commanded mode");
    const auto [display_mode, display_mode_ok] = read_u8(
        ds402::od::modes_of_operation_display, ds402::od::default_subindex, "displayed mode");
    if (display_mode_ok) {
        const auto mode = static_cast<ds402::OperationMode>(static_cast<int8_t>(display_mode));
        stablecops::log::out() << "  decoded mode = " << ds402::toString(mode) << '\n';
    }

    const auto [position_counts, position_ok] =
        read_i32(ds402::od::position_actual_value, ds402::od::default_subindex, "position");
    if (position_ok && boot_actions_.counts_per_rev != 0) {
        stablecops::log::out() << "    -> "
                               << (static_cast<double>(position_counts) /
                                   boot_actions_.counts_per_rev * 360.0)
                               << " deg (" << boot_actions_.counts_per_rev << " counts/rev)\n";
    }
    read_i32(ds402::od::velocity_actual_value, ds402::od::default_subindex, "velocity");
    read_u16(ds402::od::torque_actual_value, ds402::od::default_subindex, "torque");
    const auto [error_code_value, error_code_ok] =
        read_u16(ds402::od::error_code, ds402::od::default_subindex, "error code");
    if (error_code_ok && error_code_value != 0) {
        stablecops::log::out() << "  decoded fault = "
                               << ds402::describeDeviceFault(error_code_value) << '\n';
    }

    // Actively read the error register (0x1001) and error history (0x1003) over
    // SDO. Unlike the EMCY channel this works even for a fault that latched
    // before the master attached (or an EMCY frame we missed), so a pre-existing
    // fault is surfaced at inspection time. 0x1001 is not PDO-mappable, so SDO is
    // the only way to poll it on demand.
    const auto [error_register_value, error_register_ok] =
        read_u8(ds402::od::error_register, ds402::od::default_subindex, "error register");
    if (error_register_ok) {
        stablecops::log::out() << "  decoded register = "
                               << ds402::describeErrorRegister(error_register_value) << '\n';
        // Seed the cached fault view so faulted()/telemetry reflect a latched
        // fault discovered here even before the first EMCY arrives.
        feedback_.error_register = error_register_value;
    }

    const auto [error_history_count, error_history_ok] =
        read_u8(ds402::od::predefined_error_field, ds402::od::predefined_error_field_count_subindex,
                "error history count");
    if (error_history_ok && error_history_count != 0) {
        const uint8_t entries = error_history_count <= 8 ? error_history_count : 8;
        for (uint8_t sub = ds402::od::predefined_error_field_latest_subindex; sub <= entries;
             ++sub) {
            const auto [entry, entry_ok] =
                read_u32(ds402::od::predefined_error_field, sub, "  error history entry");
            if (entry_ok) {
                const auto entry_code = static_cast<uint16_t>(entry & 0xFFFF);
                const auto entry_register = static_cast<uint8_t>((entry >> 16) & 0xFF);
                stablecops::log::out()
                    << "    -> " << ds402::describeDeviceFault(entry_code)
                    << "; register: " << ds402::describeErrorRegister(entry_register) << '\n';
            }
        }
    }
    if (error_register_ok || error_code_ok) {
        publishFeedback();
    }

    // Position source and scaling. 0x6064 (the value we stream as feedback) is
    // the control-loop position: it is derived from the primary (motor) encoder,
    // scaled by the encoder resolution (0x608F) and gear ratio (0x6091), then
    // referenced to the home offset (0x607C). To tell whether it tracks the
    // motor or the load, rotate the joint a known amount and watch which counter
    // moves how much: 0x6063/0x6064 are the scaled control position, the single-
    // turn absolute registers (0x276F/0x2772) are the raw per-revolution encoder
    // angle, and 0x221A selects whether a second (load-side) encoder is used.
    stablecops::log::out() << " position feedback & scaling\n";
    read_i32(0x6062, 0x00, "position demand value");
    read_i32(0x6063, 0x00, "position feedback value");
    read_i32(0x276F, 0x00, "encoder single-turn (primary)");
    read_i32(0x2772, 0x00, "encoder single-turn (3)");
    read_u8(0x2219, 0x00, "second feedback direction");
    read_u8(0x221A, 0x00, "second feedback mode");
    read_u32(0x608F, 0x01, "encoder increments");
    read_u32(0x608F, 0x02, "motor revolutions (resolution)");
    read_u32(0x6091, 0x01, "gear ratio: motor revolutions");
    read_u32(0x6091, 0x02, "gear ratio: shaft revolutions");

    // Drive-side PDO configuration. If the drive never emits TPDOs or ignores
    // RPDOs, the cause is almost always here: a COB-ID with the valid bit set
    // (0x8000_0000), a COB-ID that does not match the master's expectation, or a
    // transmission type that is not cyclic-synchronous (1..240).
    stablecops::log::out() << " PDO configuration (drive side)\n";
    const auto dump_pdo = [&](const char* label, uint16_t comm_index, uint16_t map_index) {
        const auto [cob_id, cob_ok] = read_u32(comm_index, 0x01, label);
        if (cob_ok) {
            const bool valid = (cob_id & 0x80000000u) == 0;
            stablecops::log::out() << "    -> COB-ID 0x" << std::hex << (cob_id & 0x7FF) << std::dec
                                   << (valid ? ", ENABLED" : ", DISABLED (valid bit set)") << '\n';
        }
        read_u8(comm_index, 0x02, "  transmission type");
        const auto [count, count_ok] = read_u8(map_index, 0x00, "  mapped object count");
        if (count_ok) {
            for (uint8_t sub = 1; sub <= count && sub <= 8; ++sub) {
                read_u32(map_index, sub, "  mapping entry");
            }
        }
    };

    dump_pdo("TPDO1 COB-ID", 0x1800, 0x1A00);
    dump_pdo("TPDO2 COB-ID", 0x1801, 0x1A01);
    dump_pdo("RPDO1 COB-ID", 0x1400, 0x1600);
    dump_pdo("RPDO2 COB-ID", 0x1401, 0x1601);
    dump_pdo("RPDO3 COB-ID", 0x1402, 0x1602);

    read_u16(0x1017, 0x00, "producer heartbeat time");
    read_u32(0x1005, 0x00, "SYNC COB-ID");

    // Disable / brake behaviour. These govern what happens to motor torque when
    // the drive leaves Operation Enabled: the manufacturer "active disable"
    // block keeps the motor energised to hold the load for a configured window,
    // and the standard option codes select the ramp used for each transition.
    // 0x60FE reveals whether a holding brake is driven from a digital output.
    stablecops::log::out() << " disable / brake configuration\n";
    read_u8(0x2103, 0x00, "disable mode");
    read_u32(0x2104, 0x00, "active disable speed threshold");
    read_u32(0x2105, 0x00, "active disable delay");
    read_u16(0x2106, 0x00, "active disable time");
    read_u32(0x2107, 0x00, "active disable deceleration");
    read_u16(0x2108, 0x00, "active disable deceleration time");
    read_u16(0x605A, 0x00, "quick stop option code");
    read_u16(0x605B, 0x00, "shutdown option code");
    read_u16(0x605C, 0x00, "disable operation option code");
    read_u32(0x6085, 0x00, "quick stop deceleration");
    read_u32(0x60FE, 0x01, "digital outputs (physical)");
    read_u32(0x60FE, 0x02, "digital outputs (mask)");

    stablecops::log::out() << " vendor runtime monitors\n";
    read_u8(0x2706, 0x00, "drive enable status");
    read_u8(0x2709, 0x00, "system state machine state");
    read_u8(0x270A, 0x00, "motor run loop state");
}

bool MotorDriver::wantsMotionAction() const {
    return boot_actions_.enable || boot_actions_.hold_position ||
           boot_actions_.csp_target_position.has_value() ||
           boot_actions_.csp_relative_move.has_value();
}

bool MotorDriver::wantsCyclicConfig() const {
    // PDO reconfiguration (transmission type 1) is needed both to command the
    // drive and to merely observe its feedback cyclically. A boot-time parameter
    // write (disable mode) or NVM save also needs the pre-operational config
    // window even when no cyclic exchange is requested.
    return wantsMotionAction() || boot_actions_.monitor || boot_actions_.disable_mode.has_value() ||
           !boot_actions_.object_writes.empty() || boot_actions_.save_params;
}

void MotorDriver::runBootActions() noexcept {
    if (!wantsMotionAction()) {
        return;
    }

    try {
        enableDrive(boot_actions_.hold_position || boot_actions_.csp_target_position.has_value() ||
                    boot_actions_.csp_relative_move.has_value());
        applyCspTarget();
    } catch (const std::exception& exception) {
        stablecops::log::err() << "boot action failed: " << exception.what() << '\n';
    }
}

ds402::Feedback MotorDriver::waitForDriveState(ds402::State expected,
                                               std::chrono::milliseconds timeout,
                                               std::chrono::milliseconds poll_interval) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    auto feedback = drive_.readFeedback();

    while (true) {
        if (stop_phase_ != StopPhase::None) {
            throw std::runtime_error("graceful stop requested while waiting for " +
                                     ds402::toString(expected));
        }
        if (feedback.state == expected) {
            return feedback;
        }
        if (feedback.state == ds402::State::Fault ||
            feedback.state == ds402::State::FaultReactionActive) {
            throw std::runtime_error("drive entered fault state while waiting for " +
                                     ds402::toString(expected));
        }
        if (feedback.error_code != 0) {
            throw std::runtime_error("drive reported nonzero error code while waiting for " +
                                     ds402::toString(expected));
        }
        if (std::chrono::steady_clock::now() >= deadline) {
            std::ostringstream message;
            message << "timed out waiting for " << ds402::toString(expected) << "; last state was "
                    << ds402::toString(feedback.state) << " (statusword 0x" << std::hex
                    << std::uppercase << std::setw(4) << std::setfill('0') << feedback.statusword
                    << ", controlword 0x" << std::setw(4) << std::setfill('0')
                    << commandValue(ds402::od::controlword) << std::dec << ", drive TPDO "
                    << (rpdo_seen_ ? "live" : "not received") << ")";
            throw std::runtime_error(message.str());
        }

        USleep(static_cast<uint_least64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(poll_interval).count()));
        feedback = drive_.readFeedback();
    }
}

void MotorDriver::primeCommandImage() {
    // Seed each mapped command object so the first cyclic frames are safe and
    // never zero a shared-PDO neighbour. Defaults are 0 (no motion); the target
    // position is seeded to the live measured position so a position mode holds
    // still at enable, and configuration objects that happen to be streamed are
    // seeded from their persisted value over SDO.
    std::error_code ec;

    for (auto& object : command_objects_) {
        object.value = 0;
    }

    if (hasCommand(ds402::od::target_position)) {
        const auto position = Wait(AsyncRead<int32_t>(ds402::od::position_actual_value, 0), ec);
        if (!ec) {
            setCommandValue(ds402::od::target_position, position);
        }
    }

    for (const uint16_t index : {ds402::od::profile_velocity, ds402::od::profile_acceleration,
                                 ds402::od::profile_deceleration}) {
        if (hasCommand(index)) {
            const auto value = Wait(AsyncRead<uint32_t>(index, 0), ec);
            if (!ec) {
                setCommandValue(index, value);
            }
        }
    }

    setCommandValue(ds402::od::controlword, ds402::controlword::shutdown);
}

void MotorDriver::setControlword(uint16_t controlword) {
    setCommandValue(ds402::od::controlword, controlword);
}

void MotorDriver::setStoppedCallback(std::function<void()> on_stopped) {
    on_stopped_ = std::move(on_stopped);
}

bool MotorDriver::isDriveDeEnergized() const {
    return feedback_.state == ds402::State::SwitchOnDisabled ||
           feedback_.state == ds402::State::NotReadyToSwitchOn;
}

void MotorDriver::finishGracefulStop() {
    stop_phase_ = StopPhase::Done;
    cyclic_active_ = false;
    stablecops::log::out() << "graceful stop: drive disabled (final state "
                           << ds402::toString(feedback_.state) << ", statusword=";
    writeHex(stablecops::log::out(), feedback_.statusword, 4) << ")\n";
    if (on_stopped_) {
        on_stopped_();
    }
}

void MotorDriver::requestGracefulStop() {
    if (stop_phase_ != StopPhase::None) {
        return;  // shutdown already in progress
    }

    // If nothing is energised (inspect-only, idle, or a failed boot) there is no
    // DS402 ramp-down to perform; hand control straight back to the caller.
    if (!cyclic_active_) {
        stablecops::log::out() << "graceful stop: drive not energised; stopping master\n";
        stop_phase_ = StopPhase::Done;
        if (on_stopped_) {
            on_stopped_();
        }
        return;
    }

    stablecops::log::out() << "graceful stop: de-energising drive (disable voltage)\n";
    // De-energise immediately: stop holding position and command disable-voltage
    // so the drive drops the power stage at once and the joint goes limp (coasts,
    // no brake). The command is streamed every SYNC until the drive confirms it
    // reached switch-on-disabled, or until the timeout forces the shutdown.
    csp_track_actual_ = false;
    const auto now = std::chrono::steady_clock::now();
    setControlword(ds402::controlword::disable_voltage);
    stop_phase_ = StopPhase::DisableVoltage;
    stop_phase_deadline_ = now + boot_actions_.state_transition_timeout;
    stop_log_due_ = now;
}

void MotorDriver::requestQuickStop() {
    if (stop_phase_ != StopPhase::None && stop_phase_ != StopPhase::Done) {
        stablecops::log::warn() << "quick stop ignored: a graceful stop is already in progress\n";
        return;
    }
    if (!cyclic_active_) {
        stablecops::log::warn() << "quick stop ignored: drive is not energised\n";
        return;
    }

    // Quick stop is a priority action: abort any in-flight control sequence so
    // nothing else drives the controlword this cycle.
    setpoint_phase_ = SetpointPhase::Idle;
    enable_phase_ = EnablePhase::Idle;
    const bool homing_active = homing_phase_ != ds402::HomingPhase::Idle &&
                               homing_phase_ != ds402::HomingPhase::Done &&
                               homing_phase_ != ds402::HomingPhase::Failed;
    if (homing_active) {
        failHoming("quick stop requested");
    }

    // Stop commanding motion and glue the position target so a later re-enable
    // cannot step, then command the CiA402 quick stop. The drive decelerates on
    // its quick-stop ramp (0x6085, per option code 0x605A) instead of coasting,
    // and stays energised in quick-stop-active when the option code keeps it
    // there (5..8). The command image streams controlword every SYNC, so the
    // hold persists without any further action.
    if (hasCommand(ds402::od::target_velocity)) {
        setCommandValue(ds402::od::target_velocity, 0);
    }
    if (hasCommand(ds402::od::target_torque)) {
        setCommandValue(ds402::od::target_torque, 0);
    }
    if (hasCommand(ds402::od::target_position)) {
        setCommandValue(ds402::od::target_position, feedback_.position);
    }
    csp_track_actual_ = false;
    setControlword(ds402::controlword::quick_stop);
    stablecops::log::out()
        << "quick stop commanded: decelerating on the quick-stop ramp (holds energised in "
           "quick-stop-active if option code 0x605A is 5..8)\n";
}

void MotorDriver::advanceGracefulStop() {
    const auto now = std::chrono::steady_clock::now();
    if (stop_phase_ != StopPhase::DisableVoltage) {
        return;
    }

    // Keep streaming disable-voltage so the drive sees the command on every SYNC
    // and drops its power stage. Finish as soon as the drive confirms it reached
    // switch-on-disabled, and unconditionally finish at the deadline so a drive
    // that never reports the state can never leave the joint energised.
    setControlword(ds402::controlword::disable_voltage);

    if (isDriveDeEnergized()) {
        stablecops::log::out() << "graceful stop: power stage off (state "
                               << ds402::toString(feedback_.state) << ")\n";
        finishGracefulStop();
        return;
    }

    if (now >= stop_phase_deadline_) {
        stablecops::log::err() << "graceful stop: timed out waiting for de-energize; forcing "
                                  "shutdown (state="
                               << ds402::toString(feedback_.state) << " statusword=";
        writeHex(stablecops::log::err(), feedback_.statusword, 4) << ")\n";
        finishGracefulStop();
        return;
    }

    if (now >= stop_log_due_) {
        stablecops::log::out() << "graceful stop: waiting for drive to de-energize; state="
                               << ds402::toString(feedback_.state) << " statusword=";
        writeHex(stablecops::log::out(), feedback_.statusword, 4) << " controlword=";
        writeHex(stablecops::log::out(),
                 static_cast<uint32_t>(commandValue(ds402::od::controlword)), 4)
            << " torque=" << static_cast<int>(feedback_.torque)
            << " velocity=" << feedback_.velocity << '\n';
        stop_log_due_ = now + kPowerOffRetryLogInterval;
    }
}

void MotorDriver::enableDrive(bool prime_csp_target) {
    auto feedback = drive_.readFeedback();
    stablecops::log::out() << "enabling DS402 operation from state "
                           << ds402::toString(feedback.state) << ", mode "
                           << ds402::toString(feedback.mode) << '\n';

    if (feedback.state == ds402::State::Fault ||
        feedback.state == ds402::State::FaultReactionActive) {
        throw std::runtime_error("refusing to enable while the drive is in fault");
    }
    if (feedback.error_code != 0) {
        throw std::runtime_error("refusing to enable with nonzero drive error code");
    }

    const bool csp_mode = feedback.mode == ds402::OperationMode::CyclicSynchronousPosition;
    if (prime_csp_target && !csp_mode) {
        throw std::runtime_error("CSP hold/target requires cyclic synchronous position mode");
    }

    // Start streaming a coherent command image before touching the state machine.
    // In a cyclic mode the drive will only walk the DS402 states while it is
    // receiving a valid, position-tracking command frame on every SYNC.
    primeCommandImage();
    csp_track_actual_ = csp_mode;
    setControlword(ds402::controlword::shutdown);
    cyclic_active_ = true;

    // Let a few SYNC cycles flow so the drive has the image before we command it.
    {
        const auto deadline =
            std::chrono::steady_clock::now() + boot_actions_.state_transition_timeout;
        const auto first_cycle = sync_count_;
        while (sync_count_ < first_cycle + 4) {
            if (std::chrono::steady_clock::now() >= deadline) {
                throw std::runtime_error("no SYNC cycles observed; is the master producing SYNC?");
            }
            USleep(1000);
        }
    }

    stablecops::log::out() << "  cyclic streaming active: sync cycles=" << sync_count_
                           << ", drive TPDO feedback "
                           << (rpdo_seen_ ? "LIVE (cyclic)" : "NOT received (SDO fallback)")
                           << "; statusword=";
    writeHex(stablecops::log::out(), feedback_.statusword, 4) << '\n';

    if (feedback.state == ds402::State::OperationEnabled) {
        setControlword(ds402::controlword::enable_operation);
        csp_track_actual_ = false;
        stablecops::log::out() << "  drive already in operation enabled; holding position "
                               << commandValue(ds402::od::target_position) << '\n';
        return;
    }

    // Fault-reset pulse per the vendor recipe (manual Table 5-5): a rising edge
    // on controlword bit 7 clears any latched fault and drops the drive to
    // switch-on-disabled before we walk it up the DS402 ladder. The pulse is
    // streamed for a few SYNC cycles and is harmless when no fault is latched.
    setControlword(ds402::controlword::fault_reset);
    USleep(4000);

    setControlword(ds402::controlword::shutdown);
    feedback =
        waitForDriveState(ds402::State::ReadyToSwitchOn, boot_actions_.state_transition_timeout);

    setControlword(ds402::controlword::switch_on);
    feedback = waitForDriveState(ds402::State::SwitchedOn, boot_actions_.state_transition_timeout);

    setControlword(ds402::controlword::enable_operation);
    feedback =
        waitForDriveState(ds402::State::OperationEnabled, boot_actions_.state_transition_timeout);

    // Freeze the held target at the position captured the instant we enabled.
    csp_track_actual_ = false;

    stablecops::log::out() << "  enabled operation, statusword=";
    writeHex(stablecops::log::out(), feedback.statusword, 4)
        << " state=" << ds402::toString(feedback.state) << '\n';

    if (csp_mode) {
        stablecops::log::out() << "  holding current position at "
                               << commandValue(ds402::od::target_position) << '\n';
    }
}

void MotorDriver::requestProfileMove(int32_t target_position, bool relative) {
    // Stage the target into the cyclic image and arm the handshake. The rising
    // edge of controlword bit 4 is produced over the next few cycles by
    // advanceProfileSetpoint(); re-arming mid-move is fine (change-immediately).
    setCommandValue(ds402::od::target_position, target_position);
    profile_move_relative_ = relative;
    setpoint_phase_ = SetpointPhase::Assert;
}

void MotorDriver::requestFaultReset() {
    if (stop_phase_ != StopPhase::None) {
        return;  // a shutdown is already in progress; do not fight it
    }

    // Clear the latched EMCY view so faulted() reflects the recovery attempt; a
    // drive that is still in trouble will re-announce the emergency (and the
    // statusword/0x603F fault path still catches a persisting fault).
    feedback_.emergency_error_code = 0;
    feedback_.error_register = 0;
    feedback_.vendor_error.fill(0);

    // Reset targets to a safe hold before re-energising: zero any velocity/
    // torque setpoint and glue the position target to the measured position so
    // recovery never produces a step.
    if (hasCommand(ds402::od::target_velocity)) {
        setCommandValue(ds402::od::target_velocity, 0);
    }
    if (hasCommand(ds402::od::target_torque)) {
        setCommandValue(ds402::od::target_torque, 0);
    }
    const bool position_mode = feedback_.mode == ds402::OperationMode::CyclicSynchronousPosition ||
                               feedback_.mode == ds402::OperationMode::ProfilePosition;
    csp_track_actual_ = position_mode;

    // Re-enable to the configured operating state: if the drive was meant to be
    // driven, walk all the way back to operation enabled; otherwise just clear
    // the fault and leave it safely disabled.
    recover_to_enabled_ = boot_actions_.enable || boot_actions_.hold_position ||
                          boot_actions_.csp_target_position.has_value() ||
                          boot_actions_.csp_relative_move.has_value();
    cyclic_active_ = true;
    enable_phase_ = EnablePhase::FaultReset;
    enable_phase_deadline_ =
        std::chrono::steady_clock::now() + boot_actions_.state_transition_timeout;
    stablecops::log::out() << "fault reset requested (recover to "
                           << (recover_to_enabled_ ? "operation enabled" : "disabled") << ")\n";
}

void MotorDriver::requestEnableOperation(bool hold_position) {
    if (stop_phase_ != StopPhase::None && stop_phase_ != StopPhase::Done) {
        throw std::runtime_error("cannot enable while graceful stop is in progress");
    }
    stop_phase_ = StopPhase::None;
    boot_actions_.enable = true;
    boot_actions_.hold_position = hold_position;
    enableDrive(hold_position);
}

void MotorDriver::requestOperationMode(ds402::OperationMode mode, bool confirm) {
    if (feedback_.state == ds402::State::OperationEnabled &&
        std::abs(feedback_.velocity) > kModeSwitchStationaryVelocity) {
        throw std::runtime_error("refusing to switch operation mode while the axis is moving");
    }

    const bool position_mode = mode == ds402::OperationMode::CyclicSynchronousPosition ||
                               mode == ds402::OperationMode::ProfilePosition;
    if (position_mode && hasCommand(ds402::od::target_position)) {
        setCommandValue(ds402::od::target_position, feedback_.position);
    }
    if (hasCommand(ds402::od::target_velocity)) {
        setCommandValue(ds402::od::target_velocity, 0);
    }
    if (hasCommand(ds402::od::target_torque)) {
        setCommandValue(ds402::od::target_torque, 0);
    }

    std::error_code ec;
    Wait(AsyncWrite(ds402::od::modes_of_operation, ds402::od::default_subindex,
                    static_cast<int8_t>(mode)),
         ec);
    if (ec) {
        throw std::system_error(ec, "SDO write to modes of operation (0x6060:00) failed");
    }
    boot_actions_.mode = mode;
    stablecops::log::out() << "operation mode requested: " << ds402::toString(mode)
                           << " (0x6060=" << static_cast<int>(static_cast<int8_t>(mode)) << ")\n";

    // Writing 0x6060 only requests the mode; the drive reflects the mode it has
    // actually entered in 0x6061. Confirm it took, so a firmware/state that
    // silently rejects the change surfaces as an error instead of leaving the
    // caller commanding the wrong mode. Only done off the cyclic path (the
    // homing restore drives this from OnSync and must stay non-blocking).
    if (confirm) {
        const auto deadline =
            std::chrono::steady_clock::now() + boot_actions_.state_transition_timeout;
        while (drive_.readMode() != mode) {
            if (std::chrono::steady_clock::now() >= deadline) {
                throw std::runtime_error(
                    "operation mode change to " + ds402::toString(mode) +
                    " was not confirmed by the drive (0x6061) within the timeout");
            }
            USleep(2000);
        }
        stablecops::log::out() << "  operation mode confirmed by 0x6061: " << ds402::toString(mode)
                               << '\n';
    }
}

void MotorDriver::requestHoming(const ds402::HomingConfig& config) {
    if (stop_phase_ != StopPhase::None && stop_phase_ != StopPhase::Done) {
        throw std::runtime_error("cannot home while graceful stop is in progress");
    }
    if (enable_phase_ != EnablePhase::Idle || setpoint_phase_ != SetpointPhase::Idle ||
        (homing_phase_ != ds402::HomingPhase::Idle && homing_phase_ != ds402::HomingPhase::Done &&
         homing_phase_ != ds402::HomingPhase::Failed)) {
        throw std::runtime_error("cannot home while another control sequence is active");
    }
    if (!hasCommand(ds402::od::target_velocity)) {
        throw std::runtime_error("homing requires target velocity in the RPDO map");
    }
    if (config.search_velocity == 0 || config.approach_velocity == 0 ||
        config.center_velocity == 0 || config.center_final_velocity == 0) {
        throw std::invalid_argument("homing velocities must be nonzero");
    }
    if (config.threshold_torque <= 0 || config.stopped_velocity < 0) {
        throw std::invalid_argument("homing contact thresholds must be positive");
    }
    if (config.min_travel <= 0 || config.max_travel <= config.min_travel) {
        throw std::invalid_argument("homing travel limits are invalid");
    }
    if (config.backoff_distance <= 0 || config.center_slowdown_distance <= 0 ||
        config.center_tolerance <= 0 || config.center_settle_tolerance < config.center_tolerance) {
        throw std::invalid_argument("homing backoff and center tolerances are invalid");
    }
    if (config.timeout.count() <= 0 || config.contact_dwell.count() <= 0 ||
        config.settle_time.count() < 0) {
        throw std::invalid_argument("homing timing values are invalid");
    }

    homing_restore_mode_ = feedback_.mode;
    homing_restore_enabled_ = feedback_.state == ds402::State::OperationEnabled;

    if (hasCommand(ds402::od::target_velocity)) {
        setCommandValue(ds402::od::target_velocity, 0);
    }
    if (hasCommand(ds402::od::target_torque)) {
        setCommandValue(ds402::od::target_torque, 0);
    }
    if (hasCommand(ds402::od::target_position)) {
        setCommandValue(ds402::od::target_position, feedback_.position);
    }

    if (feedback_.mode != ds402::OperationMode::CyclicSynchronousVelocity) {
        if (feedback_.state != ds402::State::SwitchOnDisabled &&
            feedback_.state != ds402::State::NotReadyToSwitchOn) {
            setControlword(ds402::controlword::disable_voltage);
            waitForDriveState(ds402::State::SwitchOnDisabled,
                              boot_actions_.state_transition_timeout);
        }
        requestOperationMode(ds402::OperationMode::CyclicSynchronousVelocity);
    }

    if (feedback_.state != ds402::State::OperationEnabled ||
        feedback_.mode != ds402::OperationMode::CyclicSynchronousVelocity) {
        enableDrive(false);
    }

    homing_config_ = config;
    homing_result_ = {};
    homing_start_position_ = feedback_.position;
    homing_backoff_target_ = 0;
    homing_center_target_ = 0;
    homing_contact_active_ = false;
    homing_deadline_ = std::chrono::steady_clock::now() + config.timeout;
    homing_phase_ = ds402::HomingPhase::SearchNegative;

    setControlword(ds402::controlword::enable_operation);
    setCommandValue(ds402::od::target_velocity, -std::abs(config.search_velocity));
    if (hasCommand(ds402::od::target_torque)) {
        setCommandValue(ds402::od::target_torque, 0);
    }
    stablecops::log::out() << "homing: searching negative hardstop from position "
                           << feedback_.position << " at velocity "
                           << -std::abs(config.search_velocity) << '\n';
}

bool MotorDriver::homingTimedOut(std::chrono::steady_clock::time_point now) const {
    return now >= homing_deadline_;
}

bool MotorDriver::homingContactDetected(int /*direction*/,
                                        std::chrono::steady_clock::time_point now) {
    // Contact = high torque while the axis is stalled. The velocity gate keeps a
    // mid-travel torque transient (friction bump, acceleration) from latching a
    // false hardstop while the axis is still moving at search speed.
    const bool loaded = std::abs(feedback_.torque) >= homing_config_.threshold_torque;
    const bool stalled = std::abs(feedback_.velocity) <= homing_config_.stopped_velocity;
    if (!homing_contact_active_) {
        if (!loaded || !stalled) {
            return false;
        }
        homing_contact_active_ = true;
        homing_contact_since_ = now;
        stablecops::log::out() << "homing: contact torque threshold reached at position "
                               << feedback_.position
                               << " (torque=" << static_cast<int>(feedback_.torque)
                               << ", velocity=" << feedback_.velocity
                               << "); commanding zero velocity\n";
    }

    // Stop pushing immediately. The dwell confirms the contact while the
    // commanded velocity is zero instead of driving into the hardstop.
    setCommandValue(ds402::od::target_velocity, 0);

    return now - homing_contact_since_ >= homing_config_.contact_dwell;
}

void MotorDriver::failHoming(const std::string& reason) {
    if (hasCommand(ds402::od::target_velocity)) {
        setCommandValue(ds402::od::target_velocity, 0);
    }
    if (hasCommand(ds402::od::target_torque)) {
        setCommandValue(ds402::od::target_torque, 0);
    }
    setControlword(ds402::controlword::enable_operation);
    homing_result_.success = false;
    homing_contact_active_ = false;
    homing_phase_ = ds402::HomingPhase::Failed;
    stablecops::log::err() << "homing failed: " << reason << " (position=" << feedback_.position
                           << ", velocity=" << feedback_.velocity
                           << ", torque=" << static_cast<int>(feedback_.torque) << ")\n";
}

void MotorDriver::advanceHoming() {
    // A blocking SDO exchange from a previous cycle is still in flight on its
    // fiber; keep streaming the current command image and try again next SYNC.
    if (homing_command_in_flight_) {
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    const auto command_search_velocity = std::abs(homing_config_.search_velocity);
    const auto command_approach_velocity = std::abs(homing_config_.approach_velocity);
    const auto command_center_velocity = std::abs(homing_config_.center_velocity);
    const auto command_center_final_velocity = std::abs(homing_config_.center_final_velocity);

    const bool csv_motion_phase = homing_phase_ == ds402::HomingPhase::SearchNegative ||
                                  homing_phase_ == ds402::HomingPhase::BackoffNegative ||
                                  homing_phase_ == ds402::HomingPhase::SearchPositive ||
                                  homing_phase_ == ds402::HomingPhase::MoveToCenter ||
                                  homing_phase_ == ds402::HomingPhase::WaitAtCenter ||
                                  homing_phase_ == ds402::HomingPhase::ZeroAtCenter;

    if (feedback_.state == ds402::State::Fault ||
        feedback_.state == ds402::State::FaultReactionActive || feedback_.error_code != 0) {
        failHoming("drive entered fault/error state");
        return;
    }
    if (csv_motion_phase && feedback_.state != ds402::State::OperationEnabled) {
        failHoming("drive is no longer operation enabled");
        return;
    }
    if (csv_motion_phase && feedback_.mode != ds402::OperationMode::CyclicSynchronousVelocity) {
        failHoming("drive left cyclic synchronous velocity mode");
        return;
    }
    if (homingTimedOut(now)) {
        failHoming("timeout");
        return;
    }

    setControlword(ds402::controlword::enable_operation);

    const auto travelled_from_start =
        std::llabs(static_cast<long long>(feedback_.position) - homing_start_position_);
    if ((homing_phase_ == ds402::HomingPhase::SearchNegative ||
         homing_phase_ == ds402::HomingPhase::SearchPositive) &&
        travelled_from_start > homing_config_.max_travel) {
        failHoming("maximum search travel exceeded");
        return;
    }

    switch (homing_phase_) {
        case ds402::HomingPhase::SearchNegative:
            setCommandValue(ds402::od::target_velocity, -command_search_velocity);
            if (homingContactDetected(-1, now)) {
                homing_result_.lower_limit_position = feedback_.position;
                homing_backoff_target_ =
                    homing_result_.lower_limit_position + homing_config_.backoff_distance;
                homing_contact_active_ = false;
                setCommandValue(ds402::od::target_velocity, command_approach_velocity);
                homing_phase_ = ds402::HomingPhase::BackoffNegative;
                stablecops::log::out()
                    << "homing: negative hardstop at " << homing_result_.lower_limit_position
                    << "; backing off to " << homing_backoff_target_ << '\n';
            }
            break;

        case ds402::HomingPhase::BackoffNegative:
            setCommandValue(ds402::od::target_velocity, command_approach_velocity);
            if (feedback_.position >= homing_backoff_target_) {
                homing_contact_active_ = false;
                setCommandValue(ds402::od::target_velocity, command_search_velocity);
                homing_phase_ = ds402::HomingPhase::SearchPositive;
                stablecops::log::out() << "homing: searching positive hardstop\n";
            }
            break;

        case ds402::HomingPhase::SearchPositive:
            setCommandValue(ds402::od::target_velocity, command_search_velocity);
            if (homingContactDetected(1, now)) {
                homing_result_.upper_limit_position = feedback_.position;
                homing_result_.travel =
                    homing_result_.upper_limit_position - homing_result_.lower_limit_position;
                if (homing_result_.travel < homing_config_.min_travel ||
                    homing_result_.travel > homing_config_.max_travel) {
                    failHoming("measured travel outside configured limits");
                    return;
                }

                const auto center = static_cast<long long>(homing_result_.lower_limit_position) +
                                    (static_cast<long long>(homing_result_.travel) / 2) +
                                    static_cast<long long>(homing_config_.home_offset);
                if (center < std::numeric_limits<int32_t>::min() ||
                    center > std::numeric_limits<int32_t>::max()) {
                    failHoming("computed center is outside int32 range");
                    return;
                }
                homing_center_target_ = static_cast<int32_t>(center);
                homing_result_.center_position = homing_center_target_;
                homing_contact_active_ = false;
                homing_deadline_ = now + homing_config_.timeout;
                homing_phase_ = ds402::HomingPhase::MoveToCenter;
                stablecops::log::out()
                    << "homing: positive hardstop at " << homing_result_.upper_limit_position
                    << "; center target " << homing_center_target_ << '\n';
            }
            break;

        case ds402::HomingPhase::MoveToCenter: {
            const auto delta = static_cast<long long>(homing_center_target_) - feedback_.position;
            if (std::llabs(delta) <= homing_config_.center_tolerance) {
                setCommandValue(ds402::od::target_velocity, 0);
                homing_settle_until_ = now + homing_config_.settle_time;
                homing_phase_ = ds402::HomingPhase::WaitAtCenter;
                stablecops::log::out() << "homing: center reached, settling\n";
            } else {
                const auto speed = std::llabs(delta) > homing_config_.center_slowdown_distance
                                       ? command_center_velocity
                                       : command_center_final_velocity;
                setCommandValue(ds402::od::target_velocity, delta > 0 ? speed : -speed);
            }
            break;
        }

        case ds402::HomingPhase::WaitAtCenter:
            setCommandValue(ds402::od::target_velocity, 0);
            if (std::llabs(static_cast<long long>(homing_center_target_) - feedback_.position) >
                homing_config_.center_settle_tolerance) {
                homing_phase_ = ds402::HomingPhase::MoveToCenter;
            } else if (now >= homing_settle_until_) {
                homing_phase_ = ds402::HomingPhase::ZeroAtCenter;
            }
            break;

        case ds402::HomingPhase::ZeroAtCenter:
            setCommandValue(ds402::od::target_velocity, 0);
            if (hasCommand(ds402::od::target_position)) {
                setCommandValue(ds402::od::target_position, 0);
            }
            // Both writes go over SDO and suspend this fiber; the in-flight
            // guard stops the next OnSync from re-issuing them (a repeated NVM
            // store would wear the EEPROM), and the catch turns an SDO abort
            // into a homing failure instead of terminating (OnSync is noexcept).
            homing_command_in_flight_ = true;
            try {
                drive_.setCurrentPositionAsZero();
                if (homing_config_.save_zero_to_nvm) {
                    drive_.storeApplicationParameters();
                }
            } catch (const std::exception& exception) {
                homing_command_in_flight_ = false;
                failHoming(std::string("failed to zero/store position: ") + exception.what());
                return;
            }
            homing_command_in_flight_ = false;
            homing_result_.success = true;
            stablecops::log::out()
                << "homing: zero set at center (lower=" << homing_result_.lower_limit_position
                << ", upper=" << homing_result_.upper_limit_position
                << ", travel=" << homing_result_.travel << ")\n";
            if (homing_restore_mode_ == ds402::OperationMode::CyclicSynchronousVelocity &&
                homing_restore_enabled_) {
                homing_phase_ = ds402::HomingPhase::Done;
            } else {
                homing_deadline_ = now + homing_config_.timeout;
                setControlword(ds402::controlword::disable_voltage);
                homing_phase_ = ds402::HomingPhase::RestoreDisable;
                stablecops::log::out() << "homing: restoring previous mode "
                                       << ds402::toString(homing_restore_mode_) << '\n';
            }
            break;

        case ds402::HomingPhase::RestoreDisable:
            setCommandValue(ds402::od::target_velocity, 0);
            if (hasCommand(ds402::od::target_torque)) {
                setCommandValue(ds402::od::target_torque, 0);
            }
            setControlword(ds402::controlword::disable_voltage);
            if (isDriveDeEnergized()) {
                if (homing_restore_mode_ != ds402::OperationMode::CyclicSynchronousVelocity) {
                    homing_phase_ = ds402::HomingPhase::RestoreMode;
                } else {
                    cyclic_active_ = homing_restore_enabled_;
                    homing_phase_ = ds402::HomingPhase::Done;
                }
            }
            break;

        case ds402::HomingPhase::RestoreMode:
            homing_command_in_flight_ = true;  // 0x6060 write suspends this fiber
            try {
                requestOperationMode(homing_restore_mode_);
            } catch (const std::exception& exception) {
                homing_command_in_flight_ = false;
                failHoming(std::string("failed to restore previous mode: ") + exception.what());
                return;
            }
            homing_command_in_flight_ = false;
            if (homing_restore_enabled_) {
                const bool position_mode =
                    homing_restore_mode_ == ds402::OperationMode::CyclicSynchronousPosition ||
                    homing_restore_mode_ == ds402::OperationMode::ProfilePosition;
                if (position_mode && hasCommand(ds402::od::target_position)) {
                    setCommandValue(ds402::od::target_position, feedback_.position);
                }
                if (hasCommand(ds402::od::target_velocity)) {
                    setCommandValue(ds402::od::target_velocity, 0);
                }
                if (hasCommand(ds402::od::target_torque)) {
                    setCommandValue(ds402::od::target_torque, 0);
                }
                csp_track_actual_ = position_mode;
                cyclic_active_ = true;
                setControlword(ds402::controlword::shutdown);
                enable_phase_ = EnablePhase::Shutdown;
                enable_phase_deadline_ = now + boot_actions_.state_transition_timeout;
                homing_phase_ = ds402::HomingPhase::RestoreEnable;
            } else {
                cyclic_active_ = false;
                homing_phase_ = ds402::HomingPhase::Done;
            }
            break;

        case ds402::HomingPhase::RestoreEnable:
            if (enable_phase_ != EnablePhase::Idle) {
                break;
            }
            if (feedback_.state == ds402::State::OperationEnabled) {
                homing_phase_ = ds402::HomingPhase::Done;
                stablecops::log::out() << "homing: previous mode/state restored\n";
            } else {
                failHoming("failed to restore previous operation enabled state");
            }
            break;

        case ds402::HomingPhase::Idle:
        case ds402::HomingPhase::Done:
        case ds402::HomingPhase::Failed:
            break;
    }
}

void MotorDriver::advanceEnableLadder() {
    const auto now = std::chrono::steady_clock::now();
    const auto state = feedback_.state;

    const auto fail = [&](const char* during) {
        stablecops::log::err() << "fault recovery: timed out during " << during
                               << " (state=" << ds402::toString(state) << ", statusword=";
        writeHex(stablecops::log::err(), feedback_.statusword, 4) << ")\n";
        setControlword(ds402::controlword::disable_voltage);
        csp_track_actual_ = false;
        enable_phase_ = EnablePhase::Idle;
    };

    // A re-fault during the ladder (after the initial reset) is unrecoverable
    // here; drop to a safe disabled state and stop.
    if (enable_phase_ != EnablePhase::FaultReset &&
        (state == ds402::State::Fault || state == ds402::State::FaultReactionActive)) {
        fail("recovery (drive re-faulted)");
        return;
    }

    const auto advance = [&](EnablePhase next, uint16_t controlword) {
        setControlword(controlword);
        enable_phase_ = next;
        enable_phase_deadline_ = now + boot_actions_.state_transition_timeout;
    };

    switch (enable_phase_) {
        case EnablePhase::FaultReset:
            // Rising edge on controlword bit 7 clears the latched fault; the
            // drive drops to switch-on-disabled. Harmless if no fault is latched.
            setControlword(ds402::controlword::fault_reset);
            if (state != ds402::State::Fault && state != ds402::State::FaultReactionActive) {
                if (recover_to_enabled_) {
                    advance(EnablePhase::Shutdown, ds402::controlword::shutdown);
                } else {
                    setControlword(ds402::controlword::shutdown);
                    csp_track_actual_ = false;
                    enable_phase_ = EnablePhase::Idle;
                    stablecops::log::out() << "fault recovery: fault cleared; drive disabled\n";
                }
            } else if (now >= enable_phase_deadline_) {
                fail("fault reset");
            }
            break;
        case EnablePhase::Shutdown:
            setControlword(ds402::controlword::shutdown);
            if (state == ds402::State::ReadyToSwitchOn || state == ds402::State::SwitchedOn ||
                state == ds402::State::OperationEnabled) {
                advance(EnablePhase::SwitchOn, ds402::controlword::switch_on);
            } else if (now >= enable_phase_deadline_) {
                fail("shutdown");
            }
            break;
        case EnablePhase::SwitchOn:
            setControlword(ds402::controlword::switch_on);
            if (state == ds402::State::SwitchedOn || state == ds402::State::OperationEnabled) {
                advance(EnablePhase::EnableOp, ds402::controlword::enable_operation);
            } else if (now >= enable_phase_deadline_) {
                fail("switch on");
            }
            break;
        case EnablePhase::EnableOp:
            setControlword(ds402::controlword::enable_operation);
            if (state == ds402::State::OperationEnabled) {
                csp_track_actual_ = false;  // freeze the held position
                enable_phase_ = EnablePhase::Idle;
                stablecops::log::out() << "fault recovery: operation re-enabled (statusword=";
                writeHex(stablecops::log::out(), feedback_.statusword, 4) << ")\n";
            } else if (now >= enable_phase_deadline_) {
                fail("enable operation");
            }
            break;
        case EnablePhase::Idle:
            break;
    }
}

void MotorDriver::advanceProfileSetpoint() {
    // DS402 Profile Position handshake: pulse controlword bit 4 (new setpoint),
    // wait for statusword bit 12 (setpoint acknowledge), then drop bit 4 and wait
    // for the drive to clear the acknowledge. change-immediately (bit 5) makes a
    // new setpoint take effect at once; relative (bit 6) is optional per move.
    constexpr uint16_t kSetpointAck = 0x1000;  // statusword bit 12
    const bool ack = (feedback_.statusword & kSetpointAck) != 0;

    const uint16_t base = ds402::controlword::enable_operation;
    uint16_t edge =
        base | ds402::controlword::new_setpoint | ds402::controlword::change_immediately;
    if (profile_move_relative_) {
        edge |= ds402::controlword::relative_position;
    }

    switch (setpoint_phase_) {
        case SetpointPhase::Assert:
            setControlword(edge);
            setpoint_phase_ = SetpointPhase::WaitAck;
            break;
        case SetpointPhase::WaitAck:
            setControlword(edge);
            if (ack) {
                setpoint_phase_ = SetpointPhase::Clear;
            }
            break;
        case SetpointPhase::Clear:
            setControlword(base);
            if (!ack) {
                setpoint_phase_ = SetpointPhase::Idle;
            }
            break;
        case SetpointPhase::Idle:
            break;
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
    stablecops::log::out() << "  CSP target position set to " << target << " (delta " << delta
                           << " counts)\n";
}

}  // namespace stablecops::lely
