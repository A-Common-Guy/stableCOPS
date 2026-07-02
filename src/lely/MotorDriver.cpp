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

#include "stablecops/ds402/ObjectDictionary.hpp"

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

PdoDataType objectTypeFor(uint16_t index, uint8_t bit_length) {
    const bool is_signed = isSignedObject(index);
    switch (bit_length) {
        case 8:
            return is_signed ? PdoDataType::I8 : PdoDataType::U8;
        case 16:
            return is_signed ? PdoDataType::I16 : PdoDataType::U16;
        case 32:
        default:
            return is_signed ? PdoDataType::I32 : PdoDataType::U32;
    }
}

// How often to log progress while waiting for the drive to confirm it has
// dropped its power stage during shutdown.
constexpr auto kPowerOffRetryLogInterval = std::chrono::milliseconds(200);

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
        case PdoDataType::U8:
            return sub.Read<uint8_t>(ec);
        case PdoDataType::I8:
            return sub.Read<int8_t>(ec);
        case PdoDataType::U16:
            return sub.Read<uint16_t>(ec);
        case PdoDataType::I16:
            return sub.Read<int16_t>(ec);
        case PdoDataType::U32:
            return sub.Read<uint32_t>(ec);
        case PdoDataType::I32:
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
        case PdoDataType::U8:
            sub.Write(static_cast<uint8_t>(object.value), ec);
            return;
        case PdoDataType::I8:
            sub.Write(static_cast<int8_t>(object.value), ec);
            return;
        case PdoDataType::U16:
            sub.Write(static_cast<uint16_t>(object.value), ec);
            return;
        case PdoDataType::I16:
            sub.Write(static_cast<int16_t>(object.value), ec);
            return;
        case PdoDataType::U32:
            sub.Write(static_cast<uint32_t>(object.value), ec);
            return;
        case PdoDataType::I32:
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
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    feedback_published_ = feedback_;
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
                          feedback_.error_code != 0;
    if (in_fault && !fault_active_logged_) {
        std::cerr << "drive FAULT: state=" << ds402::toString(feedback_.state) << " statusword=";
        writeHex(std::cerr, feedback_.statusword, 4) << " error_code=";
        writeHex(std::cerr, feedback_.error_code, 4) << '\n';
        fault_active_logged_ = true;
    } else if (!in_fault && fault_active_logged_) {
        std::cout << "drive fault cleared (state=" << ds402::toString(feedback_.state) << ")\n";
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

    std::lock_guard<std::mutex> lock(feedback_mutex_);
    stats_published_ = stats_;
}

ds402::DriveController& MotorDriver::drive() {
    return drive_;
}

const ds402::DriveController& MotorDriver::drive() const {
    return drive_;
}

bool MotorDriver::isCommandObject(uint16_t index) const {
    switch (index) {
        case ds402::od::controlword:
        case ds402::od::target_position:
        case ds402::od::target_velocity:
        case ds402::od::target_torque:
        case ds402::od::profile_velocity:
        case ds402::od::profile_acceleration:
        case ds402::od::profile_deceleration:
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

uint8_t MotorDriver::readU8(uint16_t index, uint8_t subindex) {
    if (index == ds402::od::modes_of_operation_display && rpdo_seen_) {
        return static_cast<uint8_t>(static_cast<int8_t>(feedback_.mode));
    }
    return Wait(AsyncRead<uint8_t>(index, subindex));
}

uint16_t MotorDriver::readU16(uint16_t index, uint8_t subindex) {
    if (rpdo_seen_) {
        if (index == ds402::od::statusword) {
            return feedback_.statusword;
        }
        if (index == ds402::od::torque_actual_value) {
            return static_cast<uint16_t>(feedback_.torque);
        }
    }
    return Wait(AsyncRead<uint16_t>(index, subindex));
}

uint32_t MotorDriver::readU32(uint16_t index, uint8_t subindex) {
    return Wait(AsyncRead<uint32_t>(index, subindex));
}

int32_t MotorDriver::readI32(uint16_t index, uint8_t subindex) {
    if (rpdo_seen_) {
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
// DS402 command object that is not currently mapped it is dropped (those objects
// are PDO-only on this firmware and abort SDO downloads); anything else falls
// back to a blocking SDO write (configuration objects).
void MotorDriver::writeU8(uint16_t index, uint8_t subindex, uint8_t value) {
    if (auto* object = findCommand(index)) {
        object->value = value;
        return;
    }
    if (isCommandObject(index)) {
        return;
    }
    Wait(AsyncWrite(index, subindex, value));
}

void MotorDriver::writeU16(uint16_t index, uint8_t subindex, uint16_t value) {
    if (auto* object = findCommand(index)) {
        object->value = value;
        return;
    }
    if (isCommandObject(index)) {
        return;
    }
    Wait(AsyncWrite(index, subindex, value));
}

void MotorDriver::writeU32(uint16_t index, uint8_t subindex, uint32_t value) {
    if (auto* object = findCommand(index)) {
        object->value = value;
        return;
    }
    if (isCommandObject(index)) {
        return;
    }
    Wait(AsyncWrite(index, subindex, value));
}

void MotorDriver::writeI32(uint16_t index, uint8_t subindex, int32_t value) {
    if (auto* object = findCommand(index)) {
        object->value = value;
        return;
    }
    if (isCommandObject(index)) {
        return;
    }
    Wait(AsyncWrite(index, subindex, value));
}

void MotorDriver::OnBoot(::lely::canopen::NmtState state, char error,
                         const std::string& reason) noexcept {
    if (error != 0) {
        std::cerr << "node " << static_cast<int>(id()) << " boot failed: " << reason << " ("
                  << error << ")\n";
        return;
    }

    std::cout << "node " << static_cast<int>(id()) << " booted, NMT state "
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
            std::cerr << "node " << static_cast<int>(id())
                      << " configuration failed: " << ec.message() << '\n';
        }
    }
    result(ec);
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
        std::cerr << "  SDO write to modes of operation (0x6060:00) failed: " << ec.message()
                  << '\n';
        return ec;
    }
    std::cout << "  operation mode set to " << ds402::toString(mode)
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
            std::cerr << "  SDO write to " << what << " (";
            writeHex(std::cerr, index, 4) << ":00) failed: " << ec.message() << '\n';
            return false;
        }
        std::cout << "  " << what << " set to " << *value << '\n';
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
    return {};
}

std::error_code MotorDriver::configurePdos() noexcept {
    std::cout << "node " << static_cast<int>(id())
              << ": configuring drive PDOs for cyclic synchronous operation\n";

    std::error_code ec;
    const auto download = [&](uint16_t index, uint8_t subindex, auto value,
                              const char* what) -> bool {
        Wait(AsyncWrite(index, subindex, value), ec);
        if (ec) {
            std::cerr << "  SDO write to " << what << " (";
            writeHex(std::cerr, index, 4)
                << ':' << std::setw(2) << std::setfill('0') << std::uppercase << std::hex
                << static_cast<int>(subindex) << std::dec << ") failed: " << ec.message() << '\n';
        }
        return !ec;
    };

    // The drive ships its PDOs with transmission type 0 (event-driven), so out
    // of NVM it never streams TPDOs on SYNC and never runs the cyclic exchange.
    // PDO parameters are only reliably SDO-writable while the node is
    // pre-operational, so we rewrite them here (manual Table 5-5). The layout
    // comes entirely from the generated summary.json (pdo_map_), which is
    // derived from the same profile as dcf/master.dcf, so both ends of the bus
    // stay coherent.
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
        std::cout << "node " << static_cast<int>(id()) << ": " << role << " 0x" << std::hex
                  << pdo.comm_index << std::dec
                  << " COB-ID 0x" << std::hex << base_cob_id << std::dec
                  << " set to transmission type " << static_cast<int>(pdo.transmission_type) << ", "
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
            std::cout << "node " << static_cast<int>(id()) << ": RxPDO 0x" << std::hex
                      << pdo.comm_index << std::dec
                      << " disabled (unused)\n";
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
        publishFeedback();
    }

    // Liveness reflects staleness rather than latching: it is true only while
    // fresh feedback keeps arriving, so readers (and the facade) can trust it.
    const bool stale = feedbackStale(now);
    feedback_live_.store(rpdo_seen_ && !stale, std::memory_order_release);

    // Safety watchdog: if the drive stops talking while energised, drop the
    // power stage. requestGracefulStop is one-shot, so this logs/acts once.
    if (cyclic_active_ && stop_phase_ == StopPhase::None && stale) {
        std::cerr << "feedback watchdog: no drive feedback for >"
                  << boot_actions_.feedback_timeout.count() << " ms; de-energising\n";
        requestGracefulStop();
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
        std::cout << "  decoded state = " << ds402::toString(ds402::decodeState(statusword))
                  << '\n';
    }

    read_u32(0x6502, 0x00, "supported modes");
    read_u8(ds402::od::modes_of_operation, ds402::od::default_subindex, "commanded mode");
    const auto [display_mode, display_mode_ok] = read_u8(
        ds402::od::modes_of_operation_display, ds402::od::default_subindex, "displayed mode");
    if (display_mode_ok) {
        const auto mode = static_cast<ds402::OperationMode>(static_cast<int8_t>(display_mode));
        std::cout << "  decoded mode = " << ds402::toString(mode) << '\n';
    }

    const auto [position_counts, position_ok] =
        read_i32(ds402::od::position_actual_value, ds402::od::default_subindex, "position");
    if (position_ok && boot_actions_.counts_per_rev != 0) {
        std::cout << "    -> "
                  << (static_cast<double>(position_counts) / boot_actions_.counts_per_rev * 360.0)
                  << " deg (" << boot_actions_.counts_per_rev << " counts/rev)\n";
    }
    read_i32(ds402::od::velocity_actual_value, ds402::od::default_subindex, "velocity");
    read_u16(ds402::od::torque_actual_value, ds402::od::default_subindex, "torque");
    read_u16(ds402::od::error_code, ds402::od::default_subindex, "error code");

    // Position source and scaling. 0x6064 (the value we stream as feedback) is
    // the control-loop position: it is derived from the primary (motor) encoder,
    // scaled by the encoder resolution (0x608F) and gear ratio (0x6091), then
    // referenced to the home offset (0x607C). To tell whether it tracks the
    // motor or the load, rotate the joint a known amount and watch which counter
    // moves how much: 0x6063/0x6064 are the scaled control position, the single-
    // turn absolute registers (0x276F/0x2772) are the raw per-revolution encoder
    // angle, and 0x221A selects whether a second (load-side) encoder is used.
    std::cout << " position feedback & scaling\n";
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
    std::cout << " PDO configuration (drive side)\n";
    const auto dump_pdo = [&](const char* label, uint16_t comm_index, uint16_t map_index) {
        const auto [cob_id, cob_ok] = read_u32(comm_index, 0x01, label);
        if (cob_ok) {
            const bool valid = (cob_id & 0x80000000u) == 0;
            std::cout << "    -> COB-ID 0x" << std::hex << (cob_id & 0x7FF) << std::dec
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
    std::cout << " disable / brake configuration\n";
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

    std::cout << " vendor runtime monitors\n";
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
    // drive and to merely observe its feedback cyclically.
    return wantsMotionAction() || boot_actions_.monitor;
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
        std::cerr << "boot action failed: " << exception.what() << '\n';
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
    std::cout << "graceful stop: drive disabled (final state " << ds402::toString(feedback_.state)
              << ", statusword=";
    writeHex(std::cout, feedback_.statusword, 4) << ")\n";
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
        std::cout << "graceful stop: drive not energised; stopping master\n";
        stop_phase_ = StopPhase::Done;
        if (on_stopped_) {
            on_stopped_();
        }
        return;
    }

    std::cout << "graceful stop: de-energising drive (disable voltage)\n";
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
        std::cout << "graceful stop: power stage off (state " << ds402::toString(feedback_.state)
                  << ")\n";
        finishGracefulStop();
        return;
    }

    if (now >= stop_phase_deadline_) {
        std::cerr << "graceful stop: timed out waiting for de-energize; forcing "
                     "shutdown (state="
                  << ds402::toString(feedback_.state) << " statusword=";
        writeHex(std::cerr, feedback_.statusword, 4) << ")\n";
        finishGracefulStop();
        return;
    }

    if (now >= stop_log_due_) {
        std::cout << "graceful stop: waiting for drive to de-energize; state="
                  << ds402::toString(feedback_.state) << " statusword=";
        writeHex(std::cout, feedback_.statusword, 4) << " controlword=";
        writeHex(std::cout, static_cast<uint32_t>(commandValue(ds402::od::controlword)), 4)
            << " torque=" << static_cast<int>(feedback_.torque)
            << " velocity=" << feedback_.velocity << '\n';
        stop_log_due_ = now + kPowerOffRetryLogInterval;
    }
}

void MotorDriver::enableDrive(bool prime_csp_target) {
    auto feedback = drive_.readFeedback();
    std::cout << "enabling DS402 operation from state " << ds402::toString(feedback.state)
              << ", mode " << ds402::toString(feedback.mode) << '\n';

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

    std::cout << "  cyclic streaming active: sync cycles=" << sync_count_
              << ", drive TPDO feedback "
              << (rpdo_seen_ ? "LIVE (cyclic)" : "NOT received (SDO fallback)") << "; statusword=";
    writeHex(std::cout, feedback_.statusword, 4) << '\n';

    if (feedback.state == ds402::State::OperationEnabled) {
        setControlword(ds402::controlword::enable_operation);
        csp_track_actual_ = false;
        std::cout << "  drive already in operation enabled; holding position "
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

    std::cout << "  enabled operation, statusword=";
    writeHex(std::cout, feedback.statusword, 4)
        << " state=" << ds402::toString(feedback.state) << '\n';

    if (csp_mode) {
        std::cout << "  holding current position at " << commandValue(ds402::od::target_position)
                  << '\n';
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
    std::cout << "fault reset requested (recover to "
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

void MotorDriver::requestOperationMode(ds402::OperationMode mode) {
    if (feedback_.state == ds402::State::OperationEnabled && std::abs(feedback_.velocity) > 0) {
        throw std::runtime_error("refusing to switch operation mode while velocity is nonzero");
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
    std::cout << "operation mode requested: " << ds402::toString(mode)
              << " (0x6060=" << static_cast<int>(static_cast<int8_t>(mode)) << ")\n";
}

void MotorDriver::advanceEnableLadder() {
    const auto now = std::chrono::steady_clock::now();
    const auto state = feedback_.state;

    const auto fail = [&](const char* during) {
        std::cerr << "fault recovery: timed out during " << during
                  << " (state=" << ds402::toString(state) << ", statusword=";
        writeHex(std::cerr, feedback_.statusword, 4) << ")\n";
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
                    std::cout << "fault recovery: fault cleared; drive disabled\n";
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
                std::cout << "fault recovery: operation re-enabled (statusword=";
                writeHex(std::cout, feedback_.statusword, 4) << ")\n";
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
    std::cout << "  CSP target position set to " << target << " (delta " << delta << " counts)\n";
}

}  // namespace stablecops::lely
