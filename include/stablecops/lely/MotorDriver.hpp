#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <system_error>
#include <vector>

#include <lely/coapp/fiber_driver.hpp>

#include "stablecops/config/PdoMap.hpp"
#include "stablecops/ds402/DriveController.hpp"
#include "stablecops/ds402/ObjectAccess.hpp"

namespace stablecops::lely {

struct BootActionConfig {
    bool inspect{false};
    bool enable{false};
    bool hold_position{false};
    bool monitor{false};
    // When set, selected over SDO (0x6060) while pre-operational at boot.
    std::optional<ds402::OperationMode> mode;
    // Profile-mode parameters, written over SDO at boot when set.
    std::optional<uint32_t> profile_velocity;
    std::optional<uint32_t> profile_acceleration;
    std::optional<uint32_t> profile_deceleration;
    std::optional<uint32_t> torque_slope;
    std::optional<int32_t> csp_target_position;
    std::optional<int32_t> csp_relative_move;
    int32_t max_position_step{10000};
    // Output-shaft counts per revolution of 0x6064, for the degrees readout.
    uint32_t counts_per_rev{524288};
    std::chrono::milliseconds state_transition_timeout{2000};
    // Feedback-staleness watchdog window; 0 disables it.
    std::chrono::milliseconds feedback_timeout{100};
    // Nominal cyclic period in microseconds, used as the reference for jitter
    // telemetry (should match the master's SYNC period from the DCF).
    uint32_t sync_period_us{1000};
};

// Measured cadence of the cyclic SYNC, accumulated on the loop thread and
// published for other threads. Intervals are between consecutive OnSync calls;
// jitter is the worst absolute deviation from the nominal period. All times in
// microseconds. cycles is the number of measured intervals.
struct CyclicStats {
    uint64_t cycles{0};
    double last_us{0.0};
    double min_us{0.0};
    double max_us{0.0};
    double mean_us{0.0};
    double max_jitter_us{0.0};
};

// CANopen integer type of a PDO-mapped object, derived from its bit length and
// signedness. Used to read/write each mapped object with the correct width and
// sign on the cyclic path.
enum class PdoDataType { U8, I8, U16, I16, U32, I32 };

// One object that rides in a cyclic PDO, keyed by (index, subindex). For command
// objects `value` holds the latest commanded value the master streams; for
// feedback objects it holds the most recently received raw value.
struct CyclicObject {
    uint16_t index{0};
    uint8_t subindex{0};
    PdoDataType type{PdoDataType::U32};
    int64_t value{0};
};

class MotorDriver final : public ::lely::canopen::FiberDriver, public ds402::ObjectAccess {
public:
    MotorDriver(::lely::canopen::AsyncMaster& master, uint8_t node_id,
                BootActionConfig boot_actions, config::PdoMap pdo_map);

    ds402::DriveController& drive();
    const ds402::DriveController& drive() const;

    // Start a Profile Position move to an absolute (or relative) target. Safe to
    // call from the event loop (e.g. via CanopenApplication::post). The target is
    // staged into the cyclic command image and the DS402 new-setpoint handshake
    // (controlword bit 4 vs statusword bit 12) is driven from OnSync. Only
    // meaningful once the drive is enabled in Profile Position mode.
    void requestProfileMove(int32_t target_position, bool relative);

    // Clear a latched fault and (if the drive was configured to run) walk it
    // back up to operation enabled, all non-blocking from the cyclic path. Safe
    // to call from the event loop (e.g. via CanopenApplication::post). Targets
    // are reset to a safe hold (zero velocity/torque, position glued to actual)
    // before re-enabling so recovery never produces a step.
    void requestFaultReset();

    // Runtime CiA402 enable using the same safe ladder as boot-time enable. Safe
    // to call from the event loop after the node has booted and PDO monitoring
    // has been configured.
    void requestEnableOperation(bool hold_position);

    // Request a CiA402 operation-mode change by writing 0x6060 over SDO. When
    // changing into a position mode while already enabled, the position target is
    // first glued to the actual position so the mode switch cannot introduce a
    // step. Safe to call from the driver's fiber executor.
    void requestOperationMode(ds402::OperationMode mode);

    // Begin a non-blocking shutdown: immediately command disable-voltage so the
    // power stage drops and the joint goes limp (coasts, no brake), then invoke
    // the stopped-callback once it is confirmed de-energised or after a bounded
    // timeout. Safe to call from the event loop (e.g. a signal handler); the
    // disable command is streamed from OnSync so SYNC keeps flowing meanwhile.
    void requestGracefulStop();
    void setStoppedCallback(std::function<void()> on_stopped);

    // Thread-safe snapshot of the latest feedback, published every cycle from the
    // loop thread. Safe to call from any thread (e.g. an application reading
    // telemetry while the Lely event loop runs on its own thread).
    ds402::Feedback feedbackSnapshot() const;
    bool feedbackLive() const;

    // Runs hardstop-midpoint homing. The routine snapshots the current mode,
    // switches safely through CSV for the search, then restores the previous
    // mode and enabled state when complete.
    void requestHoming(const ds402::HomingConfig& config);
    ds402::HomingPhase homingPhase() const;
    ds402::HomingResult homingResult() const;

    // Thread-safe snapshot of the measured cyclic cadence (interval min/max/mean
    // and worst-case jitter vs the nominal period). Published every OnSync.
    CyclicStats cyclicStats() const;

    uint8_t readU8(uint16_t index, uint8_t subindex) override;
    uint16_t readU16(uint16_t index, uint8_t subindex) override;
    uint32_t readU32(uint16_t index, uint8_t subindex) override;
    int32_t readI32(uint16_t index, uint8_t subindex) override;

    void writeU8(uint16_t index, uint8_t subindex, uint8_t value) override;
    void writeU16(uint16_t index, uint8_t subindex, uint16_t value) override;
    void writeU32(uint16_t index, uint8_t subindex, uint32_t value) override;
    void writeI32(uint16_t index, uint8_t subindex, int32_t value) override;

protected:
    void OnBoot(::lely::canopen::NmtState state, char error,
                const std::string& reason) noexcept override;
    void OnConfig(std::function<void(std::error_code)> result) noexcept override;
    void OnRpdoWrite(uint16_t index, uint8_t subindex) noexcept override;
    void OnSync(uint8_t counter, const time_point& time) noexcept override;

private:
    void advanceHoming();
    void failHoming(const std::string& reason);
    bool homingContactDetected(int direction, std::chrono::steady_clock::time_point now);
    bool homingTimedOut(std::chrono::steady_clock::time_point now) const;

    bool wantsMotionAction() const;
    bool wantsCyclicConfig() const;
    std::error_code configurePdos() noexcept;
    std::error_code selectOperationMode() noexcept;
    std::error_code configureProfileParameters() noexcept;
    void advanceProfileSetpoint();
    void inspectNode() noexcept;
    void runBootActions() noexcept;
    ds402::Feedback waitForDriveState(
        ds402::State expected, std::chrono::milliseconds timeout,
        std::chrono::milliseconds poll_interval = std::chrono::milliseconds{20});
    void primeCommandImage();
    void enableDrive(bool hold_position);
    void applyCspTarget();
    void setControlword(uint16_t controlword);
    void advanceGracefulStop();
    void advanceEnableLadder();
    bool feedbackStale(std::chrono::steady_clock::time_point now) const;
    void logFaultTransition();
    bool isDriveDeEnergized() const;
    void finishGracefulStop();

    bool isCommandObject(uint16_t index) const;
    bool isFeedbackObject(uint16_t index) const;

    // Cyclic exchange driven entirely by the loaded PdoMap.
    void buildCyclicObjects();
    CyclicObject* findCommand(uint16_t index);
    const CyclicObject* findCommand(uint16_t index) const;
    bool hasCommand(uint16_t index) const;
    int64_t commandValue(uint16_t index) const;
    void setCommandValue(uint16_t index, int64_t value);
    int64_t readMappedObject(const CyclicObject& object, std::error_code& ec);
    void writeMappedObject(const CyclicObject& object, std::error_code& ec);
    void decodeFeedbackObject(uint16_t index, int64_t raw);
    void publishFeedback();

    ds402::DriveController drive_;
    BootActionConfig boot_actions_;
    config::PdoMap pdo_map_;

    enum class StopPhase { None, DisableVoltage, Done };

    // Active RxPDO objects the master transmits to the drive (commands) and
    // active TxPDO objects it receives (feedback), built once from pdo_map_.
    std::vector<CyclicObject> command_objects_;
    std::vector<CyclicObject> feedback_objects_;
    ds402::Feedback feedback_;

    void updateCyclicStats(std::chrono::steady_clock::time_point now);

    // Published snapshot for other threads; written under the lock at the end of
    // each OnSync, read by feedbackSnapshot().
    mutable std::mutex feedback_mutex_;
    ds402::Feedback feedback_published_;
    CyclicStats stats_published_;
    std::atomic<bool> feedback_live_{false};

    // Cyclic cadence measurement, owned by the loop thread.
    std::chrono::steady_clock::time_point last_sync_time_{};
    double interval_sum_us_{0.0};
    CyclicStats stats_{};
    bool cyclic_active_{false};
    bool csp_track_actual_{false};
    bool rpdo_seen_{false};
    uint64_t sync_count_{0};

    ds402::HomingPhase homing_phase_{ds402::HomingPhase::Idle};
    ds402::HomingConfig homing_config_;
    ds402::HomingResult homing_result_;
    int32_t homing_start_position_{0};
    int32_t homing_backoff_target_{0};
    int32_t homing_center_target_{0};
    ds402::OperationMode homing_restore_mode_{ds402::OperationMode::CyclicSynchronousPosition};
    std::chrono::steady_clock::time_point homing_deadline_{};
    std::chrono::steady_clock::time_point homing_contact_since_{};
    std::chrono::steady_clock::time_point homing_settle_until_{};
    bool homing_contact_active_{false};
    bool homing_restore_enabled_{false};

    // Profile Position new-setpoint handshake, driven from OnSync so the
    // controlword edge is produced coherently in the cyclic stream.
    enum class SetpointPhase { Idle, Assert, WaitAck, Clear };
    SetpointPhase setpoint_phase_{SetpointPhase::Idle};
    bool profile_move_relative_{false};

    // Non-blocking fault-reset + re-enable ladder, driven from OnSync.
    enum class EnablePhase { Idle, FaultReset, Shutdown, SwitchOn, EnableOp };
    EnablePhase enable_phase_{EnablePhase::Idle};
    bool recover_to_enabled_{false};
    std::chrono::steady_clock::time_point enable_phase_deadline_{};

    // Feedback-staleness watchdog state.
    std::chrono::steady_clock::time_point last_feedback_time_{};
    bool fault_active_logged_{false};

    StopPhase stop_phase_{StopPhase::None};
    std::chrono::steady_clock::time_point stop_phase_deadline_{};
    std::chrono::steady_clock::time_point stop_log_due_{};
    std::function<void()> on_stopped_{};
};

}  // namespace stablecops::lely
