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
#include "stablecops/lely/CyclicStats.hpp"

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
    // Vendor "Disable Mode" (0x2103), written over SDO at boot when set. Selects
    // the power-stage behaviour on disable (coast/high-impedance vs short-circuit
    // dynamic braking); concrete values are drive-specific.
    std::optional<uint8_t> disable_mode;
    // Ad-hoc raw object writes applied over SDO at boot (pre-operational), in
    // order, for drive configuration/experimentation.
    std::vector<ds402::ObjectWrite> object_writes;
    // Persist the configuration objects written at boot to NVM (0x1010:03).
    bool save_params{false};
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

// One object that rides in a cyclic PDO, keyed by (index, subindex). For command
// objects `value` holds the latest commanded value the master streams; for
// feedback objects it holds the most recently received raw value. `type` is the
// CANopen integer type derived from the mapped bit length and signedness.
struct CyclicObject {
    uint16_t index{0};
    uint8_t subindex{0};
    ds402::ObjectWidth type{ds402::ObjectWidth::U32};
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
    // step. When `confirm` is set, block on this driver's fiber until the drive
    // reflects the new mode in 0x6061 (throwing on timeout); `confirm` must stay
    // false when called from the cyclic path (OnSync), which cannot block. Safe
    // to call from the driver's fiber executor.
    void requestOperationMode(ds402::OperationMode mode, bool confirm = false);

    // Begin a non-blocking shutdown: immediately command disable-voltage so the
    // power stage drops and the joint goes limp (coasts, no brake), then invoke
    // the stopped-callback once it is confirmed de-energised or after a bounded
    // timeout. Safe to call from the event loop (e.g. a signal handler); the
    // disable command is streamed from OnSync so SYNC keeps flowing meanwhile.
    void requestGracefulStop();
    void setStoppedCallback(std::function<void()> on_stopped);

    // Controlled quick stop: command the CiA402 quick-stop (controlword 0x02) so
    // the drive decelerates on its quick-stop ramp instead of coasting, then
    // holds in quick-stop-active (still energised) when its quick-stop option
    // code (0x605A) is configured to stay there. Motion targets are zeroed and
    // the position target glued first so a later re-enable never steps. Unlike
    // requestGracefulStop this keeps the power stage on and does not tear the bus
    // down; recover with requestEnableOperation()/requestFaultReset(). Safe to
    // call from the event loop.
    void requestQuickStop();

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

    // Standard CANopen fault channels, orthogonal to the cyclic TPDO stream.
    // OnEmcy surfaces the drive's emergency messages (eec/error register/vendor
    // bytes) into the fault path; OnState/OnHeartbeat/OnNodeGuarding detect node
    // loss via the master's consumer heartbeat (or node guarding) independently
    // of PDO cadence.
    void OnEmcy(uint16_t eec, uint8_t er, uint8_t msef[5]) noexcept override;
    void OnState(::lely::canopen::NmtState state) noexcept override;
    void OnHeartbeat(bool occurred) noexcept override;
    void OnNodeGuarding(bool occurred) noexcept override;

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

    // Common reaction to a lost/restored node reported by heartbeat or node
    // guarding: log once, drop liveness, and de-energise if still running.
    void handleNodeLoss(const char* channel, bool lost);

    // True for the DS402 objects this firmware only accepts via PDO (controlword
    // and the motion targets); SDO downloads to them abort with vendor code
    // 0x00000002, so unmapped writes are dropped with a warning instead.
    bool isPdoOnlyCommand(uint16_t index) const;
    bool isFeedbackObject(uint16_t index) const;
    // Stage a write into the cyclic command image when the object is mapped,
    // drop it (with a warning) when it is PDO-only but unmapped, and fall back
    // to a blocking SDO download otherwise. Shared by the typed write overrides.
    template <typename T>
    void writeStagedOrSdo(uint16_t index, uint8_t subindex, T value);
    // True when `index` actually rides in an active TxPDO (i.e. its cached value
    // is refreshed every cycle), so a read can be served from the snapshot
    // instead of a blocking SDO round trip.
    bool feedbackMapped(uint16_t index) const;

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

    enum class StopPhase : uint8_t { None, DisableVoltage, Done };

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
    // Set while a homing phase has a blocking SDO exchange in flight on its
    // fiber. OnSync keeps arriving during that suspension (each callback runs on
    // its own fiber), so advanceHoming() must not re-enter and re-issue the
    // same SDO writes.
    bool homing_command_in_flight_{false};

    // Profile Position new-setpoint handshake, driven from OnSync so the
    // controlword edge is produced coherently in the cyclic stream.
    enum class SetpointPhase : uint8_t { Idle, Assert, WaitAck, Clear };
    SetpointPhase setpoint_phase_{SetpointPhase::Idle};
    bool profile_move_relative_{false};

    // Non-blocking fault-reset + re-enable ladder, driven from OnSync.
    enum class EnablePhase : uint8_t { Idle, FaultReset, Shutdown, SwitchOn, EnableOp };
    EnablePhase enable_phase_{EnablePhase::Idle};
    bool recover_to_enabled_{false};
    std::chrono::steady_clock::time_point enable_phase_deadline_{};

    // Feedback-staleness watchdog state.
    std::chrono::steady_clock::time_point last_feedback_time_{};
    bool fault_active_logged_{false};

    // Node-loss latch shared by the heartbeat and node-guarding channels, so the
    // reaction and logging happen once per loss regardless of which fired.
    bool node_loss_{false};

    StopPhase stop_phase_{StopPhase::None};
    std::chrono::steady_clock::time_point stop_phase_deadline_{};
    std::chrono::steady_clock::time_point stop_log_due_{};
    std::function<void()> on_stopped_{};
};

}  // namespace stablecops::lely
