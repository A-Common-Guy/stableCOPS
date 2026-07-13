#pragma once

#include <chrono>
#include <cstdint>
#include <memory>

#include "stablecops/app/MotorConfig.hpp"
#include "stablecops/ds402/DriveController.hpp"
#include "stablecops/ds402/ObjectAccess.hpp"
#include "stablecops/lely/CyclicStats.hpp"

namespace stablecops::app {

class Bus;

// Integer width/sign of a CANopen object, shared with the ds402 layer.
using ObjectDataType = ds402::ObjectWidth;

// High-level, thread-safe handle to a single CANopen drive.
//
// You only ever hold MotorDrive objects. Each one names a CAN interface and a
// node id. Drives that name the same interface transparently share one hidden,
// ref-counted bus (one Lely master, one event-loop thread, one SYNC); different
// interfaces get independent buses, each with its own loop thread - so several
// chains are just MotorDrives constructed with different interface names.
//
// Lifecycle is static: construct all drives for a chain, then call start(). The
// first start() on any drive of a chain boots the whole chain once; start() on
// siblings is then a no-op. The last MotorDrive released on an interface tears
// that bus down (graceful stop + join). Constructing a new drive for an
// interface that has already started throws.
class MotorDrive {
public:
    explicit MotorDrive(MotorConfig config);
    ~MotorDrive();

    MotorDrive(const MotorDrive&) = delete;
    MotorDrive& operator=(const MotorDrive&) = delete;

    // Boot this drive's chain. The shared bus launches its loop thread, applies
    // any real-time tuning, constructs the Lely application for all registered
    // nodes, and resets the master. Blocks until the application is constructed;
    // rethrows any construction error (e.g. bad profile path or CAN open
    // failure). Idempotent and shared across all drives on the interface.
    void start();

    // Request a graceful stop of this drive (ramp down / de-energise if
    // enabled). Does not tear down the shared bus (siblings keep running); the
    // bus is torn down when the last MotorDrive on the interface is destroyed.
    void stop();

    // Controlled quick stop: decelerate on the drive's quick-stop ramp (rather
    // than coasting like stop()) and hold in quick-stop-active while energised,
    // when the drive's quick-stop option code (0x605A) is configured to stay
    // there. Keeps the power stage on and does not tear down the shared bus;
    // recover with enableOperation() or resetFault(). Prefer this over stop()
    // for a loaded or vertical axis, where coasting would drop the load.
    void quickStop();

    // Tooling-oriented controls for the shared bus. shutdownBus() gracefully
    // tears down the whole chain; forceStopBus() breaks a stuck graceful
    // shutdown by stopping the CANopen loop immediately.
    void shutdownBus();
    bool forceStopBus();

    bool running() const;

    // Latest feedback snapshot, safe to call from any thread. feedbackLive() is
    // true only while fresh cyclic feedback keeps arriving; it goes false if the
    // drive stops talking (the same staleness that trips the safety watchdog).
    ds402::Feedback feedback() const;
    bool feedbackLive() const;

    // Block until feedbackLive() is true or `timeout` elapses; returns whether
    // feedback is live. Typical use: right after start(), before reading the
    // first position.
    bool waitUntilLive(std::chrono::milliseconds timeout) const;

    // Latest output-shaft angle, converted from feedback().position using the
    // configured counts_per_rev. Convenience over the raw count.
    double positionDegrees() const;
    double positionRadians() const;

    // Drive status, derived from the latest feedback. enabled() requires live
    // feedback so a stale "operation enabled" snapshot never reads as enabled.
    bool enabled() const;
    bool faulted() const;
    uint16_t errorCode() const;

    // Clear a latched fault and recover to the configured operating state
    // (re-enabled if the drive was meant to run, otherwise safely disabled).
    // Applied on the loop thread; observe the result via faulted()/enabled().
    void resetFault();

    // Walk the CiA402 state machine to Operation Enabled at runtime using the
    // same safe ladder as boot-time enable. `hold_position` primes the current
    // CSP target before enabling, so a position-mode drive does not step.
    void enableOperation(bool hold_position = true);

    // Request a CiA402 operation mode change (0x6060). The drive may reject this
    // over SDO depending on its state/firmware; errors are surfaced to the
    // caller instead of silently falling back.
    void setOperationMode(ds402::OperationMode mode);

    // Typed CANopen object access routed to the Lely loop thread. Reads of
    // PDO-mapped feedback may return the cached TPDO value; writes to mapped
    // command objects are staged into the cyclic command image. Other objects use
    // SDO through MotorDriver's ObjectAccess implementation.
    int64_t readObject(uint16_t index, uint8_t subindex, ObjectDataType type) const;
    void writeObject(uint16_t index, uint8_t subindex, ObjectDataType type, int64_t value);

    // Runtime cyclic setpoints, applied on the loop thread for the next SYNC.
    // They take effect only when the drive is enabled in the matching cyclic
    // mode and the target object is mapped into an active RxPDO.
    void commandPosition(int32_t counts);
    void commandVelocity(int32_t units);
    void commandTorque(int16_t units);

    // Profile Position move (drive runs its own trajectory using the configured
    // profile velocity/accel/decel). Absolute by default; relative adds to the
    // current position. Effective only when enabled in Profile Position mode.
    void moveToPosition(int32_t counts, bool relative = false);

    // Start and observe a hardstop-midpoint homing routine. The drive must
    // already be enabled in Cyclic Synchronous Velocity mode; callers should use
    // the proven sequence stop -> mode CSV -> enable -> startHoming().
    void startHoming(const ds402::HomingConfig& config);
    ds402::HomingPhase homingPhase() const;
    ds402::HomingResult homingResult() const;

    // Measured cyclic cadence of this drive's bus (shared by all drives on the
    // interface): interval min/max/mean and worst-case jitter vs the nominal
    // SYNC period. Use it to verify achieved latency/jitter.
    stablecops::lely::CyclicStats cyclicStats() const;

private:
    MotorConfig config_;
    uint8_t node_id_;
    // The shared bus for config_.can_interface. Several MotorDrives on the same
    // interface hold the same Bus; it lives until the last one is released.
    std::shared_ptr<Bus> bus_;
};

}  // namespace stablecops::app
