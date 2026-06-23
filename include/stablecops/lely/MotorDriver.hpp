#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <optional>
#include <system_error>

#include <lely/coapp/fiber_driver.hpp>

#include "stablecops/config/PdoMap.hpp"
#include "stablecops/ds402/DriveController.hpp"
#include "stablecops/ds402/ObjectAccess.hpp"

namespace stablecops::lely {

struct BootActionConfig {
    bool inspect{false};
    bool enable{false};
    bool hold_position{false};
    std::optional<int32_t> csp_target_position;
    std::optional<int32_t> csp_relative_move;
    int32_t max_position_step{10000};
    std::chrono::milliseconds state_transition_timeout{2000};
};

// Coherent snapshot of every command object the master transmits to the drive.
// The vendor groups several command objects into each RPDO, so the master must
// always emit a full, consistent image; updating a single object in isolation
// would zero its PDO neighbours on the wire.
struct CommandImage {
    uint16_t controlword{0x0006};       // 0x6040
    int8_t mode{0};                     // 0x6060 modes of operation
    int32_t target_position{0};         // 0x607A
    int32_t target_velocity{0};         // 0x60FF
    uint32_t profile_velocity{0};       // 0x6081
    uint32_t profile_acceleration{0};   // 0x6083
    uint32_t profile_deceleration{0};   // 0x6084
};

class MotorDriver final : public ::lely::canopen::FiberDriver,
                          public ds402::ObjectAccess {
public:
    MotorDriver(::lely::canopen::AsyncMaster& master,
                uint8_t node_id,
                BootActionConfig boot_actions,
                config::PdoMap pdo_map);

    ds402::DriveController& drive();
    const ds402::DriveController& drive() const;

    // Begin a non-blocking shutdown: immediately command disable-voltage so the
    // power stage drops and the joint goes limp (coasts, no brake), then invoke
    // the stopped-callback once it is confirmed de-energised or after a bounded
    // timeout. Safe to call from the event loop (e.g. a signal handler); the
    // disable command is streamed from OnSync so SYNC keeps flowing meanwhile.
    void requestGracefulStop();
    void setStoppedCallback(std::function<void()> on_stopped);

    uint8_t readU8(uint16_t index, uint8_t subindex) override;
    uint16_t readU16(uint16_t index, uint8_t subindex) override;
    uint32_t readU32(uint16_t index, uint8_t subindex) override;
    int32_t readI32(uint16_t index, uint8_t subindex) override;

    void writeU8(uint16_t index, uint8_t subindex, uint8_t value) override;
    void writeU16(uint16_t index, uint8_t subindex, uint16_t value) override;
    void writeU32(uint16_t index, uint8_t subindex, uint32_t value) override;
    void writeI32(uint16_t index, uint8_t subindex, int32_t value) override;

protected:
    void OnBoot(::lely::canopen::NmtState state,
                char error,
                const std::string& reason) noexcept override;
    void OnConfig(std::function<void(std::error_code)> result) noexcept override;
    void OnRpdoWrite(uint16_t index, uint8_t subindex) noexcept override;
    void OnSync(uint8_t counter, const time_point& time) noexcept override;

private:
    bool wantsMotionAction() const;
    std::error_code configurePdos() noexcept;
    void inspectNode() noexcept;
    void runBootActions() noexcept;
    ds402::Feedback waitForDriveState(ds402::State expected,
                                      std::chrono::milliseconds timeout,
                                      std::chrono::milliseconds poll_interval =
                                          std::chrono::milliseconds{20});
    void primeCommandImage();
    void enableDrive(bool hold_position);
    void applyCspTarget();
    void setControlword(uint16_t controlword);
    void advanceGracefulStop();
    bool isDriveDeEnergized() const;
    void finishGracefulStop();

    bool isCommandObject(uint16_t index) const;
    bool isFeedbackObject(uint16_t index) const;

    ds402::DriveController drive_;
    BootActionConfig boot_actions_;
    config::PdoMap pdo_map_;

    enum class StopPhase { None, DisableVoltage, Done };

    CommandImage command_;
    ds402::Feedback feedback_;
    bool cyclic_active_{false};
    bool csp_track_actual_{false};
    bool rpdo_seen_{false};
    uint64_t sync_count_{0};

    StopPhase stop_phase_{StopPhase::None};
    std::chrono::steady_clock::time_point stop_phase_deadline_{};
    std::chrono::steady_clock::time_point stop_log_due_{};
    std::function<void()> on_stopped_{};
};

}  // namespace stablecops::lely
