#pragma once

#include <cstdint>

#include <lely/coapp/fiber_driver.hpp>

#include "stablecops/ds402/DriveController.hpp"
#include "stablecops/ds402/ObjectAccess.hpp"

namespace stablecops::lely {

class MotorDriver final : public ::lely::canopen::FiberDriver,
                          public ds402::ObjectAccess {
public:
    MotorDriver(::lely::canopen::AsyncMaster& master,
                uint8_t node_id,
                bool inspect_on_boot);

    ds402::DriveController& drive();
    const ds402::DriveController& drive() const;

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

private:
    void inspectNode() noexcept;

    ds402::DriveController drive_;
    bool inspect_on_boot_{false};
};

}  // namespace stablecops::lely
