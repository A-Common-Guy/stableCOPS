#pragma once

#include <memory>

#include "stablecops/ds402/ObjectAccess.hpp"

namespace stablecops::lely {

class LelyObjectAccess final : public ds402::ObjectAccess {
public:
    LelyObjectAccess();
    ~LelyObjectAccess() override;

    LelyObjectAccess(const LelyObjectAccess&) = delete;
    LelyObjectAccess& operator=(const LelyObjectAccess&) = delete;

    LelyObjectAccess(LelyObjectAccess&&) noexcept;
    LelyObjectAccess& operator=(LelyObjectAccess&&) noexcept;

    uint8_t readU8(uint16_t index, uint8_t subindex) override;
    uint16_t readU16(uint16_t index, uint8_t subindex) override;
    uint32_t readU32(uint16_t index, uint8_t subindex) override;
    int32_t readI32(uint16_t index, uint8_t subindex) override;

    void writeU8(uint16_t index, uint8_t subindex, uint8_t value) override;
    void writeU16(uint16_t index, uint8_t subindex, uint16_t value) override;
    void writeU32(uint16_t index, uint8_t subindex, uint32_t value) override;
    void writeI32(uint16_t index, uint8_t subindex, int32_t value) override;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace stablecops::lely
