#pragma once

#include <cstdint>

namespace stablecops::ds402 {

class ObjectAccess {
public:
    virtual ~ObjectAccess() = default;

    virtual uint8_t readU8(uint16_t index, uint8_t subindex) = 0;
    virtual uint16_t readU16(uint16_t index, uint8_t subindex) = 0;
    virtual uint32_t readU32(uint16_t index, uint8_t subindex) = 0;
    virtual int32_t readI32(uint16_t index, uint8_t subindex) = 0;

    virtual void writeU8(uint16_t index, uint8_t subindex, uint8_t value) = 0;
    virtual void writeU16(uint16_t index, uint8_t subindex, uint16_t value) = 0;
    virtual void writeU32(uint16_t index, uint8_t subindex, uint32_t value) = 0;
    virtual void writeI32(uint16_t index, uint8_t subindex, int32_t value) = 0;
};

}  // namespace stablecops::ds402
