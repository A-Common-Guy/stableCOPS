#pragma once

#include <cstdint>
#include <string>

// Human-readable decoding of the drive's error/diagnostic values, per the
// EYou RP-series CANopen manual (V1.01). The manual reports faults through
// three related values that all share the same code space:
//   * 0x603F  - CiA402 "fieldbus fault code" (streamed in the TPDO)
//   * EMCY    - emergency error code (byte 0..1) + error register (byte 2)
//   * 0x1001  - error register bitfield (fault classes currently active)
//   * 0x1003  - pre-defined error field (history of the above)
// These helpers turn the raw numbers into text for logs and the commissioning
// UI so an operator sees "Motor Over-Temperature [Drive Fault]" instead of a
// bare 0x4210.
namespace stablecops::ds402 {

// Decode the 8-bit error register (0x1001 / EMCY byte 2) into a comma-separated
// list of the fault classes whose bit is set, e.g. "temperature, current".
// Returns "none" when no bit is set.
std::string describeErrorRegister(uint8_t error_register);

// Decode a device fault code (0x603F or an EMCY emergency error code) into its
// name and category from manual Table 4-2, e.g.
// "IPM Bottom Leg Short Circuit [Drive Fault]". Returns "no fault" for 0 and a
// generic "unknown device fault" string for codes not in the table.
std::string describeDeviceFault(uint16_t fault_code);

// Decode an SDO abort code (manual Table 4-1) into a short explanation, e.g.
// "SDO protocol timeout". Returns a generic string for unlisted codes.
std::string describeAbortCode(uint32_t abort_code);

}  // namespace stablecops::ds402
