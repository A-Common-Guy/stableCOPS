#pragma once

#include <cstdint>

namespace stablecops::ds402::od {

// CiA301 diagnostic objects. The error register (0x1001) is an 8-bit RO summary
// of the active fault classes and is NOT PDO-mappable, so it can only be read
// over SDO or observed in the byte 2 of an EMCY frame. The pre-defined error
// field (0x1003) is the drive's error history array: sub 0 is the number of
// stored entries and sub 1 is the most recent (each entry packs the emergency
// error code in bits 0..15 and the error register in bits 16..23).
constexpr uint16_t error_register = 0x1001;
constexpr uint16_t predefined_error_field = 0x1003;
constexpr uint8_t predefined_error_field_count_subindex = 0x00;
constexpr uint8_t predefined_error_field_latest_subindex = 0x01;

constexpr uint16_t error_code = 0x603F;
constexpr uint16_t controlword = 0x6040;
constexpr uint16_t statusword = 0x6041;
constexpr uint16_t modes_of_operation = 0x6060;
constexpr uint16_t modes_of_operation_display = 0x6061;
constexpr uint16_t position_actual_value = 0x6064;
constexpr uint16_t velocity_actual_value = 0x606C;
constexpr uint16_t target_torque = 0x6071;
constexpr uint16_t max_torque = 0x6072;
constexpr uint16_t torque_actual_value = 0x6077;
constexpr uint16_t target_position = 0x607A;
constexpr uint16_t profile_velocity = 0x6081;
constexpr uint16_t profile_acceleration = 0x6083;
constexpr uint16_t profile_deceleration = 0x6084;
constexpr uint16_t torque_slope = 0x6087;
constexpr uint16_t target_velocity = 0x60FF;

constexpr uint16_t store_parameters = 0x1010;
constexpr uint8_t store_application_parameters_subindex = 0x03;
constexpr uint32_t store_parameters_signature = 0x65766173;  // ASCII "save"

constexpr uint16_t set_current_position_zero = 0x2262;

// Vendor "Disable Mode" (0x2103): selects what the power stage does when the
// drive leaves Operation Enabled (e.g. coast/high-impedance vs short-circuit
// dynamic braking). The concrete enum values are drive-specific.
constexpr uint16_t disable_mode = 0x2103;

constexpr uint8_t default_subindex = 0x00;

}  // namespace stablecops::ds402::od
