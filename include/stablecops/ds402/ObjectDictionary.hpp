#pragma once

#include <cstdint>

namespace stablecops::ds402::od {

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
constexpr uint16_t target_velocity = 0x60FF;

constexpr uint16_t mit_parameter_0 = 0x2130;
constexpr uint16_t mit_parameter_1 = 0x2131;
constexpr uint16_t mit_parameter_2 = 0x2132;
constexpr uint16_t mit_parameter_3 = 0x2133;

constexpr uint16_t set_current_position_zero = 0x2262;

constexpr uint8_t default_subindex = 0x00;

}  // namespace stablecops::ds402::od
