#pragma once

#include <chrono>
#include <cstdint>
#include <string>

namespace stablecops::app {

struct MotorConfig {
    std::string can_interface{"can0"};
    std::string master_dcf_path{"dcf/master.dcf"};
    uint8_t master_node_id{127};
    uint8_t node_id{1};
    bool inspect_on_boot{false};
    std::chrono::milliseconds boot_timeout{5000};
    std::chrono::milliseconds state_transition_timeout{2000};
};

}  // namespace stablecops::app
