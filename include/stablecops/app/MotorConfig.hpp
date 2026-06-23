#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

namespace stablecops::app {

struct MotorConfig {
    std::string can_interface{"can0"};
    std::string master_dcf_path{"dcf/master.dcf"};
    // Generated PDO layout summary; the single source of truth for which
    // objects ride in each RxPDO/TxPDO. Generated alongside master.dcf from the
    // motor profile, so both ends of the bus stay coherent.
    std::string summary_path{"generated/canopen/euservo_rp/euservo_rp.summary.json"};
    uint8_t master_node_id{127};
    uint8_t node_id{1};
    bool inspect_on_boot{false};
    bool enable_on_boot{false};
    bool hold_position_on_boot{false};
    std::optional<int32_t> csp_target_position;
    std::optional<int32_t> csp_relative_move;
    int32_t max_position_step{10000};
    std::chrono::milliseconds boot_timeout{5000};
    std::chrono::milliseconds state_transition_timeout{2000};
};

}  // namespace stablecops::app
