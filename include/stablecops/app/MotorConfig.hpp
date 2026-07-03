#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include "stablecops/app/RealtimeScheduling.hpp"
#include "stablecops/ds402/State.hpp"

namespace stablecops::app {

// MotorConfig describes one drive. Several drives that name the same
// can_interface share a single bus (one master, one loop thread, one SYNC), so
// the fields are split in two:
//   - Bus-level (can_interface, master_dcf_path, summary_path, master_node_id,
//     sync_period_us, rt) must match across all drives on that interface; the
//     first drive to register defines the bus and mismatching siblings are
//     rejected.
//   - Per-drive (node_id, boot behaviour, mode, profile params, targets,
//     counts_per_rev, feedback_timeout) apply only to that node.
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
    // Configure the drive's PDOs for cyclic transfer and stream SYNC without
    // enabling the power stage, so feedback can be observed with the joint safe.
    bool monitor_on_boot{false};
    // Operation mode (0x6060) to select over SDO while the node is

    // CSP/CSV/CST, so only this object changes between cyclic modes.
    std::optional<ds402::OperationMode> operation_mode;

    std::optional<uint32_t> profile_velocity;
    std::optional<uint32_t> profile_acceleration;
    std::optional<uint32_t> profile_deceleration;
    std::optional<uint32_t> torque_slope;
    std::optional<int32_t> csp_target_position;
    std::optional<int32_t> csp_relative_move;
    int32_t max_position_step{10000};
    uint32_t counts_per_rev{524288};
    std::chrono::milliseconds boot_timeout{5000};
    std::chrono::milliseconds state_transition_timeout{2000};

    std::chrono::milliseconds feedback_timeout{100};

    uint32_t sync_period_us{1000};
    // Bus-level. Opt-in real-time tuning of the bus loop thread (off by
    // default). Shared by all drives on the interface.
    RtConfig rt;
};

}  // namespace stablecops::app
