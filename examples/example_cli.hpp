#pragma once

// Helpers shared by the stableCOPS examples: every example accepts the same
// bus/node flags and waits for cyclic feedback the same way; only the
// mode-specific options and the control loop differ per example.

#include <cstdint>
#include <string>

#include "stablecops/app/MotorConfig.hpp"

namespace examples {

// The flags handled by parseCommonArg, for usage strings.
constexpr const char* kCommonUsage =
    "[--can can0] [--dcf path] [--summary path] [--master-node 127] [--node 1]";

// Consume argv[i] (and its value, advancing i) when it is one of the common
// bus/node flags. Returns false for anything else, including a common flag with
// a missing value, so the caller falls through to its usage message.
inline bool parseCommonArg(stablecops::app::MotorConfig& config, int argc, char** argv, int& i) {
    const std::string arg = argv[i];
    if (i + 1 >= argc) {
        return false;
    }
    if (arg == "--can") {
        config.can_interface = argv[++i];
    } else if (arg == "--dcf") {
        config.master_dcf_path = argv[++i];
    } else if (arg == "--summary") {
        config.summary_path = argv[++i];
    } else if (arg == "--master-node") {
        config.master_node_id = static_cast<uint8_t>(std::stoi(argv[++i]));
    } else if (arg == "--node") {
        config.node_id = static_cast<uint8_t>(std::stoi(argv[++i]));
    } else {
        return false;
    }
    return true;
}

}  // namespace examples
