#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "stablecops/app/CanopenApplication.hpp"
#include "stablecops/app/MotorConfig.hpp"
#include "stablecops/app/RealtimeScheduling.hpp"
#include "stablecops/ds402/ObjectAccess.hpp"
#include "stablecops/ds402/State.hpp"
#include "ToolCli.hpp"

namespace {

using stablecops::tools::parseNodeList;
using stablecops::tools::parseOperationMode;

// Parse a raw SDO-write spec "index:sub:type=value", e.g. "0x605C:0:i16=0".
// index/sub/value accept 0x-prefixed hex or decimal; type is one of
// u8,u16,u32,i8,i16,i32.
std::optional<stablecops::ds402::ObjectWrite> parseObjectWrite(const std::string& spec) {
    using stablecops::ds402::ObjectWidth;
    const auto eq = spec.find('=');
    const auto colon1 = spec.find(':');
    if (eq == std::string::npos || colon1 == std::string::npos || colon1 > eq) {
        return std::nullopt;
    }
    const auto colon2 = spec.find(':', colon1 + 1);
    if (colon2 == std::string::npos || colon2 > eq) {
        return std::nullopt;
    }
    const std::string index_str = spec.substr(0, colon1);
    const std::string sub_str = spec.substr(colon1 + 1, colon2 - colon1 - 1);
    const std::string type_str = spec.substr(colon2 + 1, eq - colon2 - 1);
    const std::string value_str = spec.substr(eq + 1);

    stablecops::ds402::ObjectWrite write;
    try {
        write.index = static_cast<uint16_t>(std::stoul(index_str, nullptr, 0));
        write.subindex = static_cast<uint8_t>(std::stoul(sub_str, nullptr, 0));
        write.value = static_cast<int64_t>(std::stoll(value_str, nullptr, 0));
    } catch (...) {
        return std::nullopt;
    }
    if (type_str == "u8") write.width = ObjectWidth::U8;
    else if (type_str == "u16") write.width = ObjectWidth::U16;
    else if (type_str == "u32") write.width = ObjectWidth::U32;
    else if (type_str == "i8") write.width = ObjectWidth::I8;
    else if (type_str == "i16") write.width = ObjectWidth::I16;
    else if (type_str == "i32") write.width = ObjectWidth::I32;
    else return std::nullopt;
    return write;
}

int runMaster(int argc, char** argv) {
    stablecops::app::MotorConfig config;
    std::vector<uint8_t> node_ids;  // empty => use config.node_id (single node)
    bool show_stats = false;

    const auto print_usage = [] {
        std::cerr << "usage: stablecops_master [--can can0] [--dcf dcf/master.dcf] "
                     "[--summary generated/.../<name>.summary.json] "
                     "[--master-node 127] [--node 1] [--nodes 1,2,3] "
                     "[--inspect] [--monitor] [--enable] "
                     "[--mode csp|csv|cst|pp|pv|pt] "
                     "[--profile-velocity n] [--profile-accel n] [--profile-decel n] "
                     "[--torque-slope n] [--disable-mode n] "
                     "[--set index:sub:type=value] [--save] "
                     "[--hold-position] [--csp-target counts] [--csp-relative counts] "
                     "[--max-position-step counts] [--feedback-timeout ms] "
                     "[--counts-per-rev n] [--sync-period-us 1000] "
                     "[--rt] [--rt-prio 80] [--rt-cpu N] [--no-mlock] "
                     "[--stats] [--run]\n";
    };

    bool run = false;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--run") {
            run = true;
        } else if (arg == "--inspect") {
            config.inspect_on_boot = true;
        } else if (arg == "--monitor") {
            config.monitor_on_boot = true;
        } else if (arg == "--enable") {
            config.enable_on_boot = true;
        } else if (arg == "--hold-position") {
            config.enable_on_boot = true;
            config.hold_position_on_boot = true;
        } else if (arg == "--mode" && i + 1 < argc) {
            const std::string mode_name = argv[++i];
            const auto mode = parseOperationMode(mode_name);
            if (!mode) {
                std::cerr << "unknown --mode '" << mode_name
                          << "' (expected csp, csv, cst, pp, pv, or pt)\n";
                print_usage();
                return EXIT_FAILURE;
            }
            config.operation_mode = mode;
        } else if (arg == "--profile-velocity" && i + 1 < argc) {
            config.profile_velocity = static_cast<uint32_t>(std::stoul(argv[++i]));
        } else if (arg == "--profile-accel" && i + 1 < argc) {
            config.profile_acceleration = static_cast<uint32_t>(std::stoul(argv[++i]));
        } else if (arg == "--profile-decel" && i + 1 < argc) {
            config.profile_deceleration = static_cast<uint32_t>(std::stoul(argv[++i]));
        } else if (arg == "--torque-slope" && i + 1 < argc) {
            config.torque_slope = static_cast<uint32_t>(std::stoul(argv[++i]));
        } else if (arg == "--disable-mode" && i + 1 < argc) {
            config.disable_mode = static_cast<uint8_t>(std::stoul(argv[++i]));
        } else if (arg == "--set" && i + 1 < argc) {
            const std::string spec = argv[++i];
            const auto write = parseObjectWrite(spec);
            if (!write) {
                std::cerr << "invalid --set '" << spec
                          << "' (expected index:sub:type=value, e.g. 0x605C:0:i16=0)\n";
                print_usage();
                return EXIT_FAILURE;
            }
            config.object_writes.push_back(*write);
        } else if (arg == "--save") {
            config.save_params = true;
        } else if (arg == "--can" && i + 1 < argc) {
            config.can_interface = argv[++i];
        } else if (arg == "--dcf" && i + 1 < argc) {
            config.master_dcf_path = argv[++i];
        } else if (arg == "--summary" && i + 1 < argc) {
            config.summary_path = argv[++i];
        } else if (arg == "--master-node" && i + 1 < argc) {
            config.master_node_id = static_cast<uint8_t>(std::stoi(argv[++i]));
        } else if (arg == "--node" && i + 1 < argc) {
            config.node_id = static_cast<uint8_t>(std::stoi(argv[++i]));
        } else if (arg == "--nodes" && i + 1 < argc) {
            auto parsed = parseNodeList(argv[++i]);
            if (!parsed) {
                std::cerr << "invalid --nodes list (expected e.g. 1,2,3)\n";
                print_usage();
                return EXIT_FAILURE;
            }
            node_ids = *parsed;
        } else if (arg == "--csp-target" && i + 1 < argc) {
            config.enable_on_boot = true;
            config.csp_target_position = std::stoi(argv[++i]);
        } else if (arg == "--csp-relative" && i + 1 < argc) {
            config.enable_on_boot = true;
            config.csp_relative_move = std::stoi(argv[++i]);
        } else if (arg == "--max-position-step" && i + 1 < argc) {
            config.max_position_step = std::stoi(argv[++i]);
            if (config.max_position_step < 0) {
                print_usage();
                return EXIT_FAILURE;
            }
        } else if (arg == "--feedback-timeout" && i + 1 < argc) {
            const long ms = std::stol(argv[++i]);
            if (ms < 0) {
                print_usage();
                return EXIT_FAILURE;
            }
            config.feedback_timeout = std::chrono::milliseconds{ms};
        } else if (arg == "--counts-per-rev" && i + 1 < argc) {
            config.counts_per_rev = static_cast<uint32_t>(std::stoul(argv[++i]));
        } else if (arg == "--sync-period-us" && i + 1 < argc) {
            config.sync_period_us = static_cast<uint32_t>(std::stoul(argv[++i]));
        } else if (arg == "--rt") {
            config.rt.enabled = true;
        } else if (arg == "--rt-prio" && i + 1 < argc) {
            config.rt.priority = std::stoi(argv[++i]);
        } else if (arg == "--rt-cpu" && i + 1 < argc) {
            config.rt.cpu = std::stoi(argv[++i]);
        } else if (arg == "--no-mlock") {
            config.rt.lock_memory = false;
        } else if (arg == "--stats") {
            show_stats = true;
        } else {
            print_usage();
            return EXIT_FAILURE;
        }
    }

    if (node_ids.empty()) {
        node_ids.push_back(config.node_id);
    }

    // One config per node; all share the bus-level fields from `config`.
    std::vector<stablecops::app::MotorConfig> node_configs;
    for (uint8_t node_id : node_ids) {
        stablecops::app::MotorConfig node_config = config;
        node_config.node_id = node_id;
        node_configs.push_back(node_config);
    }

    std::cout << "stableCOPS CANopen master\n"
              << "CAN interface: " << config.can_interface << '\n'
              << "Master Node ID: " << static_cast<int>(config.master_node_id) << '\n'
              << "Node IDs: ";
    for (std::size_t i = 0; i < node_ids.size(); ++i) {
        std::cout << static_cast<int>(node_ids[i])
                  << (i + 1 < node_ids.size() ? "," : "");
    }
    std::cout << '\n'
              << "Inspect on boot: " << (config.inspect_on_boot ? "yes" : "no") << '\n'
              << "Monitor on boot: " << (config.monitor_on_boot ? "yes" : "no") << '\n'
              << "Enable on boot: " << (config.enable_on_boot ? "yes" : "no") << '\n'
              << "Operation mode: "
              << (config.operation_mode
                      ? stablecops::ds402::toString(*config.operation_mode)
                      : std::string("persisted (unchanged)"))
              << '\n'
              << "Hold position: " << (config.hold_position_on_boot ? "yes" : "no") << '\n'
              << "Disable mode (0x2103): "
              << (config.disable_mode ? std::to_string(static_cast<int>(*config.disable_mode))
                                      : std::string("persisted (unchanged)"))
              << '\n'
              << "Raw object writes: " << config.object_writes.size() << '\n'
              << "Save params to NVM: " << (config.save_params ? "yes" : "no") << '\n'
              << "Max position step: " << config.max_position_step << " counts\n"
              << "Feedback timeout: " << config.feedback_timeout.count() << " ms"
              << (config.feedback_timeout.count() == 0 ? " (watchdog disabled)" : "") << '\n'
              << "SYNC period (nominal): " << config.sync_period_us << " us\n"
              << "Real-time: "
              << (config.rt.enabled
                      ? ("SCHED_FIFO prio " + std::to_string(config.rt.priority) +
                         (config.rt.cpu >= 0
                              ? ", cpu " + std::to_string(config.rt.cpu)
                              : std::string(", unpinned")) +
                         (config.rt.lock_memory ? ", mlock" : ", no mlock"))
                      : std::string("off"))
              << '\n'
              << "Master DCF: " << config.master_dcf_path << '\n'
              << "PDO summary: " << config.summary_path << '\n';

    if (!run) {
        std::cout << "\nPass --run to open SocketCAN and start the Lely master.\n";
        return EXIT_SUCCESS;
    }

    stablecops::app::CanopenApplication app(node_configs);
    app.resetMaster();

    // The loop runs on this thread; tune it before the cyclic work starts.
    stablecops::app::applyRealtimeScheduling(config.rt, "stablecops-master");

    std::atomic<bool> stats_done{false};
    std::thread stats_thread;
    if (show_stats) {
        stats_thread = std::thread([&app, &stats_done, node_ids] {
            while (!stats_done.load()) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
                const auto stats = app.cyclicStats();
                std::cout << "cycle: n=" << stats.cycles << " last="
                          << stats.last_us << "us mean=" << stats.mean_us
                          << "us min=" << stats.min_us << "us max=" << stats.max_us
                          << "us jitter(max)=" << stats.max_jitter_us << "us\n";
                for (uint8_t node_id : node_ids) {
                    const auto fb = app.feedback(node_id);
                    std::cout << "  node " << static_cast<int>(node_id)
                              << " state=" << stablecops::ds402::toString(fb.state)
                              << " pos=" << fb.position << " vel=" << fb.velocity
                              << (app.feedbackLive(node_id) ? " [live]" : " [stale]")
                              << '\n';
                }
            }
        });
    }

    app.run();

    stats_done.store(true);
    if (stats_thread.joinable()) {
        stats_thread.join();
    }

    return EXIT_SUCCESS;
}

}  // namespace

int main(int argc, char** argv) {
    // Malformed numeric arguments (std::stoi/std::stoul) and boot failures
    // (bad DCF path, CAN open error) surface as a message instead of terminate.
    try {
        return runMaster(argc, argv);
    } catch (const std::exception& exception) {
        std::cerr << "stablecops_master: " << exception.what() << '\n';
        return EXIT_FAILURE;
    }
}
