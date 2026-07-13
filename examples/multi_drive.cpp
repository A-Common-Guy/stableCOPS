// Example: control two drives on one CAN chain from one shared bus.
//
// You construct one MotorDrive per node, both naming the same CAN interface.
// They transparently share a single hidden bus (one master, one loop thread,
// one SYNC), so the two joints are stepped coherently every cycle. start() on
// the first drive boots the whole chain. The shared cyclic cadence (jitter) is
// printed from cyclicStats(). Optionally enable real-time tuning with --rt.
//
// This holds each joint's position captured at enable (no motion by default).
//
// SAFETY: with --enable the power stage is energised. Keep the joints clear and
// be ready to cut power.
//
// Run:
//   sudo ./canup.sh
//   build/examples/multi_drive --can can0 --nodes 1,2 --seconds 10
//   build/examples/multi_drive --can can0 --nodes 1,2 --enable --rt --rt-cpu 2

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "example_cli.hpp"
#include "stablecops/app/MotorConfig.hpp"
#include "stablecops/app/MotorDrive.hpp"
#include "stablecops/ds402/State.hpp"

namespace {

std::vector<uint8_t> parseNodes(const std::string& spec) {
    std::vector<uint8_t> nodes;
    std::stringstream stream(spec);
    std::string token;
    while (std::getline(stream, token, ',')) {
        if (!token.empty()) {
            nodes.push_back(static_cast<uint8_t>(std::stoi(token)));
        }
    }
    return nodes;
}

}  // namespace

int main(int argc, char** argv) {
    stablecops::app::MotorConfig base;
    base.monitor_on_boot = true;  // configure PDO feedback without energising.
    std::vector<uint8_t> node_ids{1, 2};
    double seconds = 10.0;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (examples::parseCommonArg(base, argc, argv, i)) {
            continue;
        }
        if (arg == "--nodes" && i + 1 < argc) {
            node_ids = parseNodes(argv[++i]);
        } else if (arg == "--enable") {
            base.enable_on_boot = true;
            base.hold_position_on_boot = true;
            base.monitor_on_boot = false;
            base.operation_mode =
                stablecops::ds402::OperationMode::CyclicSynchronousPosition;
        } else if (arg == "--seconds" && i + 1 < argc) {
            seconds = std::stod(argv[++i]);
        } else if (arg == "--rt") {
            base.rt.enabled = true;
        } else if (arg == "--rt-cpu" && i + 1 < argc) {
            base.rt.cpu = std::stoi(argv[++i]);
        } else if (arg == "--rt-prio" && i + 1 < argc) {
            base.rt.priority = std::stoi(argv[++i]);
        } else {
            std::cerr << "usage: multi_drive [--can can0] [--nodes 1,2] [--enable] "
                         "[--seconds 10] [--rt] [--rt-cpu N] [--rt-prio 80] "
                         "[--dcf path] [--summary path] [--master-node 127]\n";
            return EXIT_FAILURE;
        }
    }

    if (node_ids.empty()) {
        std::cerr << "no nodes given\n";
        return EXIT_FAILURE;
    }

    std::cout << "multi-drive on " << base.can_interface << ", nodes ";
    for (uint8_t id : node_ids) std::cout << static_cast<int>(id) << ' ';
    std::cout << "(shared bus / SYNC)\n";

    // One MotorDrive per node, all naming the same interface => one shared bus.
    std::vector<std::unique_ptr<stablecops::app::MotorDrive>> drives;
    for (uint8_t id : node_ids) {
        stablecops::app::MotorConfig config = base;
        config.node_id = id;
        drives.push_back(std::make_unique<stablecops::app::MotorDrive>(config));
    }

    // Boot the whole chain once (first start() boots the bus; rest are no-ops).
    drives.front()->start();

    const auto end = std::chrono::steady_clock::now() +
                     std::chrono::milliseconds(static_cast<long>(seconds * 1000));
    while (std::chrono::steady_clock::now() < end) {
        const auto stats = drives.front()->cyclicStats();
        std::cout << "cycle n=" << stats.cycles << " mean=" << stats.mean_us
                  << "us max=" << stats.max_us
                  << "us jitter(max)=" << stats.max_jitter_us << "us\n";
        for (auto& drive : drives) {
            const auto fb = drive->feedback();
            std::cout << "  node state=" << stablecops::ds402::toString(fb.state)
                      << " pos=" << fb.position << " (" << drive->positionDegrees()
                      << " deg) vel=" << fb.velocity
                      << (drive->feedbackLive() ? " [live]" : " [stale]") << '\n';
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // Drives de-energise as they go out of scope; the last one tears the bus
    // down (graceful stop + join).
    std::cout << "done\n";
    return EXIT_SUCCESS;
}
