// capture_ecat_zero -- drive both CAN joints of one leg (top = node 1, bottom =
// node 2) to their NVM-saved mechanical zero in Cyclic Synchronous Position,
// then latch the two EtherCAT encoders at that pose and save their raw readings
// as the pitch/roll zero offsets to a file. capture_rom (and any ROM plotting)
// then subtracts these so the encoders read 0 at the mechanical zero.
//
// Because moving to zero requires enabling the drives, the joint may be stiff to
// backdrive afterwards (the drive applies dynamic braking on disable). For a
// free-spinning range-of-motion capture, power-cycle the drives after this step
// and then run capture_rom -- the saved offsets stay valid across a power cycle
// as long as the EtherCAT encoders are absolute and the mechanical zero is
// unchanged.
//
// SAFETY: this MOVES both joints to zero. Keep the leg clear; Ctrl-C
// de-energises on exit.
//
// Build with -DSTABLECOPS_BUILD_ECAT=ON.
//
// Usage:
//   sudo ./canup.sh
//   sudo build/examples/capture_ecat_zero --rt \
//       --ecat enp0s31f6 --can can0 --top-node 1 --bottom-node 2 \
//       --out ecat_zero.csv

#include "ecat/master_runtime.hpp"
#include "ecat/pdo_handles.hpp"

#include "stablecops/app/MotorConfig.hpp"
#include "stablecops/app/MotorDrive.hpp"
#include "stablecops/app/RealtimeScheduling.hpp"
#include "stablecops/ds402/State.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <thread>

namespace {

void printUsage(const char* argv0) {
    std::fprintf(stderr,
                 "usage: %s --ecat <iface> [--ecat-config <yaml>] [--can can0] "
                 "[--top-node 1] [--bottom-node 2] [--out ecat_zero.csv] "
                 "[--counts-per-rev 524288] [--roll-index 0] [--pitch-index 1] "
                 "[--ecat-bits 11] [--approach-seconds 3.0] [--settle-seconds 1.0] "
                 "[--zero-tolerance 50] [--rt] [--rt-prio 80] [--rt-cpu N] "
                 "[--master-node 127] [--dcf path] [--summary path]\n",
                 argv0);
}

}  // namespace

int main(int argc, char** argv) {
    std::string ecat_iface;
    std::string ecat_config;
    std::string can_interface = "can0";
    std::string out_path = "ecat_zero.csv";
    std::string dcf_path;
    std::string summary_path;
    uint8_t top_node = 1;
    uint8_t bottom_node = 2;
    uint8_t master_node = 127;
    uint32_t counts_per_rev = 524288;
    std::size_t roll_index = 0;
    std::size_t pitch_index = 1;
    uint32_t ecat_bits = 11;
    double approach_seconds = 3.0;
    double settle_seconds = 1.0;
    int32_t zero_tolerance = 50;
    bool rt_enabled = false;
    int rt_prio = 80;
    int rt_cpu = -1;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto next = [&](const char* name) -> const char* {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "missing value for %s\n", name);
                std::exit(EXIT_FAILURE);
            }
            return argv[++i];
        };
        if (arg == "--ecat") {
            ecat_iface = next("--ecat");
        } else if (arg == "--ecat-config") {
            ecat_config = next("--ecat-config");
        } else if (arg == "--can") {
            can_interface = next("--can");
        } else if (arg == "--out") {
            out_path = next("--out");
        } else if (arg == "--top-node") {
            top_node = static_cast<uint8_t>(std::stoi(next("--top-node")));
        } else if (arg == "--bottom-node") {
            bottom_node = static_cast<uint8_t>(std::stoi(next("--bottom-node")));
        } else if (arg == "--master-node") {
            master_node = static_cast<uint8_t>(std::stoi(next("--master-node")));
        } else if (arg == "--dcf") {
            dcf_path = next("--dcf");
        } else if (arg == "--summary") {
            summary_path = next("--summary");
        } else if (arg == "--counts-per-rev") {
            counts_per_rev = static_cast<uint32_t>(std::stoul(next("--counts-per-rev")));
        } else if (arg == "--roll-index") {
            roll_index = static_cast<std::size_t>(std::stoul(next("--roll-index")));
        } else if (arg == "--pitch-index") {
            pitch_index = static_cast<std::size_t>(std::stoul(next("--pitch-index")));
        } else if (arg == "--ecat-bits") {
            ecat_bits = static_cast<uint32_t>(std::stoul(next("--ecat-bits")));
        } else if (arg == "--approach-seconds") {
            approach_seconds = std::stod(next("--approach-seconds"));
        } else if (arg == "--settle-seconds") {
            settle_seconds = std::stod(next("--settle-seconds"));
        } else if (arg == "--zero-tolerance") {
            zero_tolerance = static_cast<int32_t>(std::stol(next("--zero-tolerance")));
        } else if (arg == "--rt") {
            rt_enabled = true;
        } else if (arg == "--rt-prio") {
            rt_prio = std::stoi(next("--rt-prio"));
        } else if (arg == "--rt-cpu") {
            rt_cpu = std::stoi(next("--rt-cpu"));
        } else {
            printUsage(argv[0]);
            return EXIT_FAILURE;
        }
    }

    if (ecat_iface.empty()) {
        printUsage(argv[0]);
        return EXIT_FAILURE;
    }

    auto makeConfig = [&](uint8_t node) {
        stablecops::app::MotorConfig config;
        config.can_interface = can_interface;
        config.node_id = node;
        config.master_node_id = master_node;
        if (!dcf_path.empty()) config.master_dcf_path = dcf_path;
        if (!summary_path.empty()) config.summary_path = summary_path;
        config.counts_per_rev = counts_per_rev;
        config.operation_mode = stablecops::ds402::OperationMode::CyclicSynchronousPosition;
        config.enable_on_boot = true;
        config.hold_position_on_boot = true;
        config.rt.enabled = rt_enabled;
        config.rt.priority = rt_prio;
        config.rt.cpu = rt_cpu;
        return config;
    };

    stablecops::app::MotorDrive top(makeConfig(top_node));
    stablecops::app::MotorDrive bottom(makeConfig(bottom_node));

    try {
        top.start();
        bottom.start();
    } catch (const std::exception& exception) {
        std::fprintf(stderr, "CAN start failed: %s\n", exception.what());
        return EXIT_FAILURE;
    }

    auto waitEnabled = [](stablecops::app::MotorDrive& drive, const char* name) -> bool {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
        while (std::chrono::steady_clock::now() < deadline) {
            if (drive.feedbackLive() && drive.enabled()) {
                return true;
            }
            if (drive.faulted()) {
                std::fprintf(stderr, "%s drive faulted (error 0x%04X)\n", name, drive.errorCode());
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        std::fprintf(stderr, "%s drive did not reach operation enabled\n", name);
        return false;
    };

    if (!waitEnabled(top, "top") || !waitEnabled(bottom, "bottom")) {
        top.stop();
        bottom.stop();
        return EXIT_FAILURE;
    }
    std::printf("both CAN drives enabled in CSP\n");

    // Bring up the EtherCAT master so the encoders are streaming before the
    // joints reach zero.
    MasterRuntime runtime(ecat_iface, ecat_config);
    if (!runtime.start()) {
        std::fprintf(stderr, "EtherCAT start failed: %s\n", runtime.lastError().c_str());
        top.stop();
        bottom.stop();
        return EXIT_FAILURE;
    }
    auto& io = runtime.io();
    const std::size_t ecat_count = io.driveCount();
    std::printf("[capture_ecat_zero] %zu EtherCAT drive(s) on %s\n", ecat_count,
                ecat_iface.c_str());
    if (ecat_count <= pitch_index || ecat_count <= roll_index) {
        std::fprintf(stderr,
                     "not enough EtherCAT drives (%zu) for roll-index %zu / pitch-index %zu\n",
                     ecat_count, roll_index, pitch_index);
        top.stop();
        bottom.stop();
        return EXIT_FAILURE;
    }

    // Ramp both drives smoothly to zero counts.
    const int32_t top_start = top.feedback().position;
    const int32_t bottom_start = bottom.feedback().position;
    const int ramp_steps = std::max(1, static_cast<int>(approach_seconds * 1000.0));
    std::printf("ramping to zero: top from %d, bottom from %d (%d ms)\n", top_start, bottom_start,
                ramp_steps);
    for (int step = 1; step <= ramp_steps; ++step) {
        const double alpha = static_cast<double>(step) / static_cast<double>(ramp_steps);
        top.commandPosition(static_cast<int32_t>(std::llround(top_start * (1.0 - alpha))));
        bottom.commandPosition(static_cast<int32_t>(std::llround(bottom_start * (1.0 - alpha))));
        if (!top.feedbackLive() || !bottom.feedbackLive()) {
            std::fprintf(stderr, "CAN feedback went stale during approach; aborting\n");
            top.stop();
            bottom.stop();
            return EXIT_FAILURE;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    top.commandPosition(0);
    bottom.commandPosition(0);

    // Wait until both drives are actually within tolerance of zero.
    const auto zero_deadline =
        std::chrono::steady_clock::now() +
        std::chrono::milliseconds(static_cast<long>(settle_seconds * 1000.0) + 3000);
    bool at_zero = false;
    while (std::chrono::steady_clock::now() < zero_deadline) {
        if (!top.feedbackLive() || !bottom.feedbackLive()) {
            std::fprintf(stderr, "CAN feedback went stale while settling; aborting\n");
            top.stop();
            bottom.stop();
            return EXIT_FAILURE;
        }
        if (std::llabs(top.feedback().position) <= zero_tolerance &&
            std::llabs(bottom.feedback().position) <= zero_tolerance) {
            at_zero = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    std::printf("at zero (%s): top pos=%d bottom pos=%d\n",
                at_zero ? "within tolerance" : "TIMEOUT, best effort", top.feedback().position,
                bottom.feedback().position);

    // Latch the EtherCAT encoder raw values as the zero offsets.
    uint64_t roll_accum = 0;
    uint64_t pitch_accum = 0;
    const int zero_samples = 200;
    for (int s = 0; s < zero_samples && runtime.ok(); ++s) {
        roll_accum += io.get(roll_index, pdo::auxFeedback);
        pitch_accum += io.get(pitch_index, pdo::auxFeedback);
        runtime.sleepCycle();
    }
    const uint32_t roll_zero = static_cast<uint32_t>(roll_accum / zero_samples);
    const uint32_t pitch_zero = static_cast<uint32_t>(pitch_accum / zero_samples);
    std::printf("ecat zero latched: roll_raw=%u pitch_raw=%u (%u-bit)\n", roll_zero, pitch_zero,
                ecat_bits);

    // De-energise the drives before saving.
    top.stop();
    bottom.stop();

    std::ofstream out(out_path);
    if (!out) {
        std::fprintf(stderr, "failed to open output '%s'\n", out_path.c_str());
        return EXIT_FAILURE;
    }
    out << "roll_index,pitch_index,ecat_bits,roll_zero_raw,pitch_zero_raw\n";
    out << roll_index << ',' << pitch_index << ',' << ecat_bits << ',' << roll_zero << ','
        << pitch_zero << '\n';
    std::printf("saved ecat zero offsets to %s\n", out_path.c_str());
    std::printf(
        "note: the joint may be stiff now (dynamic braking on disable). Power-cycle the\n"
        "      drives before capture_rom for free backdrive; the saved offsets stay valid.\n");
    return EXIT_SUCCESS;
}
