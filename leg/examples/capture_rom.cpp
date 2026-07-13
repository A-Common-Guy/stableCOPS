// capture_rom -- range-of-motion capture. Does NOT touch the CAN drives at all:
// it only reads the two EtherCAT encoders while you move the articulation by
// hand, logging the centered roll/pitch (in radians) to a CSV until Ctrl-C.
//
// The encoder zero offsets come from a file written by capture_ecat_zero, so the
// logged angles are 0 at the mechanical zero. For free backdrive, make sure the
// CAN drives are de-energised (e.g. freshly power-cycled) before running this.
//
// Feed the resulting CSV to trajectories/plot_rom.py to draw the reached area
// and print the numerical range of motion.
//
// Build with -DSTABLECOPS_BUILD_ECAT=ON.
//
// Usage:
//   sudo build/examples/capture_rom --ecat enp0s31f6 --zero ecat_zero.csv \
//       --out rom_data.csv --rate-hz 200
//   # move the joint through its whole range, then Ctrl-C.

#include "ecat/master_runtime.hpp"
#include "ecat/pdo_handles.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr double kTwoPi = 6.283185307179586;

void printUsage(const char* argv0) {
    std::fprintf(stderr,
                 "usage: %s --ecat <iface> [--ecat-config <yaml>] [--zero ecat_zero.csv] "
                 "[--out rom_data.csv] [--rate-hz 200] [--roll-index 0] "
                 "[--pitch-index 1] [--roll-sign 1] [--pitch-sign -1] [--ecat-bits 11]\n",
                 argv0);
}

double encoderRelRad(uint32_t raw, uint32_t zero_raw, uint32_t counts_per_turn, int sign) {
    const int32_t modulus = static_cast<int32_t>(counts_per_turn);
    const int32_t half = modulus / 2;
    int32_t delta = static_cast<int32_t>(raw) - static_cast<int32_t>(zero_raw);
    delta = ((delta % modulus) + modulus + half) % modulus - half;
    return static_cast<double>(sign) * static_cast<double>(delta) /
           static_cast<double>(counts_per_turn) * kTwoPi;
}

// Load offsets written by capture_ecat_zero. Returns false if the file cannot be
// read; leaves the passed-in defaults untouched on failure.
bool loadZero(const std::string& path, std::size_t& roll_index, std::size_t& pitch_index,
              uint32_t& ecat_bits, uint32_t& roll_zero, uint32_t& pitch_zero) {
    std::ifstream file(path);
    if (!file) {
        return false;
    }
    std::string line;
    std::getline(file, line);  // header
    if (!std::getline(file, line)) {
        return false;
    }
    std::stringstream stream(line);
    std::string cell;
    std::vector<std::string> cells;
    while (std::getline(stream, cell, ',')) {
        cells.push_back(cell);
    }
    if (cells.size() < 5) {
        return false;
    }
    try {
        roll_index = static_cast<std::size_t>(std::stoul(cells[0]));
        pitch_index = static_cast<std::size_t>(std::stoul(cells[1]));
        ecat_bits = static_cast<uint32_t>(std::stoul(cells[2]));
        roll_zero = static_cast<uint32_t>(std::stoul(cells[3]));
        pitch_zero = static_cast<uint32_t>(std::stoul(cells[4]));
    } catch (...) {
        return false;
    }
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    std::string ecat_iface;
    std::string ecat_config;
    std::string zero_path = "ecat_zero.csv";
    std::string out_path = "rom_data.csv";
    double rate_hz = 200.0;
    std::size_t roll_index = 0;
    std::size_t pitch_index = 1;
    int roll_sign = 1;
    int pitch_sign = 1;
    uint32_t ecat_bits = 11;

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
        } else if (arg == "--zero") {
            zero_path = next("--zero");
        } else if (arg == "--out") {
            out_path = next("--out");
        } else if (arg == "--rate-hz") {
            rate_hz = std::stod(next("--rate-hz"));
        } else if (arg == "--roll-index") {
            roll_index = static_cast<std::size_t>(std::stoul(next("--roll-index")));
        } else if (arg == "--pitch-index") {
            pitch_index = static_cast<std::size_t>(std::stoul(next("--pitch-index")));
        } else if (arg == "--roll-sign") {
            roll_sign = std::stoi(next("--roll-sign"));
        } else if (arg == "--pitch-sign") {
            pitch_sign = std::stoi(next("--pitch-sign"));
        } else if (arg == "--ecat-bits") {
            ecat_bits = static_cast<uint32_t>(std::stoul(next("--ecat-bits")));
        } else {
            printUsage(argv[0]);
            return EXIT_FAILURE;
        }
    }

    if (ecat_iface.empty()) {
        printUsage(argv[0]);
        return EXIT_FAILURE;
    }

    uint32_t roll_zero = 0;
    uint32_t pitch_zero = 0;
    if (loadZero(zero_path, roll_index, pitch_index, ecat_bits, roll_zero, pitch_zero)) {
        std::printf("loaded zero offsets from %s: roll_raw=%u pitch_raw=%u (%u-bit, "
                    "roll-index=%zu pitch-index=%zu)\n",
                    zero_path.c_str(), roll_zero, pitch_zero, ecat_bits, roll_index, pitch_index);
    } else {
        std::fprintf(stderr,
                     "warning: could not read '%s'; using raw zero (0). Angles will not be "
                     "centered. Run capture_ecat_zero first.\n",
                     zero_path.c_str());
    }
    const uint32_t ecat_counts_per_turn = 1u << ecat_bits;

    // EtherCAT only -- the CAN drives are intentionally left untouched so the
    // articulation can be backdriven freely by hand.
    MasterRuntime runtime(ecat_iface, ecat_config);
    if (!runtime.start()) {
        std::fprintf(stderr, "EtherCAT start failed: %s\n", runtime.lastError().c_str());
        return EXIT_FAILURE;
    }
    auto& io = runtime.io();
    const std::size_t ecat_count = io.driveCount();
    std::printf("[capture_rom] %zu EtherCAT drive(s) on %s\n", ecat_count, ecat_iface.c_str());
    if (ecat_count <= pitch_index || ecat_count <= roll_index) {
        std::fprintf(stderr,
                     "not enough EtherCAT drives (%zu) for roll-index %zu / pitch-index %zu\n",
                     ecat_count, roll_index, pitch_index);
        return EXIT_FAILURE;
    }

    // Wait for the first valid PDO exchange on both encoders before recording,
    // so the leading garbage samples (raw feedback read before the bus is
    // exchanging) never enter the data and blow up the range/area analysis.
    {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (runtime.ok() && std::chrono::steady_clock::now() < deadline) {
            if (io.feedback(roll_index).valid && io.feedback(pitch_index).valid) {
                break;
            }
            runtime.sleepCycle();
        }
        // A few extra cycles of margin for the aux-encoder field to populate.
        for (int s = 0; s < 20 && runtime.ok(); ++s) {
            runtime.sleepCycle();
        }
    }

    struct Row {
        double time;
        double roll;
        double pitch;
    };
    std::vector<Row> log;
    log.reserve(200000);

    std::printf("recording range of motion at %.0f Hz -- move the joint through its full range, "
                "then Ctrl-C to stop and save.\n",
                rate_hz);
    const auto period = std::chrono::duration<double>(1.0 / (rate_hz > 0 ? rate_hz : 200.0));
    const auto t0 = std::chrono::steady_clock::now();
    double roll_min = 1e9, roll_max = -1e9, pitch_min = 1e9, pitch_max = -1e9;
    double last_print = -1.0;
    while (runtime.ok()) {
        const uint32_t roll_raw = io.get(roll_index, pdo::auxFeedback);
        const uint32_t pitch_raw = io.get(pitch_index, pdo::auxFeedback);
        Row row;
        row.time = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        row.roll = encoderRelRad(roll_raw, roll_zero, ecat_counts_per_turn, roll_sign);
        row.pitch = encoderRelRad(pitch_raw, pitch_zero, ecat_counts_per_turn, pitch_sign);
        log.push_back(row);
        roll_min = std::min(roll_min, row.roll);
        roll_max = std::max(roll_max, row.roll);
        pitch_min = std::min(pitch_min, row.pitch);
        pitch_max = std::max(pitch_max, row.pitch);

        // Live readout so the operator can see coverage while sweeping.
        if (row.time - last_print >= 0.25) {
            std::printf("\r  roll=%+6.2f deg [%.2f..%.2f]  pitch=%+6.2f deg [%.2f..%.2f]   ",
                        row.roll * 180.0 / M_PI, roll_min * 180.0 / M_PI, roll_max * 180.0 / M_PI,
                        row.pitch * 180.0 / M_PI, pitch_min * 180.0 / M_PI, pitch_max * 180.0 / M_PI);
            std::fflush(stdout);
            last_print = row.time;
        }
        std::this_thread::sleep_for(period);
    }
    std::printf("\nstopped; %zu samples captured\n", log.size());

    std::ofstream out(out_path);
    if (!out) {
        std::fprintf(stderr, "failed to open output '%s'\n", out_path.c_str());
        return EXIT_FAILURE;
    }
    out << "time,roll,pitch\n";
    out.setf(std::ios::fixed);
    out.precision(9);
    for (const auto& row : log) {
        out << row.time << ',' << row.roll << ',' << row.pitch << '\n';
    }
    std::printf("wrote %zu rows to %s\n", log.size(), out_path.c_str());
    if (!log.empty()) {
        std::printf("range: roll [%.2f, %.2f] deg (%.2f), pitch [%.2f, %.2f] deg (%.2f)\n",
                    roll_min * 180.0 / M_PI, roll_max * 180.0 / M_PI,
                    (roll_max - roll_min) * 180.0 / M_PI, pitch_min * 180.0 / M_PI,
                    pitch_max * 180.0 / M_PI, (pitch_max - pitch_min) * 180.0 / M_PI);
    }
    return EXIT_SUCCESS;
}
