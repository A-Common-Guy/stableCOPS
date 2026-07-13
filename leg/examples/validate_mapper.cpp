// validate_mapper -- validate the closed-chain ankle mapper end to end.
//
// Instead of replaying pre-computed motor references, this reads a *joint*
// trajectory (foot pitch/roll, radians), runs each sample through the mapper's
// reverse map (joint -> actuator) to get the two motor angles, commands those on
// the CAN drives in Cyclic Synchronous Position, and logs the foot pitch/roll
// actually measured by the two EtherCAT encoders. If the mapper is correct, the
// measured foot orientation matches the commanded pitch/roll.
//
// Mapper: flexion::Mapper::mapAnklePosJointToActuator({x=roll, y=pitch}, side)
//   -> AnkleActuators{inner, outer} motor angles in radians (gen3-2 rotary).
//   Default wiring: inner -> top node, outer -> bottom node (use --swap-motors
//   to flip). Left ankle by default (--side).
//
// Output CSV matches validate_trajectory (so plot_validation.py / analyze_*.py
// work unchanged): the "command" motor columns are the mapper outputs, and
// pitch/roll "expected" are the commanded joint angles.
//
// SAFETY: this MOVES both joints. Keep the leg clear; Ctrl-C de-energises.
//
// Build with -DSTABLECOPS_BUILD_ECAT=ON -DSTABLECOPS_BUILD_MAPPER=ON.
//
// Usage:
//   sudo ./canup.sh
//   sudo build/examples/validate_mapper --rt --side left \
//       --ecat enp0s31f6 --can can0 --top-node 1 --bottom-node 2 \
//       --csv leg/trajectories/pitch_roll_traj.csv --out leg/output/mapper_out.csv

#include "ecat/master_runtime.hpp"
#include "ecat/pdo_handles.hpp"

#include "stablecops/app/MotorConfig.hpp"
#include "stablecops/app/MotorDrive.hpp"
#include "stablecops/app/RealtimeScheduling.hpp"
#include "stablecops/ds402/State.hpp"

#include "mapper.hpp"

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

struct Sample {
    double time = 0.0;   // seconds
    double pitch = 0.0;  // commanded foot pitch (rad)
    double roll = 0.0;   // commanded foot roll (rad)
};

void printUsage(const char* argv0) {
    std::fprintf(stderr,
                 "usage: %s --ecat <iface> --csv <pitch/roll csv> [--ecat-config <yaml>] "
                 "[--side left|right] [--swap-motors] [--can can0] [--top-node 1] "
                 "[--bottom-node 2] [--out mapper_out.csv] [--counts-per-rev 524288] "
                 "[--roll-index 0] [--pitch-index 1] [--pitch-sign 1] [--roll-sign 1] "
                 "[--ecat-bits 11] [--approach-seconds 3.0] [--settle-seconds 1.0] "
                 "[--zero-tolerance 50] [--rt] [--rt-prio 80] [--rt-cpu N] "
                 "[--master-node 127] [--dcf path] [--summary path]\n",
                 argv0);
}

int findColumn(const std::vector<std::string>& header, std::initializer_list<const char*> names) {
    for (std::size_t i = 0; i < header.size(); ++i) {
        for (const char* n : names) {
            if (header[i] == n) {
                return static_cast<int>(i);
            }
        }
    }
    return -1;
}

// Load a joint (pitch/roll) trajectory. Accepts either explicit "pitch"/"roll"
// columns or the trajectory-generator names "pitch_foot_expected"/"...". "time"
// is optional (falls back to a 1 kHz index).
bool loadCsv(const std::string& path, std::vector<Sample>& out) {
    std::ifstream file(path);
    if (!file) {
        std::fprintf(stderr, "failed to open CSV '%s'\n", path.c_str());
        return false;
    }
    std::string line;
    if (!std::getline(file, line)) {
        std::fprintf(stderr, "CSV '%s' is empty\n", path.c_str());
        return false;
    }
    std::vector<std::string> header;
    {
        std::stringstream hs(line);
        std::string cell;
        while (std::getline(hs, cell, ',')) {
            header.push_back(cell);
        }
    }
    const int i_time = findColumn(header, {"time", "t"});
    const int i_pitch = findColumn(header, {"pitch", "pitch_foot_expected", "pitch_foot_desired",
                                            "pitch_expected"});
    const int i_roll = findColumn(header, {"roll", "roll_foot_expected", "roll_foot_desired",
                                           "roll_expected"});
    if (i_pitch < 0 || i_roll < 0) {
        std::fprintf(stderr,
                     "CSV must have pitch and roll columns (pitch/roll or "
                     "pitch_foot_expected/roll_foot_expected)\n");
        return false;
    }

    std::size_t idx = 0;
    while (std::getline(file, line)) {
        if (line.empty()) {
            continue;
        }
        std::vector<double> v;
        std::stringstream ss(line);
        std::string cell;
        while (std::getline(ss, cell, ',')) {
            try {
                v.push_back(std::stod(cell));
            } catch (...) {
                v.push_back(0.0);
            }
        }
        const std::size_t need = static_cast<std::size_t>(std::max({i_time, i_pitch, i_roll})) + 1;
        if (v.size() < need) {
            continue;
        }
        Sample s;
        s.time = (i_time >= 0) ? v[i_time] : static_cast<double>(idx) * 0.001;
        s.pitch = v[i_pitch];
        s.roll = v[i_roll];
        out.push_back(s);
        ++idx;
    }
    return !out.empty();
}

int32_t radToCounts(double rad, uint32_t counts_per_rev) {
    return static_cast<int32_t>(std::llround(rad / kTwoPi * static_cast<double>(counts_per_rev)));
}

double encoderRelRad(uint32_t raw, uint32_t zero_raw, uint32_t counts_per_turn, int sign) {
    const int32_t modulus = static_cast<int32_t>(counts_per_turn);
    const int32_t half = modulus / 2;
    int32_t delta = static_cast<int32_t>(raw) - static_cast<int32_t>(zero_raw);
    delta = ((delta % modulus) + modulus + half) % modulus - half;
    return static_cast<double>(sign) * static_cast<double>(delta) /
           static_cast<double>(counts_per_turn) * kTwoPi;
}

}  // namespace

int main(int argc, char** argv) {
    std::string ecat_iface;
    std::string ecat_config;
    std::string csv_path;
    std::string out_path = "mapper_out.csv";
    std::string can_interface = "can0";
    std::string side_str = "left";
    bool swap_motors = false;
    uint8_t top_node = 1;
    uint8_t bottom_node = 2;
    uint8_t master_node = 127;
    std::string dcf_path;
    std::string summary_path;
    uint32_t counts_per_rev = 524288;
    std::size_t roll_index = 0;
    std::size_t pitch_index = 1;
    int pitch_sign = -1;
    int roll_sign = 1;
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
        } else if (arg == "--csv") {
            csv_path = next("--csv");
        } else if (arg == "--out") {
            out_path = next("--out");
        } else if (arg == "--side") {
            side_str = next("--side");
        } else if (arg == "--swap-motors") {
            swap_motors = true;
        } else if (arg == "--can") {
            can_interface = next("--can");
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
        } else if (arg == "--pitch-index") {
            pitch_index = static_cast<std::size_t>(std::stoul(next("--pitch-index")));
        } else if (arg == "--roll-index") {
            roll_index = static_cast<std::size_t>(std::stoul(next("--roll-index")));
        } else if (arg == "--pitch-sign") {
            pitch_sign = std::stoi(next("--pitch-sign"));
        } else if (arg == "--roll-sign") {
            roll_sign = std::stoi(next("--roll-sign"));
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

    if (ecat_iface.empty() || csv_path.empty()) {
        printUsage(argv[0]);
        return EXIT_FAILURE;
    }
    flexion::Side side;
    if (side_str == "left") {
        side = flexion::Side::Left;
    } else if (side_str == "right") {
        side = flexion::Side::Right;
    } else {
        std::fprintf(stderr, "--side must be left or right\n");
        return EXIT_FAILURE;
    }

    std::vector<Sample> trajectory;
    if (!loadCsv(csv_path, trajectory)) {
        return EXIT_FAILURE;
    }
    std::printf("loaded %zu pitch/roll samples from %s (side=%s)\n", trajectory.size(),
                csv_path.c_str(), side_str.c_str());

    // Map every joint sample to the two actuator (motor) angles up front, and
    // route them to the top/bottom CAN nodes. inner -> top, outer -> bottom by
    // default (--swap-motors flips).
    flexion::Mapper mapper;
    std::vector<double> top_cmd(trajectory.size());
    std::vector<double> bottom_cmd(trajectory.size());
    for (std::size_t i = 0; i < trajectory.size(); ++i) {
        flexion::AnkleJoints joints;
        joints.y = trajectory[i].pitch;  // mapper convention: y = pitch
        joints.x = trajectory[i].roll;   //                    x = roll
        const flexion::AnkleActuators act = mapper.mapAnklePosJointToActuator(joints, side);
        top_cmd[i] = swap_motors ? act.outer : act.inner;
        bottom_cmd[i] = swap_motors ? act.inner : act.outer;
    }
    std::printf("mapper: sample0 pitch=%.4f roll=%.4f -> top=%.4f rad bottom=%.4f rad\n",
                trajectory.front().pitch, trajectory.front().roll, top_cmd.front(),
                bottom_cmd.front());

    const uint32_t ecat_counts_per_turn = 1u << ecat_bits;

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

    MasterRuntime runtime(ecat_iface, ecat_config);
    if (!runtime.start()) {
        std::fprintf(stderr, "EtherCAT start failed: %s\n", runtime.lastError().c_str());
        top.stop();
        bottom.stop();
        return EXIT_FAILURE;
    }
    auto& io = runtime.io();
    const std::size_t ecat_count = io.driveCount();
    std::printf("[validate_mapper] %zu EtherCAT drive(s) on %s\n", ecat_count, ecat_iface.c_str());
    if (ecat_count <= pitch_index || ecat_count <= roll_index) {
        std::fprintf(stderr,
                     "not enough EtherCAT drives (%zu) for roll-index %zu / pitch-index %zu\n",
                     ecat_count, roll_index, pitch_index);
        top.stop();
        bottom.stop();
        return EXIT_FAILURE;
    }

    // Ramp both drives smoothly to zero counts (mechanical/NVM zero).
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

    const auto zero_deadline =
        std::chrono::steady_clock::now() +
        std::chrono::milliseconds(static_cast<long>(settle_seconds * 1000.0) + 3000);
    bool at_zero = false;
    while (std::chrono::steady_clock::now() < zero_deadline) {
        if (!top.feedbackLive() || !bottom.feedbackLive()) {
            std::fprintf(stderr, "CAN feedback went stale while settling at zero; aborting\n");
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

    // Latch the EtherCAT encoder zero at the confirmed CAN-zero pose.
    uint64_t pitch_accum = 0;
    uint64_t roll_accum = 0;
    const int zero_samples = 200;
    for (int s = 0; s < zero_samples && runtime.ok(); ++s) {
        pitch_accum += io.get(pitch_index, pdo::auxFeedback);
        roll_accum += io.get(roll_index, pdo::auxFeedback);
        runtime.sleepCycle();
    }
    const uint32_t pitch_zero = static_cast<uint32_t>(pitch_accum / zero_samples);
    const uint32_t roll_zero = static_cast<uint32_t>(roll_accum / zero_samples);
    std::printf("ecat zero latched at CAN zero: pitch_raw=%u roll_raw=%u (%u-bit)\n", pitch_zero,
                roll_zero, ecat_bits);

    // Ramp from zero to the first mapped target so a non-zero trajectory start
    // (or a mapper zero offset) does not produce a step.
    {
        const int32_t t0c = radToCounts(top_cmd.front(), counts_per_rev);
        const int32_t b0c = radToCounts(bottom_cmd.front(), counts_per_rev);
        const int pre = std::max(1, static_cast<int>(1000));  // 1 s
        for (int step = 1; step <= pre && runtime.ok(); ++step) {
            const double a = static_cast<double>(step) / pre;
            top.commandPosition(static_cast<int32_t>(std::llround(t0c * a)));
            bottom.commandPosition(static_cast<int32_t>(std::llround(b0c * a)));
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    struct LogRow {
        double time, motor1_rad, motor2_rad, motor1_meas_rad, motor2_meas_rad;
        double pitch_meas, pitch_exp, roll_meas, roll_exp;
    };
    const double kCountToRad = kTwoPi / static_cast<double>(counts_per_rev);
    std::vector<LogRow> log;
    log.reserve(trajectory.size());

    if (rt_enabled) {
        stablecops::app::RtConfig replay_rt;
        replay_rt.enabled = true;
        replay_rt.priority = std::max(1, rt_prio - 5);
        replay_rt.cpu = rt_cpu;
        replay_rt.lock_memory = true;
        stablecops::app::applyRealtimeScheduling(replay_rt, "validate-mapper");
    }

    std::printf("replaying mapped trajectory (%zu samples)%s...\n", trajectory.size(),
                rt_enabled ? " [RT]" : " [non-RT]");
    bool aborted = false;
    for (std::size_t i = 0; i < trajectory.size(); ++i) {
        if (!runtime.ok()) {
            std::fprintf(stderr, "EtherCAT runtime stopped; aborting\n");
            aborted = true;
            break;
        }
        if (!top.feedbackLive() || !bottom.feedbackLive()) {
            std::fprintf(stderr, "CAN feedback went stale during replay; aborting\n");
            aborted = true;
            break;
        }
        if (top.faulted() || bottom.faulted()) {
            std::fprintf(stderr, "CAN drive faulted during replay; aborting\n");
            aborted = true;
            break;
        }

        const auto& sample = trajectory[i];
        top.commandPosition(radToCounts(top_cmd[i], counts_per_rev));
        bottom.commandPosition(radToCounts(bottom_cmd[i], counts_per_rev));

        const uint32_t pitch_raw = io.get(pitch_index, pdo::auxFeedback);
        const uint32_t roll_raw = io.get(roll_index, pdo::auxFeedback);
        LogRow row;
        row.time = sample.time;
        row.motor1_rad = top_cmd[i];
        row.motor2_rad = bottom_cmd[i];
        row.motor1_meas_rad = static_cast<double>(top.feedback().position) * kCountToRad;
        row.motor2_meas_rad = static_cast<double>(bottom.feedback().position) * kCountToRad;
        row.pitch_meas = encoderRelRad(pitch_raw, pitch_zero, ecat_counts_per_turn, pitch_sign);
        row.pitch_exp = sample.pitch;
        row.roll_meas = encoderRelRad(roll_raw, roll_zero, ecat_counts_per_turn, roll_sign);
        row.roll_exp = sample.roll;
        log.push_back(row);

        if (i + 1 < trajectory.size()) {
            double dt = trajectory[i + 1].time - sample.time;
            if (!(dt > 0.0)) {
                dt = 0.001;
            }
            std::this_thread::sleep_for(std::chrono::duration<double>(dt));
        }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    top.stop();
    bottom.stop();

    std::ofstream out(out_path);
    if (!out) {
        std::fprintf(stderr, "failed to open output '%s'\n", out_path.c_str());
        return EXIT_FAILURE;
    }
    out << "time,motor_1_rad,motor_2_rad,motor_1_measured,motor_2_measured,"
           "pitch_measured,pitch_expected,roll_measured,roll_expected\n";
    out.setf(std::ios::fixed);
    out.precision(9);
    for (const auto& row : log) {
        out << row.time << ',' << row.motor1_rad << ',' << row.motor2_rad << ','
            << row.motor1_meas_rad << ',' << row.motor2_meas_rad << ',' << row.pitch_meas << ','
            << row.pitch_exp << ',' << row.roll_meas << ',' << row.roll_exp << '\n';
    }
    std::printf("wrote %zu rows to %s%s\n", log.size(), out_path.c_str(),
                aborted ? " (aborted early)" : "");
    return aborted ? EXIT_FAILURE : EXIT_SUCCESS;
}
