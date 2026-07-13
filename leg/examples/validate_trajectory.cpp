// validate_trajectory -- replay a recorded joint trajectory on the two CAN
// drives of one leg (top = motor_1, bottom = motor_2) in Cyclic Synchronous
// Position, while logging the foot pitch/roll measured by the two EtherCAT
// encoders against the expected pitch/roll from the trajectory CSV.
//
// Flow:
//   1. Bring both CAN drives up enabled in CSP, holding their current position.
//   2. Ramp both smoothly to their NVM-saved zero (position 0 counts).
//   3. Capture the two EtherCAT encoders at that pose as the pitch/roll zero.
//   4. Replay the CSV at its own 1 kHz cadence: command motor_1/motor_2 targets
//      (radians -> counts) and, every sample, record measured vs expected
//      pitch/roll into an output CSV.
//
// The EtherCAT encoders are treated as 11-bit single-turn sensors (2048 counts
// per revolution); the measured angle is (raw - zero) unwrapped into one turn
// and scaled to radians.
//
// CSV columns (input): time, motor_1_reference, motor_2_reference,
//                      pitch_foot_expected, roll_foot_expected   (radians).
//
// SAFETY: this MOVES both joints through the recorded trajectory. Keep the leg
// clear and be ready to cut power (Ctrl-C de-energises on exit).
//
// Build with -DSTABLECOPS_BUILD_ECAT=ON.
//
// Usage:
//   sudo ./canup.sh
//   sudo build/examples/validate_trajectory \
//       --ecat enp0s31f6 --can can0 --top-node 1 --bottom-node 2 \
//       --csv trajectories/validation_trajectory_rotor_1000Hz_left_short.csv \
//       --out validation_out.csv

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
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr double kTwoPi = 6.283185307179586;

struct Sample {
    double time = 0.0;         // seconds
    double motor1_rad = 0.0;   // top reference
    double motor2_rad = 0.0;   // bottom reference
    double pitch_exp = 0.0;    // expected foot pitch (rad)
    double roll_exp = 0.0;     // expected foot roll (rad)
};

void printUsage(const char* argv0) {
    std::fprintf(stderr,
                 "usage: %s --ecat <iface> --csv <path> [--ecat-config <yaml>] "
                 "[--can can0] [--top-node 1] [--bottom-node 2] "
                 "[--out validation_out.csv] [--counts-per-rev 524288] "
                 "[--roll-index 0] [--pitch-index 1] [--pitch-sign 1] "
                 "[--roll-sign 1] [--ecat-bits 11] [--approach-seconds 3.0] "
                 "[--settle-seconds 1.0] [--zero-tolerance 50] "
                 "[--rt] [--rt-prio 80] [--rt-cpu N] "
                 "[--master-node 127] [--dcf path] "
                 "[--summary path]\n",
                 argv0);
}

// Parse the trajectory CSV. Returns false on any malformed input.
bool loadCsv(const std::string& path, std::vector<Sample>& out) {
    std::ifstream file(path);
    if (!file) {
        std::fprintf(stderr, "failed to open CSV '%s'\n", path.c_str());
        return false;
    }
    std::string line;
    // Skip the header row.
    if (!std::getline(file, line)) {
        std::fprintf(stderr, "CSV '%s' is empty\n", path.c_str());
        return false;
    }
    std::size_t line_no = 1;
    while (std::getline(file, line)) {
        ++line_no;
        if (line.empty()) {
            continue;
        }
        std::stringstream stream(line);
        std::string cell;
        double values[5];
        int count = 0;
        while (count < 5 && std::getline(stream, cell, ',')) {
            try {
                values[count] = std::stod(cell);
            } catch (...) {
                std::fprintf(stderr, "CSV parse error at line %zu\n", line_no);
                return false;
            }
            ++count;
        }
        if (count < 5) {
            std::fprintf(stderr, "CSV line %zu has %d columns (expected 5)\n", line_no, count);
            return false;
        }
        Sample sample;
        sample.time = values[0];
        sample.motor1_rad = values[1];
        sample.motor2_rad = values[2];
        sample.pitch_exp = values[3];
        sample.roll_exp = values[4];
        out.push_back(sample);
    }
    return !out.empty();
}

int32_t radToCounts(double rad, uint32_t counts_per_rev) {
    return static_cast<int32_t>(std::llround(rad / kTwoPi * static_cast<double>(counts_per_rev)));
}

// Convert a raw 11-bit (or N-bit) single-turn encoder reading, relative to the
// captured zero, into radians unwrapped into a single turn around the zero.
double encoderRelRad(uint32_t raw, uint32_t zero_raw, uint32_t counts_per_turn, int sign) {
    const int32_t half = static_cast<int32_t>(counts_per_turn / 2);
    int32_t delta = static_cast<int32_t>(raw) - static_cast<int32_t>(zero_raw);
    // Wrap into [-half, half).
    const int32_t modulus = static_cast<int32_t>(counts_per_turn);
    delta = ((delta % modulus) + modulus + half) % modulus - half;
    return static_cast<double>(sign) * static_cast<double>(delta) / static_cast<double>(counts_per_turn) *
           kTwoPi;
}

}  // namespace

int main(int argc, char** argv) {
    std::string ecat_iface;
    std::string ecat_config;  // empty => scan mode
    std::string csv_path;
    std::string out_path = "validation_out.csv";
    std::string can_interface = "can0";
    uint8_t top_node = 1;
    uint8_t bottom_node = 2;
    uint8_t master_node = 127;
    std::string dcf_path;
    std::string summary_path;
    uint32_t counts_per_rev = 524288;
    // EtherCAT encoder order on the bus: first drive = roll, second = pitch.
    std::size_t roll_index = 0;
    std::size_t pitch_index = 1;
    // Pitch encoder reads opposite the CSV convention; invert by default.
    int pitch_sign = 1;
    int roll_sign = 1;
    uint32_t ecat_bits = 11;
    double approach_seconds = 3.0;
    double settle_seconds = 1.0;
    // Both CAN drives must be within this many counts of 0 before the EtherCAT
    // encoder zero is latched.
    int32_t zero_tolerance = 50;
    // Real-time scheduling for the CAN bus loop and this replay thread. Strongly
    // recommended: without it, CSP setpoints arrive with scheduling jitter that
    // is not phase-locked to the SYNC, producing jerky motion.
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

    std::vector<Sample> trajectory;
    if (!loadCsv(csv_path, trajectory)) {
        return EXIT_FAILURE;
    }
    std::printf("loaded %zu trajectory samples from %s\n", trajectory.size(), csv_path.c_str());

    const uint32_t ecat_counts_per_turn = 1u << ecat_bits;

    // Configure the two CAN drives: enabled in CSP on boot, holding their
    // current position so enabling never produces a step.
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

    // Wait for both drives to report live feedback and reach operation enabled.
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

    // Bring up the EtherCAT master first so the encoders are already streaming
    // when the CAN drives reach zero (the encoder zero is latched at that exact
    // moment, below).
    MasterRuntime runtime(ecat_iface, ecat_config);
    if (!runtime.start()) {
        std::fprintf(stderr, "EtherCAT start failed: %s\n", runtime.lastError().c_str());
        top.stop();
        bottom.stop();
        return EXIT_FAILURE;
    }
    auto& io = runtime.io();
    const std::size_t ecat_count = io.driveCount();
    std::printf("[validate_trajectory] %zu EtherCAT drive(s) on %s\n", ecat_count,
                ecat_iface.c_str());
    if (ecat_count <= pitch_index || ecat_count <= roll_index) {
        std::fprintf(stderr,
                     "not enough EtherCAT drives (%zu) for pitch-index %zu / roll-index %zu\n",
                     ecat_count, pitch_index, roll_index);
        top.stop();
        bottom.stop();
        return EXIT_FAILURE;
    }

    // Ramp both drives smoothly from their current position to zero counts.
    const int32_t top_start = top.feedback().position;
    const int32_t bottom_start = bottom.feedback().position;
    const int ramp_steps = std::max(1, static_cast<int>(approach_seconds * 1000.0));
    std::printf("ramping to zero: top from %d, bottom from %d (%d ms)\n", top_start, bottom_start,
                ramp_steps);
    for (int step = 1; step <= ramp_steps; ++step) {
        const double alpha = static_cast<double>(step) / static_cast<double>(ramp_steps);
        const int32_t top_target = static_cast<int32_t>(std::llround(top_start * (1.0 - alpha)));
        const int32_t bottom_target =
            static_cast<int32_t>(std::llround(bottom_start * (1.0 - alpha)));
        top.commandPosition(top_target);
        bottom.commandPosition(bottom_target);
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

    // Wait until BOTH drives have actually settled within tolerance of zero
    // before latching the encoder zero, so the datum is taken at the exact pose
    // where the CAN actuators are at their mechanical zero (not after a blind
    // delay). Falls through at a bounded timeout so a small steady-state
    // following error cannot hang the run.
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
    // Brief dwell to damp any residual motion right at zero before sampling.
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    std::printf("at zero (%s): top pos=%d bottom pos=%d\n",
                at_zero ? "within tolerance" : "TIMEOUT, using best effort",
                top.feedback().position, bottom.feedback().position);

    // Latch the EtherCAT encoder zero now, at the confirmed CAN-zero pose.
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
    std::printf("ecat zero latched at CAN zero: pitch_raw=%u roll_raw=%u (%u-bit, %u cnt/turn)\n",
                pitch_zero, roll_zero, ecat_bits, ecat_counts_per_turn);

    // Replay the trajectory at its recorded cadence, logging measured vs
    // expected pitch/roll.
    struct LogRow {
        double time;
        double motor1_rad;       // commanded (top)
        double motor2_rad;       // commanded (bottom)
        double motor1_meas_rad;  // measured encoder (top)
        double motor2_meas_rad;  // measured encoder (bottom)
        double pitch_meas;
        double pitch_exp;
        double roll_meas;
        double roll_exp;
    };
    const double kCountToRad = kTwoPi / static_cast<double>(counts_per_rev);
    std::vector<LogRow> log;
    log.reserve(trajectory.size());

    // Optionally raise this replay thread to real-time to reduce delivery
    // jitter. IMPORTANT: keep it strictly BELOW the CAN bus loop priority so the
    // SYNC/feedback thread can never be starved -- otherwise the feedback
    // watchdog trips and de-energises the drives mid-run. Best-effort: degrades
    // to normal priority with a warning.
    if (rt_enabled) {
        stablecops::app::RtConfig replay_rt;
        replay_rt.enabled = true;
        replay_rt.priority = std::max(1, rt_prio - 5);
        replay_rt.cpu = rt_cpu;
        replay_rt.lock_memory = true;
        stablecops::app::applyRealtimeScheduling(replay_rt, "validate-replay");
    }

    std::printf("replaying trajectory (%zu samples)%s...\n", trajectory.size(),
                rt_enabled ? " [RT]" : " [non-RT]");
    // Pace relative to each sample's own delta (never against an absolute
    // schedule): every sample is executed in order, and if the machine cannot
    // keep 1 kHz the whole replay simply takes longer -- it never skips samples
    // and never busy-spins (each iteration sleeps, yielding the CPU to the CAN
    // loop). The logged `time` column is the trajectory's own timestamp, so the
    // measured-vs-expected pairing stays correct even if wall-clock is stretched.
    bool aborted = false;
    for (std::size_t i = 0; i < trajectory.size(); ++i) {
        if (!runtime.ok()) {
            std::fprintf(stderr, "EtherCAT runtime stopped (Ctrl-C or bus fault); aborting\n");
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

        top.commandPosition(radToCounts(sample.motor1_rad, counts_per_rev));
        bottom.commandPosition(radToCounts(sample.motor2_rad, counts_per_rev));

        const uint32_t pitch_raw = io.get(pitch_index, pdo::auxFeedback);
        const uint32_t roll_raw = io.get(roll_index, pdo::auxFeedback);
        LogRow row;
        row.time = sample.time;
        row.motor1_rad = sample.motor1_rad;
        row.motor2_rad = sample.motor2_rad;
        row.motor1_meas_rad = static_cast<double>(top.feedback().position) * kCountToRad;
        row.motor2_meas_rad = static_cast<double>(bottom.feedback().position) * kCountToRad;
        row.pitch_meas = encoderRelRad(pitch_raw, pitch_zero, ecat_counts_per_turn, pitch_sign);
        row.pitch_exp = sample.pitch_exp;
        row.roll_meas = encoderRelRad(roll_raw, roll_zero, ecat_counts_per_turn, roll_sign);
        row.roll_exp = sample.roll_exp;
        log.push_back(row);

        // Wait this sample's own duration before advancing. Relative sleep, so we
        // always yield and never accumulate a backlog that would force skips.
        if (i + 1 < trajectory.size()) {
            double dt = trajectory[i + 1].time - sample.time;
            if (!(dt > 0.0)) {
                dt = 0.001;  // guard against non-monotonic/zero timestamps
            }
            std::this_thread::sleep_for(std::chrono::duration<double>(dt));
        }
    }

    // Hold the last commanded position briefly, then de-energise both drives.
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    top.stop();
    bottom.stop();

    // Write the results.
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
                aborted ? " (trajectory aborted early)" : "");
    return aborted ? EXIT_FAILURE : EXIT_SUCCESS;
}
