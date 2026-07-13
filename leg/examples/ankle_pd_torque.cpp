// ankle_pd_torque -- interactive joint-space PD torque control of the ankle
// through the closed-chain mapper, validated with the EtherCAT foot encoders and
// streamed to PlotJuggler over UDP.
//
// Control loop (per cycle, joint space = foot pitch/roll):
//   1. Read motor angles/velocities from the two CAN drives.
//   2. Mapper forward map -> joint pitch/roll and their velocities.
//   3. PD:  tau_joint = kp*(target - measured) - kd*measured_vel   (per axis)
//   4. Mapper torque map (joint -> actuator) -> per-motor torque [Nm].
//   5. Nm -> drive units -> CST command. 0x6071 is per-mille of the MOTOR
//      rated torque 0x6076 (= rated current x Kt, 2.4 Nm on the RP90L), NOT
//      of the 35 Nm datasheet rating -- that figure is a loss-inclusive
//      OUTPUT rating and using it as the base overshoots torque by ~1.5x.
//      units = Nm_out * 1000 / (0x6076[Nm] * gear_ratio * efficiency).
//
// Torque-scaling choices (EYou RP manual v1.01 + "RP Series Specification
// Parameters" table, investigated 2026-07):
//   - Per-mille base: READ FROM THE DRIVE (0x6076, mNm, motor side) at
//     startup, both nodes must agree; refuse to run if the read fails. The
//     manual defines 0x6071 as per-mille of rated current (0x6075), and the
//     nameplate is self-consistent: 0x6076 = 0x6075 x Kt exactly (RP90L:
//     15 Arms x 0.16 Nm/Arms = 2.4 Nm). Wrong scaling in torque mode is a
//     safety hazard, so the base is never guessed; --motor-rated-torque
//     exists only as an explicit override.
//   - Gear ratio: from the motor profile (runtime.gear_ratio in
//     config/motors/euservo_rp.yml -> summary.json -> resolveMotorConfig),
//     the same single source of truth as counts_per_rev. --gear-ratio
//     overrides; an unknown ratio is a hard error, never a guess. The
//     profile value (21.913, RP90L) comes from the vendor spec table, NOT
//     from the drive, because no OD object carries the mechanical
//     reduction: 0x6091 "Gear Ratio" is (ab)used by the vendor for position
//     scaling (reads 1:524288 = output-encoder counts) and the
//     0x2005..0x2018 nameplate has no ratio entry. The drive does IMPLY the
//     ratio via 0x2009 (max MOTOR speed, rpm) vs 0x607F (max profile
//     velocity, OUTPUT counts/s): 3000 rpm / 136.905 rpm = 21.9131 --
//     derived and cross-checked at startup with a warning on >1% mismatch,
//     but never used as the source, because 0x607F is a user-writable limit
//     that only tracks the ratio by vendor convention.
//   - Efficiency: default 1.0 (ideal, lossless). Gearbox torque loss is
//     load/speed/direction dependent, near-stall PD holds see close to the
//     ideal transmission, and 1.0 errs toward commanding LESS torque.
//     Calibration points: vendor dynamometer constant 2.4 Nm/A -> 36 Nm per
//     1000 units -> --efficiency 0.685; spec-table joint efficiency 0.82.
//   - The old --rated-torque 35 is a hard error: 35 Nm is the loss-inclusive
//     OUTPUT rating (35/15 Arms = 2.33 Nm/A), not the command base; using it
//     overshot commanded torque by ~1.5x (52.6/35).
//
// The EtherCAT encoders are read only for independent validation/telemetry.
//
// Interactive stdin commands (angles in DEGREES):
//   pitch <deg>   set pitch target        roll <deg>   set roll target
//   kp <v>        set proportional gain    kd <v>       set derivative gain
//   z             zero both targets        s            stop (kp=kd=0 -> limp)
//   q             quit (de-energises)
//
// SAFETY: torque control with no artificial clamp (drive 0x6072 max torque still
// applies). Gains start at 0 (limp). Bring kp up slowly; if the joint DIVERGES,
// the torque sign is wrong -- use a NEGATIVE kp or --torque-sign -1. Keep the leg
// clear and be ready to cut power / press 's' / Ctrl-C.
//
// Build with -DSTABLECOPS_BUILD_ECAT=ON -DSTABLECOPS_BUILD_MAPPER=ON.
//
// Usage:
//   sudo ./canup.sh
//   sudo build/examples/ankle_pd_torque --rt --side left \
//       --ecat enp0s31f6 --can can0 --top-node 1 --bottom-node 2 \
//       --zero leg/output/ecat_zero.csv --udp-port 9870

#include "ecat/master_runtime.hpp"
#include "ecat/pdo_handles.hpp"

#include "stablecops/app/MotorConfig.hpp"
#include "stablecops/app/MotorDrive.hpp"
#include "stablecops/app/RealtimeScheduling.hpp"
#include "stablecops/ds402/State.hpp"

#include "mapper.hpp"

#include <arpa/inet.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

namespace {

constexpr double kTwoPi = 6.283185307179586;
constexpr double kDeg2Rad = kTwoPi / 360.0;
constexpr double kRad2Deg = 360.0 / kTwoPi;

double encoderRelRad(uint32_t raw, uint32_t zero_raw, uint32_t counts_per_turn, int sign) {
    const int32_t modulus = static_cast<int32_t>(counts_per_turn);
    const int32_t half = modulus / 2;
    int32_t delta = static_cast<int32_t>(raw) - static_cast<int32_t>(zero_raw);
    delta = ((delta % modulus) + modulus + half) % modulus - half;
    return static_cast<double>(sign) * static_cast<double>(delta) /
           static_cast<double>(counts_per_turn) * kTwoPi;
}

// Minimal UDP JSON sender for PlotJuggler's "UDP Server" plugin (JSON parser).
class UdpJson {
  public:
    bool open(const std::string& host, uint16_t port) {
        fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (fd_ < 0) return false;
        std::memset(&addr_, 0, sizeof(addr_));
        addr_.sin_family = AF_INET;
        addr_.sin_port = htons(port);
        if (inet_pton(AF_INET, host.c_str(), &addr_.sin_addr) != 1) return false;
        return true;
    }
    void send(const char* buf, std::size_t len) const {
        if (fd_ >= 0) {
            ::sendto(fd_, buf, len, 0, reinterpret_cast<const sockaddr*>(&addr_), sizeof(addr_));
        }
    }
    ~UdpJson() {
        if (fd_ >= 0) ::close(fd_);
    }

  private:
    int fd_{-1};
    sockaddr_in addr_{};
};

bool loadZero(const std::string& path, std::size_t& roll_index, std::size_t& pitch_index,
              uint32_t& ecat_bits, uint32_t& roll_zero, uint32_t& pitch_zero) {
    std::ifstream file(path);
    if (!file) return false;
    std::string line;
    std::getline(file, line);
    if (!std::getline(file, line)) return false;
    std::stringstream ss(line);
    std::string cell;
    std::string cells[5];
    int n = 0;
    while (n < 5 && std::getline(ss, cell, ',')) cells[n++] = cell;
    if (n < 5) return false;
    try {
        roll_index = std::stoul(cells[0]);
        pitch_index = std::stoul(cells[1]);
        ecat_bits = std::stoul(cells[2]);
        roll_zero = std::stoul(cells[3]);
        pitch_zero = std::stoul(cells[4]);
    } catch (...) {
        return false;
    }
    return true;
}

void printUsage(const char* argv0) {
    std::fprintf(stderr,
                 "usage: %s --ecat <iface> [--ecat-config <yaml>] [--side left|right] "
                 "[--swap-motors] [--can can0] [--top-node 1] [--bottom-node 2] "
                 "[--motor-rated-torque Nm] [--gear-ratio R (default: motor profile)] "
                 "[--efficiency 1.0] [--torque-sign 1] "
                 "[--counts-per-rev N (default: motor profile)] "
                 "[--zero ecat_zero.csv] [--roll-index 0] [--pitch-index 1] "
                 "[--pitch-sign 1] [--roll-sign 1] [--ecat-bits 11] [--kp 0] [--kd 0] "
                 "[--rate-hz 500] [--udp-host 127.0.0.1] [--udp-port 9870] "
                 "[--rt] [--rt-prio 80] [--rt-cpu N] [--master-node 127]\n",
                 argv0);
}

}  // namespace

int main(int argc, char** argv) {
    std::string ecat_iface, ecat_config, can_interface = "can0", side_str = "left";
    std::string dcf_path, summary_path, zero_path;
    std::string udp_host = "127.0.0.1";
    uint16_t udp_port = 9870;
    bool swap_motors = false;
    uint8_t top_node = 1, bottom_node = 2, master_node = 127;
    uint32_t counts_per_rev = 0;      // 0 = from the motor profile (runtime.counts_per_rev)
    double motor_rated_torque = 0.0;  // Nm motor-side per-mille base; 0 = read 0x6076 from drive
    double gear_ratio = 0.0;          // 0 = from the motor profile (runtime.gear_ratio)
    double efficiency = 1.0;          // torque transmission efficiency (1.0 = ideal/lossless)
    int torque_sign = 1;
    std::size_t roll_index = 0, pitch_index = 1;
    int pitch_sign = 1, roll_sign = 1;
    uint32_t ecat_bits = 11;
    double kp0 = 0.0, kd0 = 0.0;
    double rate_hz = 500.0;
    bool rt_enabled = false;
    int rt_prio = 80, rt_cpu = -1;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&](const char* n) -> const char* {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "missing value for %s\n", n);
                std::exit(EXIT_FAILURE);
            }
            return argv[++i];
        };
        if (a == "--ecat") ecat_iface = next("--ecat");
        else if (a == "--ecat-config") ecat_config = next("--ecat-config");
        else if (a == "--side") side_str = next("--side");
        else if (a == "--swap-motors") swap_motors = true;
        else if (a == "--can") can_interface = next("--can");
        else if (a == "--top-node") top_node = static_cast<uint8_t>(std::stoi(next("--top-node")));
        else if (a == "--bottom-node") bottom_node = static_cast<uint8_t>(std::stoi(next("--bottom-node")));
        else if (a == "--master-node") master_node = static_cast<uint8_t>(std::stoi(next("--master-node")));
        else if (a == "--dcf") dcf_path = next("--dcf");
        else if (a == "--summary") summary_path = next("--summary");
        else if (a == "--motor-rated-torque") motor_rated_torque = std::stod(next("--motor-rated-torque"));
        else if (a == "--gear-ratio") gear_ratio = std::stod(next("--gear-ratio"));
        else if (a == "--efficiency") efficiency = std::stod(next("--efficiency"));
        else if (a == "--rated-torque") {
            std::fprintf(stderr,
                         "--rated-torque is gone: it treated the 35 Nm datasheet OUTPUT rating as\n"
                         "the 0x6071 per-mille base and overshot torque by ~1.5x. The base is the\n"
                         "MOTOR rated torque 0x6076 (read from the drive at startup); use\n"
                         "--gear-ratio/--efficiency, and --motor-rated-torque only to override\n"
                         "the drive-reported value.\n");
            return EXIT_FAILURE;
        }
        else if (a == "--torque-sign") torque_sign = std::stoi(next("--torque-sign"));
        else if (a == "--counts-per-rev") counts_per_rev = static_cast<uint32_t>(std::stoul(next("--counts-per-rev")));
        else if (a == "--zero") zero_path = next("--zero");
        else if (a == "--roll-index") roll_index = std::stoul(next("--roll-index"));
        else if (a == "--pitch-index") pitch_index = std::stoul(next("--pitch-index"));
        else if (a == "--pitch-sign") pitch_sign = std::stoi(next("--pitch-sign"));
        else if (a == "--roll-sign") roll_sign = std::stoi(next("--roll-sign"));
        else if (a == "--ecat-bits") ecat_bits = static_cast<uint32_t>(std::stoul(next("--ecat-bits")));
        else if (a == "--kp") kp0 = std::stod(next("--kp"));
        else if (a == "--kd") kd0 = std::stod(next("--kd"));
        else if (a == "--rate-hz") rate_hz = std::stod(next("--rate-hz"));
        else if (a == "--udp-host") udp_host = next("--udp-host");
        else if (a == "--udp-port") udp_port = static_cast<uint16_t>(std::stoi(next("--udp-port")));
        else if (a == "--rt") rt_enabled = true;
        else if (a == "--rt-prio") rt_prio = std::stoi(next("--rt-prio"));
        else if (a == "--rt-cpu") rt_cpu = std::stoi(next("--rt-cpu"));
        else { printUsage(argv[0]); return EXIT_FAILURE; }
    }

    if (ecat_iface.empty()) { printUsage(argv[0]); return EXIT_FAILURE; }
    if (motor_rated_torque < 0.0) { std::fprintf(stderr, "--motor-rated-torque must be > 0 Nm (omit to read 0x6076 from the drive)\n"); return EXIT_FAILURE; }
    if (efficiency <= 0.0 || efficiency > 1.0) { std::fprintf(stderr, "--efficiency must be in (0, 1]\n"); return EXIT_FAILURE; }
    const flexion::Side side = (side_str == "right") ? flexion::Side::Right : flexion::Side::Left;

    auto makeConfig = [&](uint8_t node) {
        stablecops::app::MotorConfig c;
        c.can_interface = can_interface;
        c.node_id = node;
        c.master_node_id = master_node;
        if (!dcf_path.empty()) c.master_dcf_path = dcf_path;
        if (!summary_path.empty()) c.summary_path = summary_path;
        if (counts_per_rev != 0) c.counts_per_rev = counts_per_rev;
        c.operation_mode = stablecops::ds402::OperationMode::CyclicSynchronousTorque;
        c.enable_on_boot = true;   // energised; commands 0 torque -> limp until PD acts
        c.rt.enabled = rt_enabled;
        c.rt.priority = rt_prio;
        c.rt.cpu = rt_cpu;
        return c;
    };

    stablecops::app::MotorDrive top(makeConfig(top_node));
    stablecops::app::MotorDrive bottom(makeConfig(bottom_node));

    // Actuator constants come from the motor profile unless pinned on the CLI.
    // Wrong torque scaling is a safety hazard, so an unknown gear ratio is a
    // hard error rather than a guess.
    const bool gear_from_cli = gear_ratio > 0.0;
    if (!gear_from_cli) gear_ratio = top.config().gear_ratio;
    if (gear_ratio <= 0.0) {
        std::fprintf(stderr,
                     "gear ratio unknown: set runtime.gear_ratio in the motor profile "
                     "or pass --gear-ratio\n");
        return EXIT_FAILURE;
    }
    if (counts_per_rev == 0) counts_per_rev = top.config().counts_per_rev;
    std::printf("gear ratio %.4f (%s), counts/rev %u\n", gear_ratio,
                gear_from_cli ? "--gear-ratio override" : "motor profile runtime.gear_ratio",
                counts_per_rev);
    const double kCountToRad = kTwoPi / static_cast<double>(counts_per_rev);
    try {
        top.start();
        bottom.start();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "CAN start failed: %s\n", e.what());
        return EXIT_FAILURE;
    }

    // inner/outer <-> top/bottom wiring (matches validate_mapper: inner->top).
    stablecops::app::MotorDrive* inner = swap_motors ? &bottom : &top;
    stablecops::app::MotorDrive* outer = swap_motors ? &top : &bottom;

    auto waitEnabled = [](stablecops::app::MotorDrive& d, const char* name) -> bool {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
        while (std::chrono::steady_clock::now() < deadline) {
            if (d.feedbackLive() && d.enabled()) return true;
            if (d.faulted()) { std::fprintf(stderr, "%s faulted 0x%04X\n", name, d.errorCode()); return false; }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        std::fprintf(stderr, "%s did not reach operation enabled\n", name);
        return false;
    };
    // Command zero torque immediately so enabling is limp/safe.
    top.commandTorque(0);
    bottom.commandTorque(0);
    if (!waitEnabled(top, "top") || !waitEnabled(bottom, "bottom")) {
        top.stop(); bottom.stop();
        return EXIT_FAILURE;
    }
    top.commandTorque(0);
    bottom.commandTorque(0);
    std::printf("both CAN drives enabled in CST (0 torque, limp)\n");

    // Ground the per-mille torque base in what the drive itself executes:
    // 0x6071 is per-mille of rated current, which the drive equates to
    // per-mille of MOTOR rated torque 0x6076 (mNm, = rated current x Kt).
    // RP90L: 0x6076 = 2400 mNm -> x21.913 gear = 52.6 Nm ideal output per
    // 1000 units. Refuse to run on a guess if the read fails.
    if (motor_rated_torque <= 0.0) {
        try {
            const int64_t top_mNm =
                top.readObject(0x6076, 0x00, stablecops::app::ObjectDataType::U32);
            const int64_t bottom_mNm =
                bottom.readObject(0x6076, 0x00, stablecops::app::ObjectDataType::U32);
            if (top_mNm <= 0 || top_mNm != bottom_mNm) {
                std::fprintf(stderr, "0x6076 rated torque mismatch/invalid: top=%lld bottom=%lld mNm\n",
                             static_cast<long long>(top_mNm), static_cast<long long>(bottom_mNm));
                top.stop(); bottom.stop();
                return EXIT_FAILURE;
            }
            motor_rated_torque = static_cast<double>(top_mNm) / 1000.0;
            std::printf("per-mille torque base from drive 0x6076 = %lld mNm (motor side)\n",
                        static_cast<long long>(top_mNm));
        } catch (const std::exception& e) {
            std::fprintf(stderr, "failed to read 0x6076 rated torque: %s "
                                 "(pass --motor-rated-torque to override)\n", e.what());
            top.stop(); bottom.stop();
            return EXIT_FAILURE;
        }
    }

    // Cross-check the mechanical gear ratio against what the drive implies:
    // 0x2009 is max MOTOR speed (rpm), 0x607F is max profile velocity
    // (OUTPUT counts/s); the vendor sets both to the same physical speed, so
    // their quotient reproduces the reduction ratio (RP90L: 21.9131 vs the
    // 21.9130 spec value). No OD object carries the mechanical ratio
    // directly (0x6091 holds position scaling), and 0x607F is a
    // user-writable limit -- so this is a warning-only consistency check,
    // never an override of --gear-ratio.
    try {
        const int64_t max_motor_rpm =
            top.readObject(0x2009, 0x00, stablecops::app::ObjectDataType::U32);
        const int64_t max_out_cps =
            top.readObject(0x607F, 0x00, stablecops::app::ObjectDataType::U32);
        if (max_motor_rpm > 0 && max_out_cps > 0) {
            const double out_rpm = static_cast<double>(max_out_cps) /
                                   static_cast<double>(counts_per_rev) * 60.0;
            const double derived = static_cast<double>(max_motor_rpm) / out_rpm;
            if (std::fabs(derived - gear_ratio) / gear_ratio > 0.01) {
                std::fprintf(stderr,
                             "WARNING: drive-implied gear ratio %.4f (0x2009=%lld rpm / "
                             "0x607F=%lld cnt/s) differs from --gear-ratio %.4f by >1%% -- "
                             "check the actuator model / --counts-per-rev\n",
                             derived, static_cast<long long>(max_motor_rpm),
                             static_cast<long long>(max_out_cps), gear_ratio);
            } else {
                std::printf("gear ratio %.3f consistent with drive (0x2009/0x607F -> %.4f)\n",
                            gear_ratio, derived);
            }
        }
    } catch (const std::exception& e) {
        std::fprintf(stderr, "gear-ratio cross-check skipped (SDO read failed: %s)\n", e.what());
    }

    // EtherCAT for validation only.
    MasterRuntime runtime(ecat_iface, ecat_config);
    if (!runtime.start()) {
        std::fprintf(stderr, "EtherCAT start failed: %s\n", runtime.lastError().c_str());
        top.stop(); bottom.stop();
        return EXIT_FAILURE;
    }
    auto& io = runtime.io();
    if (io.driveCount() <= pitch_index || io.driveCount() <= roll_index) {
        std::fprintf(stderr, "not enough EtherCAT drives (%zu)\n", io.driveCount());
        top.stop(); bottom.stop();
        return EXIT_FAILURE;
    }

    // EtherCAT zero: from file if given, else latch at the current (limp) pose.
    uint32_t roll_zero = 0, pitch_zero = 0;
    {
        std::size_t ri = roll_index, pi = pitch_index;
        uint32_t bits = ecat_bits;
        if (!zero_path.empty() && loadZero(zero_path, ri, pi, bits, roll_zero, pitch_zero)) {
            roll_index = ri; pitch_index = pi; ecat_bits = bits;
            std::printf("loaded ecat zero from %s (roll_raw=%u pitch_raw=%u)\n",
                        zero_path.c_str(), roll_zero, pitch_zero);
        } else {
            uint64_t ra = 0, pa = 0;
            const int n = 200;
            for (int s = 0; s < n && runtime.ok(); ++s) {
                ra += io.get(roll_index, pdo::auxFeedback);
                pa += io.get(pitch_index, pdo::auxFeedback);
                runtime.sleepCycle();
            }
            roll_zero = static_cast<uint32_t>(ra / n);
            pitch_zero = static_cast<uint32_t>(pa / n);
            std::printf("latched ecat zero at current pose (roll_raw=%u pitch_raw=%u)\n",
                        roll_zero, pitch_zero);
        }
    }
    // Resolve after any zero-file override of ecat_bits.
    const uint32_t ecat_counts_per_turn = 1u << ecat_bits;

    // Shared setpoints/gains (interactive thread -> control loop).
    std::atomic<double> pitch_des{0.0}, roll_des{0.0}, kp{kp0}, kd{kd0};
    std::atomic<bool> running{true};

    std::thread stdin_thread([&] {
        std::printf("\ncommands: 'pitch <deg>' 'roll <deg>' 'kp <v>' 'kd <v>' 'z' 's' 'q'\n> ");
        std::fflush(stdout);
        // Interruptible read: poll stdin with a timeout so this thread notices
        // running=false (e.g. after Ctrl-C stops the control loop) and exits on
        // its own -- std::getline would otherwise block forever and hang join().
        while (running.load()) {
            struct pollfd pfd{STDIN_FILENO, POLLIN, 0};
            const int r = poll(&pfd, 1, 200);  // 200 ms
            if (r <= 0 || !(pfd.revents & POLLIN)) {
                continue;  // timeout -> re-check running
            }
            std::string line;
            if (!std::getline(std::cin, line)) {
                break;  // EOF (e.g. stdin closed)
            }
            std::stringstream ss(line);
            std::string cmd; double v = 0.0;
            ss >> cmd;
            if (cmd == "q" || cmd == "quit") { running.store(false); break; }
            else if (cmd == "s" || cmd == "stop") { kp.store(0.0); kd.store(0.0); std::printf("STOP: gains 0 (limp)\n"); }
            else if (cmd == "z" || cmd == "zero") { pitch_des.store(0.0); roll_des.store(0.0); std::printf("targets zeroed\n"); }
            else if ((cmd == "pitch" || cmd == "p") && (ss >> v)) { pitch_des.store(v * kDeg2Rad); std::printf("pitch target = %.2f deg\n", v); }
            else if ((cmd == "roll" || cmd == "r") && (ss >> v)) { roll_des.store(v * kDeg2Rad); std::printf("roll target = %.2f deg\n", v); }
            else if (cmd == "kp" && (ss >> v)) { kp.store(v); std::printf("kp = %.4f\n", v); }
            else if (cmd == "kd" && (ss >> v)) { kd.store(v); std::printf("kd = %.4f\n", v); }
            else if (!cmd.empty()) std::printf("?\n");
            std::printf("> "); std::fflush(stdout);
        }
        running.store(false);
    });

    if (rt_enabled) {
        stablecops::app::RtConfig rt;
        rt.enabled = true;
        rt.priority = std::max(1, rt_prio - 5);
        rt.cpu = rt_cpu;
        rt.lock_memory = true;
        stablecops::app::applyRealtimeScheduling(rt, "ankle-pd");
    }

    UdpJson udp;
    const bool udp_ok = udp.open(udp_host, udp_port);
    if (udp_ok) std::printf("streaming telemetry to udp://%s:%u (PlotJuggler UDP-JSON)\n",
                            udp_host.c_str(), udp_port);

    flexion::Mapper mapper;
    // Nm (output) -> 0x6071 units: 1000 units = motor rated torque at the
    // motor shaft = x gear_ratio x efficiency at the output.
    const double nm_out_per_kunit = motor_rated_torque * gear_ratio * efficiency;
    const double tau_to_units = 1000.0 / nm_out_per_kunit;
    const auto period = std::chrono::duration<double>(1.0 / (rate_hz > 0 ? rate_hz : 500.0));
    const auto t0 = std::chrono::steady_clock::now();
    std::printf("control loop at %.0f Hz. torque base: %.3f Nm motor x %.3f gear x %.2f eff "
                "= %.2f Nm out per 1000 units. gains start kp=%.3f kd=%.3f\n",
                rate_hz, motor_rated_torque, gear_ratio, efficiency, nm_out_per_kunit, kp0, kd0);

    int tele_decim = 0;
    const int tele_every = std::max(1, static_cast<int>(rate_hz / 200.0));  // ~200 Hz telemetry

    while (running.load() && runtime.ok()) {
        if (!top.feedbackLive() || !bottom.feedbackLive() || top.faulted() || bottom.faulted()) {
            std::fprintf(stderr, "\nCAN feedback lost or fault; stopping\n");
            break;
        }

        // 1) motor state -> actuator (inner/outer) pos & vel [rad, rad/s]
        flexion::AnkleActuators act_pos, act_vel;
        act_pos.inner = static_cast<double>(inner->feedback().position) * kCountToRad;
        act_pos.outer = static_cast<double>(outer->feedback().position) * kCountToRad;
        act_vel.inner = static_cast<double>(inner->feedback().velocity) * kCountToRad;
        act_vel.outer = static_cast<double>(outer->feedback().velocity) * kCountToRad;

        // 2) forward map -> joint pos/vel (y=pitch, x=roll)
        const flexion::AnkleJoints jpos = mapper.mapAnklePosActuatorToJoint(act_pos, side);
        const flexion::AnkleJoints jvel = mapper.mapAnkleVelActuatorToJoint(act_pos, act_vel, side);

        // 3) joint-space PD
        const double kp_v = kp.load(), kd_v = kd.load();
        flexion::AnkleJoints tau_joint;
        tau_joint.y = kp_v * (pitch_des.load() - jpos.y) - kd_v * jvel.y;  // pitch
        tau_joint.x = kp_v * (roll_des.load() - jpos.x) - kd_v * jvel.x;   // roll

        // 4) joint torque -> actuator torque [Nm]
        const flexion::AnkleActuators tau_act =
            mapper.mapAnkleTorqueJointToActuator(act_pos, tau_joint, side);

        // 5) Nm -> drive units, command CST. Only int16 overflow guard (no clamp).
        auto toUnits = [&](double nm) -> int16_t {
            double u = torque_sign * nm * tau_to_units;
            if (u > 32767.0) u = 32767.0;
            if (u < -32768.0) u = -32768.0;
            return static_cast<int16_t>(std::lround(u));
        };
        inner->commandTorque(toUnits(tau_act.inner));
        outer->commandTorque(toUnits(tau_act.outer));

        // Validation: EtherCAT foot angles.
        const double pitch_ecat =
            encoderRelRad(io.get(pitch_index, pdo::auxFeedback), pitch_zero, ecat_counts_per_turn,
                          pitch_sign);
        const double roll_ecat =
            encoderRelRad(io.get(roll_index, pdo::auxFeedback), roll_zero, ecat_counts_per_turn,
                          roll_sign);

        if (udp_ok && (++tele_decim % tele_every == 0)) {
            const double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            char buf[512];
            const int len = std::snprintf(
                buf, sizeof(buf),
                "{\"t\":%.4f,\"pitch_des\":%.3f,\"roll_des\":%.3f,\"pitch\":%.3f,\"roll\":%.3f,"
                "\"pitch_ecat\":%.3f,\"roll_ecat\":%.3f,\"tau_pitch\":%.4f,\"tau_roll\":%.4f,"
                "\"tau_inner\":%.4f,\"tau_outer\":%.4f,\"kp\":%.3f,\"kd\":%.4f}",
                t, pitch_des.load() * kRad2Deg, roll_des.load() * kRad2Deg, jpos.y * kRad2Deg,
                jpos.x * kRad2Deg, pitch_ecat * kRad2Deg, roll_ecat * kRad2Deg, tau_joint.y,
                tau_joint.x, tau_act.inner, tau_act.outer, kp_v, kd_v);
            if (len > 0) udp.send(buf, static_cast<std::size_t>(len));
        }

        std::this_thread::sleep_for(period);
    }

    // De-energise. Signal the input thread first so its interruptible poll loop
    // exits promptly (within its 200 ms timeout) -- no "press Enter" needed.
    running.store(false);
    top.commandTorque(0);
    bottom.commandTorque(0);
    std::printf("\nstopping: commanding 0 torque and de-energising\n");
    top.stop();
    bottom.stop();
    if (stdin_thread.joinable()) {
        stdin_thread.join();
    }
    return EXIT_SUCCESS;
}
