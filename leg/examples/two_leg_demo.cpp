// two_leg_demo -- interactive joint-space PD torque control of BOTH ankles
// over CAN only, through the stablecops::leg::AnkleLeg interface.
//
// Wiring: can0 = RIGHT leg, can1 = LEFT leg; on each bus node 1 = top drive
// (inner actuator) and node 2 = bottom drive (outer actuator). Each AnkleLeg
// runs its own internal control thread (CST, mapper-based PD, see AnkleLeg.hpp);
// this demo only sets targets/gains and streams the feedback to PlotJuggler
// over UDP (one JSON datagram, keys prefixed r_/l_).
//
// SAFETY: torque control with no artificial clamp (drive 0x6072 max torque
// still applies). Gains start at 0 (limp). Bring kp up slowly; if a joint
// DIVERGES the torque sign is wrong -- use --torque-sign -1. Keep the legs
// clear and be ready to press 's' / Ctrl-C (both leave the motors limp).
//
// Interactive stdin commands (angles in DEGREES, selector r/l/b = right/left/
// both; bare commands default to both):
//   [r|l|b] pitch <deg>   set pitch target     [r|l|b] roll <deg>  set roll target
//   [r|l|b] kp <v>        kp both axes         [r|l|b] kd <v>      kd both axes
//   [r|l|b] kpp|kpr <v>   kp pitch/roll only   [r|l|b] kdp|kdr <v> kd pitch/roll only
//   [r|l|b] z             zero targets
//   st                    print status         s                   STOP: all gains 0 (limp)
//   q                     quit (de-energises)
//
// Build with -DSTABLECOPS_BUILD_MAPPER=ON (no EtherCAT needed).
//
// Usage:
//   sudo ./canup.sh can0 can1
//   sudo build/examples/two_leg_demo --rt [--right-can can0] [--left-can can1]
//       [--udp-port 9870]
// Single-leg bring-up: --right-only / --left-only.

#include "stablecops/leg/AnkleLeg.hpp"

#include <arpa/inet.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;
constexpr double kRad2Deg = 180.0 / 3.14159265358979323846;

std::atomic<bool> g_running{true};

void onSignal(int) { g_running.store(false); }

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

void printUsage(const char* argv0) {
    std::fprintf(stderr,
                 "usage: %s [--right-can can0] [--left-can can1] [--right-only] [--left-only] "
                 "[--swap-right] [--swap-left] [--rate-hz 500] [--efficiency 1.0] "
                 "[--torque-sign 1] [--gear-ratio R (default: motor profile)] "
                 "[--counts-per-rev N (default: motor profile)] "
                 "[--motor-rated-torque Nm (default: drive 0x6076)] [--kp 0] [--kd 0] "
                 "[--udp-host 127.0.0.1] [--udp-port 9870] "
                 "[--rt] [--rt-prio 80] [--rt-cpu N] [--master-node 127]\n",
                 argv0);
}

struct Leg {
    const char* name;  // "right" / "left"
    char key;          // 'r' / 'l'
    std::unique_ptr<stablecops::leg::AnkleLeg> leg;
};

}  // namespace

int main(int argc, char** argv) {
    std::string right_can = "can0", left_can = "can1";
    std::string udp_host = "127.0.0.1";
    uint16_t udp_port = 9870;
    bool right_enabled = true, left_enabled = true;
    bool swap_right = false, swap_left = false;
    uint8_t master_node = 127;
    double rate_hz = 500.0;
    double efficiency = 1.0;
    double gear_ratio = 0.0;          // 0 = from the motor profile
    uint32_t counts_per_rev = 0;      // 0 = from the motor profile
    double motor_rated_torque = 0.0;  // 0 = read 0x6076 from the drives
    int torque_sign = 1;
    double kp0 = 0.0, kd0 = 0.0;
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
        if (a == "--right-can") right_can = next("--right-can");
        else if (a == "--left-can") left_can = next("--left-can");
        else if (a == "--right-only") left_enabled = false;
        else if (a == "--left-only") right_enabled = false;
        else if (a == "--swap-right") swap_right = true;
        else if (a == "--swap-left") swap_left = true;
        else if (a == "--master-node") master_node = static_cast<uint8_t>(std::stoi(next("--master-node")));
        else if (a == "--rate-hz") rate_hz = std::stod(next("--rate-hz"));
        else if (a == "--efficiency") efficiency = std::stod(next("--efficiency"));
        else if (a == "--gear-ratio") gear_ratio = std::stod(next("--gear-ratio"));
        else if (a == "--counts-per-rev") counts_per_rev = static_cast<uint32_t>(std::stoul(next("--counts-per-rev")));
        else if (a == "--motor-rated-torque") motor_rated_torque = std::stod(next("--motor-rated-torque"));
        else if (a == "--torque-sign") torque_sign = std::stoi(next("--torque-sign"));
        else if (a == "--kp") kp0 = std::stod(next("--kp"));
        else if (a == "--kd") kd0 = std::stod(next("--kd"));
        else if (a == "--udp-host") udp_host = next("--udp-host");
        else if (a == "--udp-port") udp_port = static_cast<uint16_t>(std::stoi(next("--udp-port")));
        else if (a == "--rt") rt_enabled = true;
        else if (a == "--rt-prio") rt_prio = std::stoi(next("--rt-prio"));
        else if (a == "--rt-cpu") rt_cpu = std::stoi(next("--rt-cpu"));
        else { printUsage(argv[0]); return EXIT_FAILURE; }
    }
    if (!right_enabled && !left_enabled) {
        std::fprintf(stderr, "--right-only and --left-only exclude each other\n");
        return EXIT_FAILURE;
    }

    auto makeConfig = [&](const std::string& can, flexion::Side side, bool swap) {
        stablecops::leg::AnkleLegConfig c;
        c.can_interface = can;
        c.side = side;
        c.swap_motors = swap;
        c.master_node = master_node;
        c.rate_hz = rate_hz;
        c.efficiency = efficiency;
        c.gear_ratio = gear_ratio;
        c.counts_per_rev = counts_per_rev;
        c.motor_rated_torque_nm = motor_rated_torque;
        c.torque_sign = torque_sign;
        c.rt.enabled = rt_enabled;
        c.rt.priority = rt_prio;
        c.rt.cpu = rt_cpu;
        return c;
    };

    std::vector<Leg> legs;
    try {
        if (right_enabled) {
            legs.push_back({"right", 'r',
                            std::make_unique<stablecops::leg::AnkleLeg>(
                                makeConfig(right_can, flexion::Side::Right, swap_right))});
        }
        if (left_enabled) {
            legs.push_back({"left", 'l',
                            std::make_unique<stablecops::leg::AnkleLeg>(
                                makeConfig(left_can, flexion::Side::Left, swap_left))});
        }
    } catch (const std::exception& e) {
        std::fprintf(stderr, "config error: %s\n", e.what());
        return EXIT_FAILURE;
    }

    struct sigaction sa{};
    sa.sa_handler = onSignal;
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);

    for (auto& l : legs) {
        std::printf("starting %s leg on %s ...\n", l.name, l.leg->config().can_interface.c_str());
        try {
            l.leg->start();
        } catch (const std::exception& e) {
            std::fprintf(stderr, "%s leg start failed: %s\n", l.name, e.what());
            for (auto& other : legs) other.leg->stop();
            return EXIT_FAILURE;
        }
        std::printf("%s leg up: CST limp, torque base %.3f Nm motor x %.3f gear x %.2f eff, "
                    "counts/rev %u\n",
                    l.name, l.leg->motorRatedTorqueNm(), l.leg->gearRatio(), efficiency,
                    l.leg->countsPerRev());
        if (kp0 != 0.0 || kd0 != 0.0) {
            l.leg->setGains({kp0, kd0, kp0, kd0});
        }
    }
    std::printf("all legs enabled (kp=%.3f kd=%.3f on both axes)\n", kp0, kd0);

    // Interactive command thread (interruptible poll, as in ankle_pd_torque).
    std::thread stdin_thread([&] {
        std::printf("\ncommands: '[r|l|b] pitch|roll <deg>' '[r|l|b] kp|kd|kpp|kpr|kdp|kdr <v>' "
                    "'[r|l|b] z' 'st' 's' 'q'\n> ");
        std::fflush(stdout);
        while (g_running.load()) {
            struct pollfd pfd{STDIN_FILENO, POLLIN, 0};
            const int r = poll(&pfd, 1, 200);  // 200 ms
            if (r <= 0 || !(pfd.revents & POLLIN)) {
                continue;  // timeout -> re-check g_running
            }
            std::string line;
            if (!std::getline(std::cin, line)) {
                break;  // EOF
            }
            std::stringstream ss(line);
            std::string tok;
            ss >> tok;
            if (tok.empty()) { std::printf("> "); std::fflush(stdout); continue; }

            // Optional leg selector; bare commands apply to both legs.
            std::vector<stablecops::leg::AnkleLeg*> sel;
            std::string cmd = tok;
            if (tok == "r" || tok == "l" || tok == "b") {
                for (auto& l : legs) {
                    if (tok == "b" || l.key == tok[0]) sel.push_back(l.leg.get());
                }
                if (sel.empty()) { std::printf("that leg is not enabled\n> "); std::fflush(stdout); continue; }
                if (!(ss >> cmd)) { std::printf("?\n> "); std::fflush(stdout); continue; }
            } else {
                for (auto& l : legs) sel.push_back(l.leg.get());
            }

            double v = 0.0;
            if (cmd == "q" || cmd == "quit") { g_running.store(false); break; }
            else if (cmd == "s" || cmd == "stop") {
                for (auto& l : legs) l.leg->limp();
                std::printf("STOP: all gains 0 (limp)\n");
            }
            else if (cmd == "z" || cmd == "zero") {
                for (auto* leg : sel) leg->setTargets({0.0, 0.0});
                std::printf("targets zeroed\n");
            }
            else if (cmd == "st" || cmd == "status") {
                for (auto& l : legs) {
                    const auto fb = l.leg->feedback();
                    const auto t = l.leg->targets();
                    const auto g = l.leg->gains();
                    std::printf(
                        "%s%s: pitch %7.2f deg (des %7.2f)  roll %7.2f deg (des %7.2f)\n"
                        "       jtau  %7.3f / %7.3f Nm  cmd %7.3f / %7.3f Nm\n"
                        "       top    %8.4f rad  %7.3f Nm motor (%7.2f Nm out)\n"
                        "       bottom %8.4f rad  %7.3f Nm motor (%7.2f Nm out)\n"
                        "       kp %.3f/%.3f kd %.4f/%.4f (pitch/roll)%s\n",
                        l.name, l.leg->running() ? "" : " [STOPPED]",
                        fb.pitch_rad * kRad2Deg, t.pitch_rad * kRad2Deg,
                        fb.roll_rad * kRad2Deg, t.roll_rad * kRad2Deg,
                        fb.joint_torque_pitch_nm, fb.joint_torque_roll_nm,
                        fb.cmd_torque_pitch_nm, fb.cmd_torque_roll_nm,
                        fb.top.position_rad, fb.top.torque_motor_nm, fb.top.torque_output_nm,
                        fb.bottom.position_rad, fb.bottom.torque_motor_nm,
                        fb.bottom.torque_output_nm,
                        g.kp_pitch, g.kp_roll, g.kd_pitch, g.kd_roll,
                        l.leg->lastError().empty() ? ""
                                                   : ("\n       error: " + l.leg->lastError()).c_str());
                }
            }
            else if ((cmd == "pitch" || cmd == "p") && (ss >> v)) {
                for (auto* leg : sel) {
                    auto t = leg->targets(); t.pitch_rad = v * kDeg2Rad; leg->setTargets(t);
                }
                std::printf("pitch target = %.2f deg\n", v);
            }
            else if ((cmd == "roll" || cmd == "r") && (ss >> v)) {
                for (auto* leg : sel) {
                    auto t = leg->targets(); t.roll_rad = v * kDeg2Rad; leg->setTargets(t);
                }
                std::printf("roll target = %.2f deg\n", v);
            }
            else if ((cmd == "kp" || cmd == "kd" || cmd == "kpp" || cmd == "kpr" ||
                      cmd == "kdp" || cmd == "kdr") && (ss >> v)) {
                for (auto* leg : sel) {
                    auto g = leg->gains();
                    if (cmd == "kp") { g.kp_pitch = v; g.kp_roll = v; }
                    else if (cmd == "kd") { g.kd_pitch = v; g.kd_roll = v; }
                    else if (cmd == "kpp") g.kp_pitch = v;
                    else if (cmd == "kpr") g.kp_roll = v;
                    else if (cmd == "kdp") g.kd_pitch = v;
                    else if (cmd == "kdr") g.kd_roll = v;
                    leg->setGains(g);
                }
                std::printf("%s = %.4f\n", cmd.c_str(), v);
            }
            else std::printf("?\n");
            std::printf("> "); std::fflush(stdout);
        }
        g_running.store(false);
    });

    UdpJson udp;
    const bool udp_ok = udp.open(udp_host, udp_port);
    if (udp_ok) std::printf("streaming telemetry to udp://%s:%u (PlotJuggler UDP-JSON)\n",
                            udp_host.c_str(), udp_port);

    // Supervision + telemetry (~200 Hz). The per-leg control loops run inside
    // AnkleLeg; if any of them dies (fault, lost feedback) everything goes limp
    // and the demo exits.
    const auto t0 = std::chrono::steady_clock::now();
    while (g_running.load()) {
        for (auto& l : legs) {
            if (!l.leg->running()) {
                std::fprintf(stderr, "\n%s leg stopped: %s\n", l.name,
                             l.leg->lastError().c_str());
                for (auto& other : legs) other.leg->limp();
                g_running.store(false);
            }
        }
        if (!g_running.load()) break;

        if (udp_ok) {
            const double t =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            char buf[2048];
            int len = std::snprintf(buf, sizeof(buf), "{\"t\":%.4f", t);
            for (auto& l : legs) {
                if (len < 0 || len >= static_cast<int>(sizeof(buf)) - 1) break;
                const auto fb = l.leg->feedback();
                const auto tg = l.leg->targets();
                len += std::snprintf(
                    buf + len, sizeof(buf) - static_cast<std::size_t>(len),
                    ",\"%c_pitch_des\":%.3f,\"%c_roll_des\":%.3f,\"%c_pitch\":%.3f,"
                    "\"%c_roll\":%.3f,\"%c_jtau_pitch\":%.4f,\"%c_jtau_roll\":%.4f,"
                    "\"%c_cmd_pitch\":%.4f,\"%c_cmd_roll\":%.4f,"
                    "\"%c_top_pos\":%.4f,\"%c_bot_pos\":%.4f,"
                    "\"%c_top_tau\":%.4f,\"%c_bot_tau\":%.4f",
                    l.key, tg.pitch_rad * kRad2Deg, l.key, tg.roll_rad * kRad2Deg,
                    l.key, fb.pitch_rad * kRad2Deg, l.key, fb.roll_rad * kRad2Deg,
                    l.key, fb.joint_torque_pitch_nm, l.key, fb.joint_torque_roll_nm,
                    l.key, fb.cmd_torque_pitch_nm, l.key, fb.cmd_torque_roll_nm,
                    l.key, fb.top.position_rad, l.key, fb.bottom.position_rad,
                    l.key, fb.top.torque_motor_nm, l.key, fb.bottom.torque_motor_nm);
            }
            if (len > 0 && len < static_cast<int>(sizeof(buf)) - 1) {
                buf[len++] = '}';
                udp.send(buf, static_cast<std::size_t>(len));
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    std::printf("\nstopping: commanding 0 torque and de-energising\n");
    for (auto& l : legs) l.leg->stop();
    if (stdin_thread.joinable()) {
        stdin_thread.join();
    }
    return EXIT_SUCCESS;
}
