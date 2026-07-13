// Example: receive cyclic feedback from the drive over PDO, with the joint safe.
//
// This configures the drive's PDOs for cyclic synchronous transfer and streams
// SYNC, but never enables the power stage. It then reads the thread-safe
// feedback snapshot from the application thread and prints the decoded
// statusword/state/mode/position/velocity/torque, proving the TPDO receive path
// works end to end.
//
// Run:
//   sudo ./canup.sh
//   build/examples/pdo_feedback_monitor --can can0 --dcf dcf/master.dcf
//       --summary generated/canopen/euservo_rp/euservo_rp.summary.json
//       --master-node 127 --node 1 --seconds 10

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

#include "example_cli.hpp"
#include "stablecops/app/MotorConfig.hpp"
#include "stablecops/app/MotorDrive.hpp"
#include "stablecops/ds402/State.hpp"

int main(int argc, char** argv) {
    stablecops::app::MotorConfig config;
    config.monitor_on_boot = true;  // configure PDOs + SYNC, do not energise
    double seconds = 10.0;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (examples::parseCommonArg(config, argc, argv, i)) {
            continue;
        }
        if (arg == "--seconds" && i + 1 < argc) {
            seconds = std::stod(argv[++i]);
        } else {
            std::cerr << "usage: pdo_feedback_monitor " << examples::kCommonUsage
                      << " [--seconds 10]\n";
            return EXIT_FAILURE;
        }
    }

    std::cout << "PDO feedback monitor (read-only, power stage stays off)\n"
              << "  CAN " << config.can_interface << ", node "
              << static_cast<int>(config.node_id) << ", summary "
              << config.summary_path << '\n';

    stablecops::app::MotorDrive drive(config);
    drive.start();

    // Wait briefly for the first cyclic TPDO to arrive.
    if (!drive.waitUntilLive(std::chrono::seconds(5))) {
        std::cerr << "no cyclic feedback received within 5s; check SYNC and "
                     "PDO configuration\n";
        drive.stop();
        return EXIT_FAILURE;
    }
    std::cout << "cyclic feedback is live; sampling for " << seconds << "s\n";

    const auto end = std::chrono::steady_clock::now() +
                     std::chrono::milliseconds(static_cast<long>(seconds * 1000));
    while (std::chrono::steady_clock::now() < end) {
        const auto fb = drive.feedback();
        std::cout << "  state=" << std::setw(20) << std::left
                  << stablecops::ds402::toString(fb.state)
                  << " mode=" << std::setw(28) << std::left
                  << stablecops::ds402::toString(fb.mode)
                  << " pos=" << std::setw(10) << fb.position
                  << " vel=" << std::setw(8) << fb.velocity
                  << " torq=" << fb.torque
                  << " sw=0x" << std::hex << std::setw(4) << std::setfill('0')
                  << fb.statusword << std::dec << std::setfill(' ') << '\n';
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    drive.stop();
    std::cout << "done\n";
    return EXIT_SUCCESS;
}
