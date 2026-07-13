// Example: enable the drive in CSP, hold the current position, and stream the
// cyclic feedback from the application thread. On exit (timeout or Ctrl-C) the
// drive is de-energised by the graceful stop.
//
// This exercises the full path: PDO configuration -> DS402 enable ladder ->
// cyclic command streaming -> thread-safe feedback. Keep the joint free to move
// and be ready to cut power before running on hardware.
//
// Run:
//   sudo ./canup.sh
//   build/examples/enable_and_hold --can can0 --dcf dcf/master.dcf
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
    config.enable_on_boot = true;
    config.hold_position_on_boot = true;  // hold the position captured at enable
    double seconds = 10.0;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (examples::parseCommonArg(config, argc, argv, i)) {
            continue;
        }
        if (arg == "--seconds" && i + 1 < argc) {
            seconds = std::stod(argv[++i]);
        } else {
            std::cerr << "usage: enable_and_hold " << examples::kCommonUsage
                      << " [--seconds 10]\n";
            return EXIT_FAILURE;
        }
    }

    std::cout << "enable + hold + feedback monitor\n"
              << "  CAN " << config.can_interface << ", node "
              << static_cast<int>(config.node_id) << '\n';

    stablecops::app::MotorDrive drive(config);
    drive.start();
    drive.waitUntilLive(std::chrono::seconds(5));

    const auto end = std::chrono::steady_clock::now() +
                     std::chrono::milliseconds(static_cast<long>(seconds * 1000));
    while (std::chrono::steady_clock::now() < end) {
        const auto fb = drive.feedback();
        std::cout << "  state=" << std::setw(20) << std::left
                  << stablecops::ds402::toString(fb.state)
                  << " pos=" << std::setw(10) << fb.position
                  << " (" << std::setw(8) << drive.positionDegrees() << " deg)"
                  << " vel=" << std::setw(8) << fb.velocity
                  << " torq=" << fb.torque
                  << " sw=0x" << std::hex << std::setw(4) << std::setfill('0')
                  << fb.statusword << std::dec << std::setfill(' ') << '\n';
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "stopping (graceful de-energise)\n";
    drive.stop();
    std::cout << "done\n";
    return EXIT_SUCCESS;
}
