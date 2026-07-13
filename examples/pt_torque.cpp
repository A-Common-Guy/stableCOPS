// Simple example: enable the drive in Profile Torque (PT) and stream a target
// torque. Unlike CST, the drive ramps to the target using its own torque slope
// (0x6087). On exit the drive is de-energised by the graceful stop.
//
// SAFETY: this applies motor torque. Default --torque is 0 (no torque). A
// nonzero torque with no load will accelerate the motor freely. Keep the joint
// clear and be ready to cut power.
//
// Run:
//   sudo ./canup.sh
//   build/examples/pt_torque --can can0 --node 1 --torque 0 --torque-slope 1000 --seconds 5

#include <chrono>
#include <cstdint>
#include <cstdlib>
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
    config.operation_mode = stablecops::ds402::OperationMode::ProfileTorque;
    int16_t torque = 0;  // target torque (per-mille of rated current); 0 = none
    double seconds = 5.0;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (examples::parseCommonArg(config, argc, argv, i)) {
            continue;
        }
        if (arg == "--torque" && i + 1 < argc) {
            torque = static_cast<int16_t>(std::stoi(argv[++i]));
        } else if (arg == "--torque-slope" && i + 1 < argc) {
            config.torque_slope = static_cast<uint32_t>(std::stoul(argv[++i]));
        } else if (arg == "--seconds" && i + 1 < argc) {
            seconds = std::stod(argv[++i]);
        } else {
            std::cerr << "usage: pt_torque " << examples::kCommonUsage
                      << " [--torque 0] [--torque-slope n] [--seconds 5]\n";
            return EXIT_FAILURE;
        }
    }

    std::cout << "PT: target torque " << torque << " for " << seconds << " s\n";

    stablecops::app::MotorDrive drive(config);
    drive.start();

    const auto end = std::chrono::steady_clock::now() +
                     std::chrono::milliseconds(static_cast<long>(seconds * 1000));
    while (std::chrono::steady_clock::now() < end) {
        drive.commandTorque(torque);
        const auto fb = drive.feedback();
        std::cout << "  state=" << stablecops::ds402::toString(fb.state)
                  << " pos=" << fb.position << " vel=" << fb.velocity
                  << " torq=" << fb.torque << '\n';
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    drive.commandTorque(0);
    drive.stop();
    std::cout << "done\n";
    return EXIT_SUCCESS;
}
