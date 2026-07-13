// Simple example: enable the drive in Cyclic Synchronous Velocity (CSV) and
// stream a constant target velocity, printing the live feedback. On exit the
// drive is de-energised by the graceful stop.
//
// SAFETY: this spins the motor. Default --velocity is 0 (the drive holds zero
// velocity, i.e. stays still). Keep the joint clear and be ready to cut power.
//
// Run:
//   sudo ./canup.sh
//   build/examples/csv_velocity --can can0 --node 1 --velocity 0 --seconds 5

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
    config.operation_mode = stablecops::ds402::OperationMode::CyclicSynchronousVelocity;
    int32_t velocity = 0;  // target velocity (drive units); 0 = hold still
    double seconds = 5.0;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (examples::parseCommonArg(config, argc, argv, i)) {
            continue;
        }
        if (arg == "--velocity" && i + 1 < argc) {
            velocity = std::stoi(argv[++i]);
        } else if (arg == "--seconds" && i + 1 < argc) {
            seconds = std::stod(argv[++i]);
        } else {
            std::cerr << "usage: csv_velocity " << examples::kCommonUsage
                      << " [--velocity 0] [--seconds 5]\n";
            return EXIT_FAILURE;
        }
    }

    std::cout << "CSV: target velocity " << velocity << " for " << seconds << " s\n";

    stablecops::app::MotorDrive drive(config);
    drive.start();

    const auto end = std::chrono::steady_clock::now() +
                     std::chrono::milliseconds(static_cast<long>(seconds * 1000));
    while (std::chrono::steady_clock::now() < end) {
        drive.commandVelocity(velocity);
        const auto fb = drive.feedback();
        std::cout << "  state=" << stablecops::ds402::toString(fb.state)
                  << " pos=" << fb.position << " vel=" << fb.velocity
                  << " torq=" << fb.torque << '\n';
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    drive.commandVelocity(0);
    drive.stop();
    std::cout << "done\n";
    return EXIT_SUCCESS;
}
