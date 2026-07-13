// Example: enable the drive in Cyclic Synchronous Position (CSP) and stream a
// sine wave around the position captured at enable:
//   target = start + amplitude * sin(2*pi*t / period)
// Default --amplitude is 0, which simply holds the start position. The live
// feedback is printed; on exit the drive is de-energised by the graceful stop.
//
// SAFETY: a nonzero --amplitude MOVES the motor. Keep the joint clear and be
// ready to cut power.
//
// Run:
//   sudo ./canup.sh
//   build/examples/csp_position --can can0 --node 1 --amplitude 0 --seconds 5

#include <chrono>
#include <cmath>
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
    config.operation_mode = stablecops::ds402::OperationMode::CyclicSynchronousPosition;
    int32_t amplitude = 0;  // sine amplitude in counts; 0 = hold still
    double seconds = 5.0;
    double period = 1.0;  // sine period in seconds

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (examples::parseCommonArg(config, argc, argv, i)) {
            continue;
        }
        if (arg == "--amplitude" && i + 1 < argc) {
            amplitude = std::stoi(argv[++i]);
        } else if (arg == "--seconds" && i + 1 < argc) {
            seconds = std::stod(argv[++i]);
        } else if (arg == "--period" && i + 1 < argc) {
            period = std::stod(argv[++i]);
        } else {
            std::cerr << "usage: csp_position " << examples::kCommonUsage
                      << " [--amplitude 0] [--seconds 5] [--period 1]\n";
            return EXIT_FAILURE;
        }
    }
    if (period <= 0.0) {
        std::cerr << "--period must be positive\n";
        return EXIT_FAILURE;
    }

    std::cout << "CSP: stream (start + " << amplitude << " * sin(2*pi*t / " << period << " s)) for "
              << seconds << " s\n";

    stablecops::app::MotorDrive drive(config);
    drive.start();

    // Wait for the first cyclic feedback so we can read the position at enable.
    if (!drive.waitUntilLive(std::chrono::seconds(5))) {
        std::cerr << "no cyclic feedback received; check SYNC and PDO configuration\n";
        drive.stop();
        return EXIT_FAILURE;
    }
    const int32_t center = drive.feedback().position;
    const auto start = std::chrono::steady_clock::now();

    const auto end = std::chrono::steady_clock::now() +
                     std::chrono::milliseconds(static_cast<long>(seconds * 1000));
    while (std::chrono::steady_clock::now() < end) {
        auto ms_since_start = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start
        );
        int32_t target = center + static_cast<int32_t>((amplitude * sin(2 * M_PI * ms_since_start.count() / (period * 1000.0))));
        drive.commandPosition(target);
        const auto fb = drive.feedback();
        std::cout << "  state=" << stablecops::ds402::toString(fb.state)
                  << " pos=" << fb.position << " (" << drive.positionDegrees()
                  << " deg) target=" << target << " vel=" << fb.velocity << '\n';
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    drive.stop();
    std::cout << "done\n";
    return EXIT_SUCCESS;
}
