// Simple example: enable the drive in Profile Position (PP) and command a single
// relative move of --offset counts. The drive runs its own trajectory using the
// configured profile velocity/accel/decel; this just triggers the DS402
// new-setpoint handshake and prints the live feedback. On exit the drive is
// de-energised by the graceful stop.
//
// SAFETY: this moves the motor by --offset counts. Default --offset is 0 (no
// move). A move needs a nonzero profile velocity (--profile-velocity, or the
// drive's persisted value). Keep the joint clear and be ready to cut power.
//
// Run:
//   sudo ./canup.sh
//   build/examples/pp_move --can can0 --node 1 --offset 0
//       --profile-velocity 50000 --seconds 5

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
    config.operation_mode = stablecops::ds402::OperationMode::ProfilePosition;
    int32_t offset = 0;   // relative move in counts; 0 = no move
    double seconds = 5.0;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (examples::parseCommonArg(config, argc, argv, i)) {
            continue;
        }
        if (arg == "--offset" && i + 1 < argc) {
            offset = std::stoi(argv[++i]);
        } else if (arg == "--profile-velocity" && i + 1 < argc) {
            config.profile_velocity = static_cast<uint32_t>(std::stoul(argv[++i]));
        } else if (arg == "--profile-accel" && i + 1 < argc) {
            config.profile_acceleration = static_cast<uint32_t>(std::stoul(argv[++i]));
        } else if (arg == "--profile-decel" && i + 1 < argc) {
            config.profile_deceleration = static_cast<uint32_t>(std::stoul(argv[++i]));
        } else if (arg == "--seconds" && i + 1 < argc) {
            seconds = std::stod(argv[++i]);
        } else {
            std::cerr << "usage: pp_move " << examples::kCommonUsage
                      << " [--offset 0] [--profile-velocity n] [--profile-accel n] "
                         "[--profile-decel n] [--seconds 5]\n";
            return EXIT_FAILURE;
        }
    }

    std::cout << "PP: relative move of " << offset << " counts\n";

    stablecops::app::MotorDrive drive(config);
    drive.start();
    drive.waitUntilLive(std::chrono::seconds(5));

    // Trigger one relative profile move; the drive runs the trajectory itself.
    drive.moveToPosition(offset, /*relative=*/true);

    const auto end = std::chrono::steady_clock::now() +
                     std::chrono::milliseconds(static_cast<long>(seconds * 1000));
    while (std::chrono::steady_clock::now() < end) {
        const auto fb = drive.feedback();
        std::cout << "  state=" << stablecops::ds402::toString(fb.state)
                  << " pos=" << fb.position << " (" << drive.positionDegrees()
                  << " deg) vel=" << fb.velocity << " torq=" << fb.torque << '\n';
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    drive.stop();
    std::cout << "done\n";
    return EXIT_SUCCESS;
}
