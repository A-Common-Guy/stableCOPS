// zero_leg -- run the hardstop-midpoint homing routine on a leg and set the
// mechanical zero, applying the embedded 6.608 deg zero offset with the correct
// sign for the selected leg/drive.
//
// One CAN bus carries one leg (top + bottom drive). You choose which leg (for
// the offset sign) and which drive(s) to zero: top, bottom, or both. The zero is
// placed 6.608 deg off the geometric midpoint between the two hardstops:
//   Left  leg: top -, bottom +      Right leg: top +, bottom -
// (see zeroOffsetCounts below). By default the new zero is saved to
// the drive's NVM so it persists across power cycles.
//
// SAFETY: this MOVES the joint into both hardstops to find the range, then to
// the offset zero. Keep the joint clear and be ready to cut power (Ctrl-C
// de-energises on exit). When zeroing "both", the top drive is homed first, then
// the bottom -- one joint moves at a time.
//
// Run:
//   sudo ./canup.sh
//   build/examples/zero_leg --leg left  --segment both
//   build/examples/zero_leg --leg right --segment top --can can1
//   build/examples/zero_leg --leg left  --segment bottom --no-save

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "stablecops/app/MotorConfig.hpp"
#include "stablecops/app/MotorDrive.hpp"
#include "stablecops/ds402/State.hpp"

namespace {

// Leg/offset geometry is application-specific, so it lives here in the example
// rather than in the general-purpose library. The homing routine only sees the
// resulting signed HomingConfig::home_offset (in encoder counts).
enum class Leg : uint8_t { Left, Right };
enum class LegSegment : uint8_t { Top, Bottom };

// Mechanical offset of the true zero from the geometric midpoint between the two
// hardstops, in degrees of the output shaft. Identical magnitude for every
// leg/drive; only the sign differs.
constexpr double kZeroOffsetDegrees = 6.608;

// Signed zero offset in output-shaft encoder counts for the given leg/segment,
// ready to assign to HomingConfig::home_offset. Sign convention (relative to the
// encoder's positive direction):
//   Left  leg: top negative, bottom positive
//   Right leg: top positive, bottom negative
int32_t zeroOffsetCounts(Leg leg, LegSegment segment, uint32_t counts_per_rev) {
    const double magnitude = kZeroOffsetDegrees / 360.0 * static_cast<double>(counts_per_rev);
    // Negative for exactly the (Left,Top) and (Right,Bottom) combinations.
    const bool negative = (leg == Leg::Left) == (segment == LegSegment::Top);
    return static_cast<int32_t>(std::llround(negative ? -magnitude : magnitude));
}

const char* segmentName(LegSegment segment) {
    return segment == LegSegment::Top ? "top" : "bottom";
}

void printUsage(const char* argv0) {
    std::cerr << "usage: " << argv0
              << " --leg left|right [--segment top|bottom|both] [--can iface] "
                 "[--top-node N] [--bottom-node N] [--counts-per-rev N] "
                 "[--search-velocity N] [--approach-velocity N] "
                 "[--threshold-torque N] [--backoff-distance N] "
                 "[--timeout-ms N] [--no-save] [--dcf path] [--summary path] "
                 "[--master-node 127]\n";
}

// Home a single drive and block until the routine reaches Done/Failed (or the
// safety timeout expires). Returns true on success.
bool homeSegment(stablecops::app::MotorDrive& drive, Leg leg, LegSegment segment,
                 const stablecops::ds402::HomingConfig& base_config, uint32_t counts_per_rev) {
    stablecops::ds402::HomingConfig config = base_config;
    config.home_offset = zeroOffsetCounts(leg, segment, counts_per_rev);

    const double offset_deg =
        static_cast<double>(config.home_offset) / static_cast<double>(counts_per_rev) * 360.0;
    std::cout << "\n=== zeroing " << segmentName(segment) << " drive ===\n"
              << "  zero offset: " << offset_deg << " deg (" << config.home_offset << " counts)\n";

    try {
        drive.startHoming(config);
    } catch (const std::exception& exception) {
        std::cerr << "  failed to start homing: " << exception.what() << '\n';
        return false;
    }

    // The routine advances on the loop thread; poll its phase until it settles.
    const auto deadline =
        std::chrono::steady_clock::now() + config.timeout + std::chrono::seconds(10);
    auto phase = drive.homingPhase();
    while (phase != stablecops::ds402::HomingPhase::Done &&
           phase != stablecops::ds402::HomingPhase::Failed) {
        if (std::chrono::steady_clock::now() >= deadline) {
            std::cerr << "  homing did not finish before the safety timeout\n";
            return false;
        }
        if (!drive.feedbackLive()) {
            std::cerr << "  drive feedback went stale during homing\n";
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        phase = drive.homingPhase();
    }

    const auto result = drive.homingResult();
    if (phase == stablecops::ds402::HomingPhase::Failed || !result.success) {
        std::cerr << "  homing FAILED for the " << segmentName(segment) << " drive\n";
        return false;
    }

    std::cout << "  homing OK: lower=" << result.lower_limit_position
              << " upper=" << result.upper_limit_position << " travel=" << result.travel
              << " center=" << result.center_position << " (offset " << offset_deg
              << " deg applied)\n";
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    bool leg_set = false;
    Leg leg = Leg::Left;
    std::string segment_spec = "both";
    std::string can_interface;  // empty => default per leg
    uint8_t top_node = 1;
    uint8_t bottom_node = 2;
    bool save_to_nvm = true;

    stablecops::app::MotorConfig base;

    // CLI overrides for the profile-sourced homing/scaling values; unset flags
    // keep the actuator's values from the summary's runtime profile.
    std::optional<uint32_t> counts_per_rev_override;
    std::optional<int32_t> search_velocity;
    std::optional<int32_t> approach_velocity;
    std::optional<int16_t> threshold_torque;
    std::optional<int32_t> backoff_distance;
    std::optional<std::chrono::milliseconds> homing_timeout;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto next = [&](const char* name) -> std::string {
            if (i + 1 >= argc) {
                std::cerr << "missing value for " << name << '\n';
                std::exit(EXIT_FAILURE);
            }
            return argv[++i];
        };
        if (arg == "--leg") {
            const std::string value = next("--leg");
            if (value == "left") {
                leg = Leg::Left;
            } else if (value == "right") {
                leg = Leg::Right;
            } else {
                std::cerr << "--leg must be left or right\n";
                return EXIT_FAILURE;
            }
            leg_set = true;
        } else if (arg == "--segment") {
            segment_spec = next("--segment");
        } else if (arg == "--can") {
            can_interface = next("--can");
        } else if (arg == "--top-node") {
            top_node = static_cast<uint8_t>(std::stoi(next("--top-node")));
        } else if (arg == "--bottom-node") {
            bottom_node = static_cast<uint8_t>(std::stoi(next("--bottom-node")));
        } else if (arg == "--counts-per-rev") {
            counts_per_rev_override = static_cast<uint32_t>(std::stoul(next("--counts-per-rev")));
        } else if (arg == "--search-velocity") {
            search_velocity = std::stoi(next("--search-velocity"));
        } else if (arg == "--approach-velocity") {
            approach_velocity = std::stoi(next("--approach-velocity"));
        } else if (arg == "--threshold-torque") {
            threshold_torque = static_cast<int16_t>(std::stoi(next("--threshold-torque")));
        } else if (arg == "--backoff-distance") {
            backoff_distance = std::stoi(next("--backoff-distance"));
        } else if (arg == "--timeout-ms") {
            homing_timeout = std::chrono::milliseconds(std::stoi(next("--timeout-ms")));
        } else if (arg == "--no-save") {
            save_to_nvm = false;
        } else if (arg == "--dcf") {
            base.master_dcf_path = next("--dcf");
        } else if (arg == "--summary") {
            base.summary_path = next("--summary");
        } else if (arg == "--master-node") {
            base.master_node_id = static_cast<uint8_t>(std::stoi(next("--master-node")));
        } else {
            printUsage(argv[0]);
            return EXIT_FAILURE;
        }
    }

    if (!leg_set) {
        std::cerr << "error: --leg is required\n";
        printUsage(argv[0]);
        return EXIT_FAILURE;
    }

    std::vector<LegSegment> segments;
    if (segment_spec == "top") {
        segments = {LegSegment::Top};
    } else if (segment_spec == "bottom") {
        segments = {LegSegment::Bottom};
    } else if (segment_spec == "both") {
        segments = {LegSegment::Top, LegSegment::Bottom};
    } else {
        std::cerr << "--segment must be top, bottom or both\n";
        return EXIT_FAILURE;
    }

    // Each bus carries one leg. If no interface was given, default left->can0,
    // right->can1.
    if (can_interface.empty()) {
        can_interface = (leg == Leg::Left) ? "can0" : "can1";
    }

    // Bring the drives up cyclically without energising them; the homing routine
    // safely switches to CSV and enables the drive it is homing.
    base.can_interface = can_interface;
    base.monitor_on_boot = true;
    base.operation_mode = stablecops::ds402::OperationMode::CyclicSynchronousVelocity;
    if (counts_per_rev_override) {
        base.counts_per_rev = *counts_per_rev_override;
    }

    std::cout << "zeroing " << (leg == Leg::Left ? "left" : "right") << " leg on " << can_interface
              << ", segment(s): " << segment_spec
              << ", save to NVM: " << (save_to_nvm ? "yes" : "no") << '\n';

    // Construct one MotorDrive per node we will touch (all on the same shared
    // bus), keyed by segment so we can home each in turn.
    std::vector<std::pair<LegSegment, std::unique_ptr<stablecops::app::MotorDrive>>> drives;
    for (LegSegment segment : segments) {
        stablecops::app::MotorConfig config = base;
        config.node_id = (segment == LegSegment::Top) ? top_node : bottom_node;
        drives.emplace_back(segment, std::make_unique<stablecops::app::MotorDrive>(config));
    }

    // Actuator values come from the profile (resolved at drive construction);
    // the CLI flags override individual fields.
    const auto& resolved = drives.front().second->config();
    const uint32_t counts_per_rev = counts_per_rev_override.value_or(resolved.counts_per_rev);
    stablecops::ds402::HomingConfig homing = resolved.homing;
    if (search_velocity) {
        homing.search_velocity = *search_velocity;
    }
    if (approach_velocity) {
        homing.approach_velocity = *approach_velocity;
    }
    if (threshold_torque) {
        homing.threshold_torque = *threshold_torque;
    }
    if (backoff_distance) {
        homing.backoff_distance = *backoff_distance;
    }
    if (homing_timeout) {
        homing.timeout = *homing_timeout;
    }
    homing.save_zero_to_nvm = save_to_nvm;

    try {
        drives.front().second->start();
    } catch (const std::exception& exception) {
        std::cerr << "CAN start failed: " << exception.what() << '\n';
        return EXIT_FAILURE;
    }

    // Wait for cyclic feedback so the homing routine can enable safely.
    const auto boot_deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(base.boot_timeout);
    bool all_live = false;
    while (std::chrono::steady_clock::now() < boot_deadline) {
        all_live = true;
        for (auto& entry : drives) {
            if (!entry.second->feedbackLive()) {
                all_live = false;
                break;
            }
        }
        if (all_live) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    if (!all_live) {
        std::cerr << "drives did not report live cyclic feedback after boot; aborting\n";
        drives.front().second->stop();
        return EXIT_FAILURE;
    }

    // Home one drive at a time (top first) so only one joint moves at once.
    bool ok = true;
    for (auto& entry : drives) {
        if (!homeSegment(*entry.second, leg, entry.first, homing, counts_per_rev)) {
            ok = false;
            break;
        }
    }

    // Drives de-energise as they go out of scope; the last one tears the bus
    // down (graceful stop + join).
    std::cout << (ok ? "\nzeroing complete\n" : "\nzeroing aborted\n");
    return ok ? EXIT_SUCCESS : EXIT_FAILURE;
}
