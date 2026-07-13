#pragma once

// AnkleLeg -- joint-space PD torque control of one closed-chain ankle over CAN.
//
// One AnkleLeg owns the two CAN drives of one leg (top + bottom node on one CAN
// interface) and runs the full mapper pipeline of leg/examples/ankle_pd_torque
// on an internal control thread, with NO EtherCAT dependency: all feedback is
// derived from the drives' cyclic feedback through the closed-chain mapper.
//
// Per control cycle (joint space = foot pitch/roll, mapper convention
// y = pitch, x = roll):
//   1. Read motor angles/velocities from the two CAN drives.
//   2. Mapper forward map -> joint pitch/roll and their velocities.
//   3. Per-axis PD: tau = kp*(target - measured) - kd*measured_vel.
//   4. Mapper torque map (joint -> actuator) -> per-motor torque [Nm].
//   5. Nm -> 0x6071 per-mille units -> CST command.
// The measured drive torque (0x6077) runs the same chain backwards, so torque
// feedback is available both per motor (Nm) and joint-side (Nm, mapped).
//
// Joint-frame sign convention (hardware-validated 2026-07-13): the USER-FACING
// frame has the same positive pitch on both legs and mirror-symmetric roll.
// The raw mapper frame is already mirror-symmetric between the legs but has
// BOTH axes pointing the other way: pitch and roll are each inverted relative
// to the user frame, identically on both legs (kPitchSign/kRollSign in
// AnkleLeg.cpp). AnkleLeg applies the flips internally -- on the demand going
// into the PD and on every joint-side feedback field coming out -- so targets,
// joint feedback and joint torques are all user-frame. Motor-level feedback
// stays in the mapper's actuator frame (it is below the joint mapping).
//
// Torque scaling follows ankle_pd_torque exactly (see its header comment for
// the full investigation): the per-mille base is the MOTOR rated torque 0x6076
// read from both drives at startup (they must agree; never guessed), the gear
// ratio comes from the motor profile (runtime.gear_ratio), and efficiency
// defaults to 1.0 (ideal, errs toward commanding less torque).
//
// Prerequisites: both drives zeroed to the mechanical joint zero (zero_leg,
// NVM); the raw absolute position is used directly as the actuator angle.
//
// Lifecycle: construct all legs first (drives register on their shared buses),
// then start() each. start() boots the chain, enables both drives in CST at
// zero torque (limp), resolves the torque scaling and launches the control
// thread. Gains start at zero, so the leg stays limp until setGains() raises
// them. stop() (also run by the destructor) commands zero torque and
// de-energises.
//
// SAFETY: torque control with no artificial clamp (the drive's 0x6072 max
// torque still applies). Bring kp up slowly; if the joint DIVERGES the torque
// sign is wrong -- use AnkleLegConfig::torque_sign = -1. Keep the leg clear.
//
// Thread-safety: setTargets/setGains/limp/feedback/targets/gains/running/
// lastError may be called from any thread while the loop runs. start/stop must
// not race each other.

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include "stablecops/app/MotorConfig.hpp"
#include "stablecops/app/MotorDrive.hpp"

#include "mapper.hpp"

namespace stablecops::leg {

// Per-axis joint-space PD gains (Nm/rad, Nm/(rad/s)). Zero = limp.
struct AnkleGains {
    double kp_pitch{0.0};
    double kd_pitch{0.0};
    double kp_roll{0.0};
    double kd_roll{0.0};
};

// Joint-space position demand, radians, in the user-facing joint frame (same
// pitch direction on both legs, mirror-symmetric roll; see header comment).
struct AnkleTargets {
    double pitch_rad{0.0};
    double roll_rad{0.0};
};

// Feedback for one motor, actuator (output shaft) side, mapper sign frame.
struct MotorFeedback {
    double position_rad{0.0};      // output-shaft angle (counts_per_rev scaled)
    double velocity_rad_s{0.0};    // output-shaft velocity
    double torque_motor_nm{0.0};   // measured torque at the MOTOR shaft (0x6077)
    double torque_output_nm{0.0};  // = motor torque x gear_ratio x efficiency
};

// One coherent snapshot of the leg, produced once per control cycle.
struct AnkleFeedback {
    bool valid{false};  // false until the loop has completed one cycle

    // Joint side, through the mapper, in the user-facing joint frame (see
    // header comment on the sign convention).
    double pitch_rad{0.0};
    double roll_rad{0.0};
    double pitch_vel_rad_s{0.0};
    double roll_vel_rad_s{0.0};

    // Measured joint torque, mapped actuator -> joint from the drives' 0x6077.
    double joint_torque_pitch_nm{0.0};
    double joint_torque_roll_nm{0.0};

    // Joint torque the PD commanded this cycle (before mapping), for telemetry.
    double cmd_torque_pitch_nm{0.0};
    double cmd_torque_roll_nm{0.0};

    // Motor side, by physical drive (top = inner actuator unless swap_motors).
    MotorFeedback top;
    MotorFeedback bottom;
};

struct AnkleLegConfig {
    std::string can_interface{"can0"};
    flexion::Side side{flexion::Side::Left};

    // Wiring: node 1 = top drive = inner actuator, node 2 = bottom = outer
    // (matches validate_mapper / ankle_pd_torque). swap_motors exchanges the
    // inner/outer assignment if a leg turns out to be cabled the other way.
    uint8_t top_node{1};
    uint8_t bottom_node{2};
    bool swap_motors{false};

    uint8_t master_node{127};
    std::string master_dcf_path;  // empty = MotorConfig default
    std::string summary_path;     // empty = MotorConfig default

    double rate_hz{500.0};  // control loop rate

    // Torque scaling (see ankle_pd_torque.cpp for the rationale). Zero means
    // "resolve at start()": rated torque from the drives (0x6076, both must
    // agree), gear ratio / counts per rev from the motor profile. An
    // unresolvable value is a hard error, never a guess.
    double motor_rated_torque_nm{0.0};
    double gear_ratio{0.0};
    uint32_t counts_per_rev{0};
    double efficiency{1.0};  // (0, 1]; 1.0 = ideal/lossless
    int torque_sign{1};      // flips commanded AND measured torque

    // Applied to the bus loop thread and (at priority-5) the control thread.
    app::RtConfig rt;
};

class AnkleLeg {
public:
    // Constructs (and registers on the shared bus) both drives; does not talk
    // to the hardware yet. Throws on invalid config values.
    explicit AnkleLeg(AnkleLegConfig config);
    ~AnkleLeg();  // stop()

    AnkleLeg(const AnkleLeg&) = delete;
    AnkleLeg& operator=(const AnkleLeg&) = delete;

    // Boot the chain, enable both drives in CST at zero torque (limp), resolve
    // the torque scaling and start the control thread. Throws std::runtime_error
    // on any failure (boot, enable timeout, 0x6076 mismatch, unknown gear
    // ratio, ...); the drives are de-energised before the throw.
    void start();

    // Stop the control thread, command zero torque and de-energise both
    // drives. Idempotent.
    void stop();

    // True while the control loop is alive and both drives are healthy. Goes
    // false on fault / lost feedback; lastError() then says why.
    bool running() const;
    std::string lastError() const;

    // Joint-space demand and gains, picked up by the next control cycle.
    void setTargets(const AnkleTargets& targets);
    void setGains(const AnkleGains& gains);
    AnkleTargets targets() const;
    AnkleGains gains() const;

    // Zero all gains: both motors limp (still energised, zero torque).
    void limp();

    // Latest feedback snapshot (one coherent control cycle).
    AnkleFeedback feedback() const;

    const AnkleLegConfig& config() const { return config_; }

    // Resolved torque-scaling constants, valid after start().
    double gearRatio() const { return gear_ratio_; }
    double motorRatedTorqueNm() const { return motor_rated_torque_nm_; }
    uint32_t countsPerRev() const { return counts_per_rev_; }

private:
    void controlLoop();
    void resolveScaling();
    void fail(const std::string& reason);

    AnkleLegConfig config_;
    app::MotorDrive top_;
    app::MotorDrive bottom_;
    // inner/outer <-> top/bottom wiring, resolved once from swap_motors.
    app::MotorDrive* inner_{nullptr};
    app::MotorDrive* outer_{nullptr};

    flexion::Mapper mapper_;

    // Resolved at start().
    double motor_rated_torque_nm_{0.0};
    double gear_ratio_{0.0};
    uint32_t counts_per_rev_{0};
    double count_to_rad_{0.0};
    double tau_to_units_{0.0};  // Nm (output) -> 0x6071 per-mille units

    // Demand/gains, written by the API, read by the control loop.
    std::atomic<double> pitch_des_{0.0};
    std::atomic<double> roll_des_{0.0};
    std::atomic<double> kp_pitch_{0.0};
    std::atomic<double> kd_pitch_{0.0};
    std::atomic<double> kp_roll_{0.0};
    std::atomic<double> kd_roll_{0.0};

    std::atomic<bool> running_{false};
    bool started_{false};
    std::thread thread_;

    mutable std::mutex feedback_mutex_;
    AnkleFeedback feedback_;

    mutable std::mutex error_mutex_;
    std::string last_error_;
};

}  // namespace stablecops::leg
