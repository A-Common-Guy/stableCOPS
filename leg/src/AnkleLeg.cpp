#include "stablecops/leg/AnkleLeg.hpp"

#include "stablecops/app/RealtimeScheduling.hpp"
#include "stablecops/ds402/State.hpp"
#include "stablecops/log/Log.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <utility>

namespace stablecops::leg {

namespace {

constexpr double kTwoPi = 6.283185307179586;

// User-facing joint frame -> mapper frame (hardware-validated 2026-07-13):
// the mapper's pitch (y) AND roll (x) are inverted relative to the user frame,
// identically on both legs (the mapper's roll is already mirror-symmetric
// between the legs, so no per-side sign is needed). Each flip is applied
// consistently to the demand and to all joint-side feedback, so PD gains keep
// their usual meaning.
constexpr double kPitchSign = -1.0;
constexpr double kRollSign = -1.0;

app::MotorConfig makeMotorConfig(const AnkleLegConfig& leg, uint8_t node) {
    app::MotorConfig c;
    c.can_interface = leg.can_interface;
    c.node_id = node;
    c.master_node_id = leg.master_node;
    if (!leg.master_dcf_path.empty()) c.master_dcf_path = leg.master_dcf_path;
    if (!leg.summary_path.empty()) c.summary_path = leg.summary_path;
    if (leg.counts_per_rev != 0) c.counts_per_rev = leg.counts_per_rev;
    c.operation_mode = ds402::OperationMode::CyclicSynchronousTorque;
    c.enable_on_boot = true;  // energised; commands 0 torque -> limp until PD acts
    c.rt = leg.rt;
    return c;
}

const char* sideName(flexion::Side side) {
    return side == flexion::Side::Left ? "left" : "right";
}

std::string hexCode(uint16_t code) {
    char buf[8];
    std::snprintf(buf, sizeof(buf), "0x%04X", code);
    return buf;
}

}  // namespace

AnkleLeg::AnkleLeg(AnkleLegConfig config)
    : config_(std::move(config)),
      top_(makeMotorConfig(config_, config_.top_node)),
      bottom_(makeMotorConfig(config_, config_.bottom_node)) {
    if (config_.efficiency <= 0.0 || config_.efficiency > 1.0) {
        throw std::invalid_argument("AnkleLeg: efficiency must be in (0, 1]");
    }
    if (config_.motor_rated_torque_nm < 0.0) {
        throw std::invalid_argument(
            "AnkleLeg: motor_rated_torque_nm must be > 0 (or 0 to read 0x6076)");
    }
    if (config_.rate_hz <= 0.0) {
        throw std::invalid_argument("AnkleLeg: rate_hz must be > 0");
    }
    // inner/outer <-> top/bottom wiring (matches validate_mapper: inner->top).
    inner_ = config_.swap_motors ? &bottom_ : &top_;
    outer_ = config_.swap_motors ? &top_ : &bottom_;
}

AnkleLeg::~AnkleLeg() {
    try {
        stop();
    } catch (...) {
        // never throw out of the destructor
    }
}

void AnkleLeg::start() {
    if (started_) return;

    top_.start();
    bottom_.start();
    started_ = true;  // from here on, stop() must de-energise

    try {
        // Command zero torque immediately so enabling is limp/safe.
        top_.commandTorque(0);
        bottom_.commandTorque(0);

        auto waitEnabled = [&](app::MotorDrive& d, const char* name) {
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
            while (std::chrono::steady_clock::now() < deadline) {
                if (d.feedbackLive() && d.enabled()) return;
                if (d.faulted()) {
                    throw std::runtime_error(std::string("AnkleLeg(") + sideName(config_.side) +
                                             "): " + name + " drive faulted " +
                                             hexCode(d.errorCode()));
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
            throw std::runtime_error(std::string("AnkleLeg(") + sideName(config_.side) + "): " +
                                     name + " drive did not reach operation enabled");
        };
        waitEnabled(top_, "top");
        waitEnabled(bottom_, "bottom");
        top_.commandTorque(0);
        bottom_.commandTorque(0);

        resolveScaling();
    } catch (...) {
        stop();
        throw;
    }

    {
        std::lock_guard<std::mutex> lock(error_mutex_);
        last_error_.clear();
    }
    running_.store(true);
    thread_ = std::thread(&AnkleLeg::controlLoop, this);
}

// Resolve the Nm -> 0x6071 per-mille scaling; ankle_pd_torque's logic verbatim.
void AnkleLeg::resolveScaling() {
    // Actuator constants come from the motor profile unless pinned in the
    // config. Wrong torque scaling is a safety hazard, so an unknown gear
    // ratio is a hard error rather than a guess.
    gear_ratio_ = config_.gear_ratio > 0.0 ? config_.gear_ratio : top_.config().gear_ratio;
    if (gear_ratio_ <= 0.0) {
        throw std::runtime_error(
            "AnkleLeg: gear ratio unknown -- set runtime.gear_ratio in the motor profile or "
            "AnkleLegConfig::gear_ratio");
    }
    counts_per_rev_ =
        config_.counts_per_rev != 0 ? config_.counts_per_rev : top_.config().counts_per_rev;
    count_to_rad_ = kTwoPi / static_cast<double>(counts_per_rev_);

    // Ground the per-mille torque base in what the drive itself executes:
    // 0x6071 is per-mille of MOTOR rated torque 0x6076 (mNm). Refuse to run on
    // a guess if the read fails or the two drives disagree.
    motor_rated_torque_nm_ = config_.motor_rated_torque_nm;
    if (motor_rated_torque_nm_ <= 0.0) {
        const int64_t top_mNm = top_.readObject(0x6076, 0x00, app::ObjectDataType::U32);
        const int64_t bottom_mNm = bottom_.readObject(0x6076, 0x00, app::ObjectDataType::U32);
        if (top_mNm <= 0 || top_mNm != bottom_mNm) {
            throw std::runtime_error(
                "AnkleLeg: 0x6076 rated torque mismatch/invalid: top=" + std::to_string(top_mNm) +
                " bottom=" + std::to_string(bottom_mNm) + " mNm");
        }
        motor_rated_torque_nm_ = static_cast<double>(top_mNm) / 1000.0;
    }

    // Cross-check the mechanical gear ratio against what the drive implies
    // (0x2009 max MOTOR rpm vs 0x607F max profile velocity, OUTPUT counts/s).
    // Warning-only: 0x607F is a user-writable limit, never the source of truth.
    try {
        const int64_t max_motor_rpm = top_.readObject(0x2009, 0x00, app::ObjectDataType::U32);
        const int64_t max_out_cps = top_.readObject(0x607F, 0x00, app::ObjectDataType::U32);
        if (max_motor_rpm > 0 && max_out_cps > 0) {
            const double out_rpm =
                static_cast<double>(max_out_cps) / static_cast<double>(counts_per_rev_) * 60.0;
            const double derived = static_cast<double>(max_motor_rpm) / out_rpm;
            if (std::fabs(derived - gear_ratio_) / gear_ratio_ > 0.01) {
                log::warn() << "AnkleLeg(" << sideName(config_.side)
                            << "): drive-implied gear ratio " << derived
                            << " differs from configured " << gear_ratio_
                            << " by >1% -- check the actuator model / counts_per_rev\n";
            }
        }
    } catch (const std::exception& e) {
        log::warn() << "AnkleLeg(" << sideName(config_.side)
                    << "): gear-ratio cross-check skipped (SDO read failed: " << e.what()
                    << ")\n";
    }

    // Nm (output) -> 0x6071 units: 1000 units = motor rated torque at the
    // motor shaft = x gear_ratio x efficiency at the output.
    tau_to_units_ = 1000.0 / (motor_rated_torque_nm_ * gear_ratio_ * config_.efficiency);
}

void AnkleLeg::stop() {
    running_.store(false);
    if (thread_.joinable()) thread_.join();
    if (started_) {
        top_.commandTorque(0);
        bottom_.commandTorque(0);
        top_.stop();
        bottom_.stop();
        started_ = false;
    }
}

bool AnkleLeg::running() const { return running_.load(); }

std::string AnkleLeg::lastError() const {
    std::lock_guard<std::mutex> lock(error_mutex_);
    return last_error_;
}

void AnkleLeg::setTargets(const AnkleTargets& targets) {
    pitch_des_.store(targets.pitch_rad);
    roll_des_.store(targets.roll_rad);
}

void AnkleLeg::setGains(const AnkleGains& gains) {
    kp_pitch_.store(gains.kp_pitch);
    kd_pitch_.store(gains.kd_pitch);
    kp_roll_.store(gains.kp_roll);
    kd_roll_.store(gains.kd_roll);
}

AnkleTargets AnkleLeg::targets() const {
    AnkleTargets t;
    t.pitch_rad = pitch_des_.load();
    t.roll_rad = roll_des_.load();
    return t;
}

AnkleGains AnkleLeg::gains() const {
    AnkleGains g;
    g.kp_pitch = kp_pitch_.load();
    g.kd_pitch = kd_pitch_.load();
    g.kp_roll = kp_roll_.load();
    g.kd_roll = kd_roll_.load();
    return g;
}

void AnkleLeg::limp() { setGains(AnkleGains{}); }

AnkleFeedback AnkleLeg::feedback() const {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    return feedback_;
}

void AnkleLeg::fail(const std::string& reason) {
    {
        std::lock_guard<std::mutex> lock(error_mutex_);
        last_error_ = reason;
    }
    log::err() << "AnkleLeg(" << sideName(config_.side) << "): " << reason << '\n';
    running_.store(false);
}

void AnkleLeg::controlLoop() {
    {
        // The bus loop thread carries the cyclic traffic; run the PD slightly
        // below it (same convention as ankle_pd_torque).
        app::RtConfig rt = config_.rt;
        rt.priority = std::max(1, rt.priority - 5);
        const std::string name = std::string("ankle-") + sideName(config_.side);
        app::applyRealtimeScheduling(rt, name.c_str());
    }

    const flexion::Side side = config_.side;
    const int torque_sign = config_.torque_sign;
    const double nm_motor_per_unit = motor_rated_torque_nm_ / 1000.0;  // 0x6077 unit -> motor Nm
    const auto period = std::chrono::duration<double>(1.0 / config_.rate_hz);

    while (running_.load()) {
        if (!top_.feedbackLive() || !bottom_.feedbackLive()) {
            fail("CAN feedback lost");
            break;
        }
        if (top_.faulted() || bottom_.faulted()) {
            fail("drive fault: top=" + hexCode(top_.errorCode()) + " bottom=" +
                 hexCode(bottom_.errorCode()));
            break;
        }

        const ds402::Feedback inner_fb = inner_->feedback();
        const ds402::Feedback outer_fb = outer_->feedback();

        // 1) motor state -> actuator (inner/outer) pos & vel [rad, rad/s]
        flexion::AnkleActuators act_pos, act_vel;
        act_pos.inner = static_cast<double>(inner_fb.position) * count_to_rad_;
        act_pos.outer = static_cast<double>(outer_fb.position) * count_to_rad_;
        act_vel.inner = static_cast<double>(inner_fb.velocity) * count_to_rad_;
        act_vel.outer = static_cast<double>(outer_fb.velocity) * count_to_rad_;

        // 2) forward map -> joint pos/vel (y=pitch, x=roll)
        const flexion::AnkleJoints jpos = mapper_.mapAnklePosActuatorToJoint(act_pos, side);
        const flexion::AnkleJoints jvel = mapper_.mapAnkleVelActuatorToJoint(act_pos, act_vel, side);

        // 3) joint-space PD, per axis, in the mapper frame: the user-frame
        //    demand is flipped into it (kPitchSign / kRollSign); feedback is
        //    flipped back the same way when published below.
        const double y_des = kPitchSign * pitch_des_.load();
        const double x_des = kRollSign * roll_des_.load();
        flexion::AnkleJoints tau_joint;
        tau_joint.y = kp_pitch_.load() * (y_des - jpos.y) - kd_pitch_.load() * jvel.y;
        tau_joint.x = kp_roll_.load() * (x_des - jpos.x) - kd_roll_.load() * jvel.x;

        // 4) joint torque -> actuator torque [Nm]
        const flexion::AnkleActuators tau_act =
            mapper_.mapAnkleTorqueJointToActuator(act_pos, tau_joint, side);

        // 5) Nm -> drive units, command CST. Only int16 overflow guard (no
        //    clamp; the drive's 0x6072 max torque still applies).
        auto toUnits = [&](double nm) -> int16_t {
            double u = torque_sign * nm * tau_to_units_;
            if (u > 32767.0) u = 32767.0;
            if (u < -32768.0) u = -32768.0;
            return static_cast<int16_t>(std::lround(u));
        };
        inner_->commandTorque(toUnits(tau_act.inner));
        outer_->commandTorque(toUnits(tau_act.outer));

        // Measured torque back through the same scaling: 0x6077 per-mille ->
        // motor Nm -> output Nm (mapper actuator frame), then mapped to the
        // joint side.
        auto motorNm = [&](const ds402::Feedback& fb) {
            return torque_sign * static_cast<double>(fb.torque) * nm_motor_per_unit;
        };
        const double inner_motor_nm = motorNm(inner_fb);
        const double outer_motor_nm = motorNm(outer_fb);
        const double out_per_motor = gear_ratio_ * config_.efficiency;
        flexion::AnkleActuators tau_act_meas;
        tau_act_meas.inner = inner_motor_nm * out_per_motor;
        tau_act_meas.outer = outer_motor_nm * out_per_motor;
        const flexion::AnkleJoints jtau_meas =
            mapper_.mapAnkleTorqueActuatorToJoint(act_pos, tau_act_meas, side);

        // Publish one coherent snapshot, joint side flipped back to the
        // user frame.
        AnkleFeedback snap;
        snap.valid = true;
        snap.pitch_rad = kPitchSign * jpos.y;
        snap.roll_rad = kRollSign * jpos.x;
        snap.pitch_vel_rad_s = kPitchSign * jvel.y;
        snap.roll_vel_rad_s = kRollSign * jvel.x;
        snap.joint_torque_pitch_nm = kPitchSign * jtau_meas.y;
        snap.joint_torque_roll_nm = kRollSign * jtau_meas.x;
        snap.cmd_torque_pitch_nm = kPitchSign * tau_joint.y;
        snap.cmd_torque_roll_nm = kRollSign * tau_joint.x;
        auto motorSnap = [&](const ds402::Feedback& fb, double motor_nm) {
            MotorFeedback m;
            m.position_rad = static_cast<double>(fb.position) * count_to_rad_;
            m.velocity_rad_s = static_cast<double>(fb.velocity) * count_to_rad_;
            m.torque_motor_nm = motor_nm;
            m.torque_output_nm = motor_nm * out_per_motor;
            return m;
        };
        snap.top = motorSnap(config_.swap_motors ? outer_fb : inner_fb,
                             config_.swap_motors ? outer_motor_nm : inner_motor_nm);
        snap.bottom = motorSnap(config_.swap_motors ? inner_fb : outer_fb,
                                config_.swap_motors ? inner_motor_nm : outer_motor_nm);
        {
            std::lock_guard<std::mutex> lock(feedback_mutex_);
            feedback_ = snap;
        }

        std::this_thread::sleep_for(period);
    }

    // Always leave the drives commanded to zero torque (limp), fault or not.
    top_.commandTorque(0);
    bottom_.commandTorque(0);
    running_.store(false);
}

}  // namespace stablecops::leg
