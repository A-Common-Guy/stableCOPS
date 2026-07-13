#include "stablecops/app/CanopenApplication.hpp"

#include "stablecops/config/PdoMap.hpp"
#include "stablecops/lely/MotorDriver.hpp"
#include "stablecops/log/Log.hpp"

#include <lely/coapp/master.hpp>
#include <lely/ev/loop.hpp>
#include <lely/io2/ctx.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/sigset.hpp>
#include <lely/io2/sys/timer.hpp>

#include <chrono>
#include <csignal>
#include <optional>
#include <stdexcept>
#include <system_error>
#include <utility>
#include <time.h>

namespace stablecops::app {

namespace {

stablecops::lely::BootActionConfig makeBootActions(const MotorConfig& config) {
    stablecops::lely::BootActionConfig actions;
    actions.inspect = config.inspect_on_boot;
    actions.enable = config.enable_on_boot;
    actions.hold_position = config.hold_position_on_boot;
    actions.monitor = config.monitor_on_boot;
    actions.mode = config.operation_mode;
    actions.profile_velocity = config.profile_velocity;
    actions.profile_acceleration = config.profile_acceleration;
    actions.profile_deceleration = config.profile_deceleration;
    actions.torque_slope = config.torque_slope;
    actions.disable_mode = config.disable_mode;
    actions.object_writes = config.object_writes;
    actions.save_params = config.save_params;
    actions.csp_target_position = config.csp_target_position;
    actions.csp_relative_move = config.csp_relative_move;
    actions.max_position_step = config.max_position_step;
    actions.counts_per_rev = config.counts_per_rev;
    actions.state_transition_timeout = config.state_transition_timeout;
    actions.feedback_timeout = config.feedback_timeout;
    actions.sync_period_us = config.sync_period_us;
    return actions;
}

std::vector<MotorConfig> wrapSingle(const MotorConfig& config) {
    return std::vector<MotorConfig>{config};
}

std::vector<MotorConfig> requireNonEmpty(std::vector<MotorConfig> configs) {
    if (configs.empty()) {
        throw std::invalid_argument(
            "CanopenApplication requires at least one node config");
    }
    return configs;
}

}  // namespace

class CanopenApplication::Impl {
public:
    explicit Impl(std::vector<MotorConfig> node_configs, bool install_signal_handler)
        : configs_(requireNonEmpty(std::move(node_configs))),
          io_guard_(),
          context_(),
          poll_(context_),
          loop_(poll_.get_poll()),
          executor_(loop_.get_executor()),
          controller_(configs_.front().can_interface.c_str()),
          channel_(poll_, executor_),
          timer_(poll_, executor_, CLOCK_MONOTONIC),
          shutdown_timer_(poll_, executor_, CLOCK_MONOTONIC),
          master_(timer_, channel_, configs_.front().master_dcf_path, "",
                  configs_.front().master_node_id) {
        channel_.open(controller_);

        for (const auto& config : configs_) {
            node_ids_.push_back(config.node_id);
            // Homogeneous chains share the same PDO object mapping, but each
            // drive needs node-ID-specific COB-IDs on the wire.
            const auto pdo_map =
                stablecops::config::loadPdoMapFromSummary(config.summary_path, config.node_id);
            auto motor = std::make_unique<stablecops::lely::MotorDriver>(
                master_, config.node_id, makeBootActions(config), pdo_map);
            // On a coordinated shutdown, each drive reports when it is
            // de-energised; once all have, the bus resets and the loop stops.
            motor->setStoppedCallback([this] { onMotorStopped(); });
            motors_.push_back(std::move(motor));
        }

        // Only install the built-in SIGINT/SIGTERM handling when this
        // application owns the process lifecycle (e.g. stablecops_master, which
        // runs the loop on the main thread and has no handler of its own). When
        // embedded behind the Bus/MotorDrive API the embedder owns signals, so
        // installing it here would just steal the first Ctrl-C from them and
        // require an extra press to actually shut down.
        if (install_signal_handler) {
            sigset_.emplace(poll_, executor_);
            sigset_->insert(SIGINT);
            sigset_->insert(SIGTERM);
            armSignalWait();
        }
    }

    void armSignalWait() {
        sigset_->submit_wait(executor_, [this](int signo) {
            if (!shutdown_initiated_) {
                stablecops::log::out() << "\nreceived signal " << signo
                          << "; disabling drives and shutting down...\n";
                requestShutdown();
                // Re-arm so a second signal forces an immediate stop if the
                // controlled ramp-down stalls (e.g. lost feedback).
                armSignalWait();
            } else {
                stablecops::log::out() << "\nsecond signal; forcing immediate shutdown\n";
                loop_.stop();
            }
        });
    }

    void requestShutdown() {
        if (shutdown_initiated_) {
            return;
        }
        shutdown_initiated_ = true;
        stops_remaining_ = motors_.size();
        for (auto& motor : motors_) {
            motor->requestGracefulStop();
        }
        // Fallback: even if some drive never confirms de-energising (e.g. it
        // already went silent), tear the bus down after a bounded grace period.
        shutdown_deadline_ =
            std::chrono::steady_clock::now() + std::chrono::milliseconds{1500};
        scheduleShutdownCheck();
    }

    void onMotorStopped() {
        if (!shutdown_initiated_) {
            // A single-drive de-energise that is not a full bus teardown.
            return;
        }
        if (stops_remaining_ > 0) {
            --stops_remaining_;
        }
        if (stops_remaining_ == 0) {
            finishShutdown(/*reset_nodes=*/false);
        }
    }

    void scheduleShutdownCheck() {
        shutdown_timer_.settime(std::chrono::milliseconds{50});
        shutdown_timer_.submit_wait(
            executor_, [this](int /*overrun*/, std::error_code /*ec*/) {
                if (!shutdown_complete_ &&
                    std::chrono::steady_clock::now() >= shutdown_deadline_) {
                    finishShutdown(/*reset_nodes=*/true);
                } else if (!shutdown_complete_) {
                    scheduleShutdownCheck();
                }
            });
    }

    void finishShutdown(bool reset_nodes) {
        if (shutdown_complete_) {
            return;
        }
        shutdown_complete_ = true;

        if (!reset_nodes) {
            context_.shutdown();
            loop_.stop();
            return;
        }

        // Fallback only: if a drive never confirms de-energising, reset each
        // node as a final power-stage drop. Normal shutdown follows the vendor
        // SDK pattern and exits quietly after graceful stops.
        for (uint8_t node_id : node_ids_) {
            master_.Command(::lely::canopen::NmtCommand::RESET_NODE, node_id);
        }

        // Give reset frames time to leave the CAN channel before teardown.
        shutdown_timer_.settime(std::chrono::milliseconds{50});
        shutdown_timer_.submit_wait(
            executor_, [this](int /*overrun*/, std::error_code /*ec*/) {
                context_.shutdown();
                loop_.stop();
            });
    }

    stablecops::lely::MotorDriver* motorFor(uint8_t node_id) {
        for (std::size_t i = 0; i < node_ids_.size(); ++i) {
            if (node_ids_[i] == node_id) {
                return motors_[i].get();
            }
        }
        return nullptr;
    }

    std::vector<MotorConfig> configs_;
    std::vector<uint8_t> node_ids_;

    ::lely::io::IoGuard io_guard_;
    ::lely::io::Context context_;
    ::lely::io::Poll poll_;
    ::lely::ev::Loop loop_;
    ::lely::ev::Executor executor_;
    ::lely::io::CanController controller_;
    ::lely::io::CanChannel channel_;
    ::lely::io::Timer timer_;
    ::lely::io::Timer shutdown_timer_;
    ::lely::canopen::AsyncMaster master_;
    std::vector<std::unique_ptr<stablecops::lely::MotorDriver>> motors_;
    // Engaged only when the application installs its own signal handling; empty
    // when the embedder (Bus/MotorDrive) owns process signals.
    std::optional<::lely::io::SignalSet> sigset_;

    bool shutdown_initiated_{false};
    bool shutdown_complete_{false};
    std::size_t stops_remaining_{0};
    std::chrono::steady_clock::time_point shutdown_deadline_{};
};

CanopenApplication::CanopenApplication(std::vector<MotorConfig> node_configs,
                                       bool install_signal_handler)
    : impl_(std::make_unique<Impl>(std::move(node_configs), install_signal_handler)) {}

CanopenApplication::CanopenApplication(const MotorConfig& config, bool install_signal_handler)
    : CanopenApplication(wrapSingle(config), install_signal_handler) {}

CanopenApplication::~CanopenApplication() = default;

const std::vector<uint8_t>& CanopenApplication::nodeIds() const {
    return impl_->node_ids_;
}

stablecops::lely::MotorDriver& CanopenApplication::motor() {
    return *impl_->motors_.front();
}

stablecops::lely::MotorDriver* CanopenApplication::motorIfPresent(uint8_t node_id) {
    return impl_->motorFor(node_id);
}

void CanopenApplication::resetMaster() {
    impl_->master_.Reset();
}

void CanopenApplication::requestShutdown() {
    impl_->requestShutdown();
}

void CanopenApplication::post(std::function<void()> task) {
    impl_->executor_.post(std::move(task));
}

ds402::Feedback CanopenApplication::feedback() const {
    return impl_->motors_.front()->feedbackSnapshot();
}

ds402::Feedback CanopenApplication::feedback(uint8_t node_id) const {
    if (auto* motor = impl_->motorFor(node_id)) {
        return motor->feedbackSnapshot();
    }
    return {};
}

bool CanopenApplication::feedbackLive() const {
    return impl_->motors_.front()->feedbackLive();
}

bool CanopenApplication::feedbackLive(uint8_t node_id) const {
    if (auto* motor = impl_->motorFor(node_id)) {
        return motor->feedbackLive();
    }
    return false;
}

stablecops::lely::CyclicStats CanopenApplication::cyclicStats() const {
    return impl_->motors_.front()->cyclicStats();
}

void CanopenApplication::run() {
    impl_->loop_.run();
}

void CanopenApplication::stop() {
    impl_->context_.shutdown();
    impl_->loop_.stop();
}

}  // namespace stablecops::app
