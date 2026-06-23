#include "stablecops/app/CanopenApplication.hpp"

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
#include <iostream>
#include <system_error>
#include <time.h>

namespace stablecops::app {

namespace {

stablecops::lely::BootActionConfig makeBootActions(const MotorConfig& config) {
    stablecops::lely::BootActionConfig actions;
    actions.inspect = config.inspect_on_boot;
    actions.enable = config.enable_on_boot;
    actions.hold_position = config.hold_position_on_boot;
    actions.csp_target_position = config.csp_target_position;
    actions.csp_relative_move = config.csp_relative_move;
    actions.max_position_step = config.max_position_step;
    actions.state_transition_timeout = config.state_transition_timeout;
    return actions;
}

}  // namespace

class CanopenApplication::Impl {
public:
    explicit Impl(const MotorConfig& config)
        : io_guard_(),
          context_(),
          poll_(context_),
          loop_(poll_.get_poll()),
          executor_(loop_.get_executor()),
          controller_(config.can_interface.c_str()),
          channel_(poll_, executor_),
          timer_(poll_, executor_, CLOCK_MONOTONIC),
          shutdown_timer_(poll_, executor_, CLOCK_MONOTONIC),
          master_(timer_, channel_, config.master_dcf_path, "", config.master_node_id),
          motor_(master_, config.node_id, makeBootActions(config)),
          sigset_(poll_, executor_) {
        channel_.open(controller_);

        // On a clean shutdown signal, ramp the drive down before stopping the
        // loop so the joint is never left energised when the master exits.
        motor_.setStoppedCallback([this, node_id = config.node_id] {
            stopNodeThenExit(node_id);
        });
        sigset_.insert(SIGINT);
        sigset_.insert(SIGTERM);
        armSignalWait();
    }

    void armSignalWait() {
        sigset_.submit_wait(executor_, [this](int signo) {
            if (!stop_initiated_) {
                stop_initiated_ = true;
                std::cout << "\nreceived signal " << signo
                          << "; disabling drive and shutting down...\n";
                motor_.requestGracefulStop();
                // Re-arm so a second signal forces an immediate stop if the
                // controlled ramp-down stalls (e.g. lost feedback).
                armSignalWait();
            } else {
                std::cout << "\nsecond signal; forcing immediate shutdown\n";
                loop_.stop();
            }
        });
    }

    void stopNodeThenExit(uint8_t node_id) {
        master_.Command(::lely::canopen::NmtCommand::STOP, node_id);
        shutdown_timer_.settime(std::chrono::milliseconds{50});
        shutdown_timer_.submit_wait(
            executor_,
            [this](int /*overrun*/, std::error_code /*ec*/) {
                context_.shutdown();
                loop_.stop();
            });
    }

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
    stablecops::lely::MotorDriver motor_;
    ::lely::io::SignalSet sigset_;
    bool stop_initiated_{false};
};

CanopenApplication::CanopenApplication(const MotorConfig& config)
    : impl_(std::make_unique<Impl>(config)) {}

CanopenApplication::~CanopenApplication() = default;

stablecops::lely::MotorDriver& CanopenApplication::motor() {
    return impl_->motor_;
}

void CanopenApplication::resetMaster() {
    impl_->master_.Reset();
}

void CanopenApplication::run() {
    impl_->loop_.run();
}

void CanopenApplication::stop() {
    impl_->context_.shutdown();
    impl_->loop_.stop();
}

}  // namespace stablecops::app
