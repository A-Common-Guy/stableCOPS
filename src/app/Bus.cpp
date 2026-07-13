#include "stablecops/app/Bus.hpp"

#include <exception>
#include <future>
#include <map>
#include <stdexcept>
#include <utility>

#include "stablecops/app/CanopenApplication.hpp"
#include "stablecops/app/RealtimeScheduling.hpp"
#include "stablecops/lely/MotorDriver.hpp"

namespace stablecops::app {

namespace {

// Process-wide registry of live buses, keyed by CAN interface name. Holds weak
// references so a bus is destroyed as soon as its last MotorDrive is released.
std::mutex& registryMutex() {
    static std::mutex mutex;
    return mutex;
}

std::map<std::string, std::weak_ptr<Bus>>& registry() {
    static std::map<std::string, std::weak_ptr<Bus>> map;
    return map;
}

}  // namespace

Bus::Bus(const MotorConfig& config) : interface_(config.can_interface), bus_template_(config) {}

Bus::~Bus() {
    shutdown();
    std::lock_guard<std::mutex> lock(registryMutex());
    auto it = registry().find(interface_);
    if (it != registry().end() && it->second.expired()) {
        registry().erase(it);
    }
}

std::shared_ptr<Bus> Bus::getOrCreate(const MotorConfig& config) {
    std::lock_guard<std::mutex> lock(registryMutex());
    auto& map = registry();
    auto it = map.find(config.can_interface);
    if (it != map.end()) {
        if (auto existing = it->second.lock()) {
            existing->validateBusMatch(config);
            return existing;
        }
    }
    // No live bus for this interface (absent or expired); create a fresh one.
    std::shared_ptr<Bus> bus(new Bus(config));
    map[config.can_interface] = bus;
    return bus;
}

void Bus::validateBusMatch(const MotorConfig& config) const {
    const auto& a = bus_template_;
    const bool rt_match = a.rt.enabled == config.rt.enabled &&
                          a.rt.priority == config.rt.priority && a.rt.cpu == config.rt.cpu &&
                          a.rt.lock_memory == config.rt.lock_memory;
    if (a.master_dcf_path != config.master_dcf_path || a.summary_path != config.summary_path ||
        a.master_node_id != config.master_node_id || a.sync_period_us != config.sync_period_us ||
        !rt_match) {
        throw std::invalid_argument(
            "Bus: drive on interface '" + config.can_interface +
            "' has bus-level config that differs from the existing bus (dcf, "
            "summary, master node id, sync period, or rt must match across all "
            "drives on one interface)");
    }
}

void Bus::registerNode(const MotorConfig& config) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (started_) {
        throw std::logic_error("Bus: cannot add a drive to interface '" + interface_ +
                               "' after it has started (construct all drives for a chain before "
                               "starting any of them)");
    }
    for (const auto& existing : node_configs_) {
        if (existing.node_id == config.node_id) {
            throw std::invalid_argument("Bus: node id already registered on interface '" +
                                        interface_ + "'");
        }
    }
    node_configs_.push_back(config);
}

void Bus::start() {
    std::vector<MotorConfig> node_configs;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (started_) {
            return;
        }
        started_ = true;
        node_configs = node_configs_;
    }

    std::promise<void> ready;
    auto ready_future = ready.get_future();

    RtConfig rt = bus_template_.rt;
    loop_thread_ = std::thread([this, &ready, node_configs = std::move(node_configs), rt] {
        // Tune the loop thread before any cyclic work begins (opt-in; degrades
        // gracefully without privileges).
        applyRealtimeScheduling(rt, "stablecops-bus");

        // Construct the application here so the FiberDrivers live on the thread
        // that runs their tasks (required by Lely); destroy it here too.
        std::unique_ptr<CanopenApplication> app;
        try {
            // The embedder (MotorDrive / tools built on it) owns process signals;
            // disable the application's built-in SIGINT/SIGTERM handling so the
            // two don't race for the first Ctrl-C.
            app = std::make_unique<CanopenApplication>(node_configs,
                                                       /*install_signal_handler=*/false);
        } catch (...) {
            running_.store(false);
            ready.set_exception(std::current_exception());
            return;
        }

        app_.store(app.get(), std::memory_order_release);
        running_.store(true);
        app->post([app = app.get()] { app->resetMaster(); });
        ready.set_value();

        app->run();

        // The loop has exited (shutdown or forced stop); report not-running
        // even before shutdown() joins this thread.
        running_.store(false);
        app_.store(nullptr, std::memory_order_release);
        app.reset();  // destroy Lely objects on the loop thread
    });

    try {
        ready_future.get();
    } catch (...) {
        if (loop_thread_.joinable()) {
            loop_thread_.join();
        }
        running_.store(false);
        throw;
    }
}

void Bus::shutdown() {
    if (!loop_thread_.joinable()) {
        running_.store(false);
        return;
    }
    if (auto* app = app_.load(std::memory_order_acquire)) {
        app->post([app] { app->requestShutdown(); });
    }
    loop_thread_.join();
    running_.store(false);
}

bool Bus::forceStop() {
    if (auto* app = app_.load(std::memory_order_acquire)) {
        app->stop();
        return true;
    }
    return false;
}

void Bus::stopNode(uint8_t node_id) {
    postToDriver(node_id,
                 [](stablecops::lely::MotorDriver& motor) { motor.requestGracefulStop(); });
}

bool Bus::running() const {
    return running_.load();
}

void Bus::postToDriver(uint8_t node_id, std::function<void(stablecops::lely::MotorDriver&)> fn) {
    auto* app = app_.load(std::memory_order_acquire);
    if (app == nullptr) {
        return;
    }
    app->post([app, node_id, fn = std::move(fn)]() mutable {
        if (auto* motor = app->motorIfPresent(node_id)) {
            motor->Defer([motor, fn = std::move(fn)] { fn(*motor); });
        }
    });
}

void Bus::invokeOnDriver(uint8_t node_id, std::function<void(stablecops::lely::MotorDriver&)> fn) {
    auto* app = app_.load(std::memory_order_acquire);
    if (app == nullptr) {
        throw std::logic_error("Bus: cannot access node before the bus is running");
    }

    auto done = std::make_shared<std::promise<void>>();
    auto future = done->get_future();
    app->post([app, node_id, fn = std::move(fn), done]() mutable {
        auto* motor = app->motorIfPresent(node_id);
        if (motor == nullptr) {
            try {
                throw std::invalid_argument("Bus: node id is not registered");
            } catch (...) {
                done->set_exception(std::current_exception());
            }
            return;
        }
        motor->Defer([motor, fn = std::move(fn), done] {
            try {
                fn(*motor);
                done->set_value();
            } catch (...) {
                done->set_exception(std::current_exception());
            }
        });
    });
    future.get();
}

ds402::Feedback Bus::feedback(uint8_t node_id) const {
    if (auto* app = app_.load(std::memory_order_acquire)) {
        return app->feedback(node_id);
    }
    return {};
}

bool Bus::feedbackLive(uint8_t node_id) const {
    if (auto* app = app_.load(std::memory_order_acquire)) {
        return app->feedbackLive(node_id);
    }
    return false;
}

stablecops::lely::CyclicStats Bus::cyclicStats() const {
    if (auto* app = app_.load(std::memory_order_acquire)) {
        return app->cyclicStats();
    }
    return {};
}

}  // namespace stablecops::app
