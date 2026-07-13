#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "stablecops/app/MotorConfig.hpp"
#include "stablecops/ds402/DriveController.hpp"
#include "stablecops/lely/CyclicStats.hpp"

// Forward-declared so this header stays free of Lely includes; the callbacks
// passed to postToDriver/invokeOnDriver need the full MotorDriver header only
// in the translation units that define them.
namespace stablecops::lely {
class MotorDriver;
}

namespace stablecops::app {

class CanopenApplication;

// One physical CAN interface = one Lely master, one event-loop thread, one SYNC
// stream. Bus owns those and is shared (ref-counted) by every MotorDrive that
// names the same interface, so drives on one chain transparently share the
// master/loop/SYNC while different interfaces stay fully independent.
//
// Hidden behind a process-wide registry keyed by interface name; users only
// ever hold MotorDrive handles. The first drive for an interface creates the
// bus, siblings join it, and the last MotorDrive released tears it down.
//
// Lifecycle is static: register all nodes, then start() boots the whole chain
// once. Registering a node after start() throws.
class Bus {
public:
    // Get (or lazily create) the shared bus for config.can_interface. The
    // bus-level fields (dcf, summary, master node id, sync period, rt) must
    // match an existing bus for that interface; a mismatch throws.
    static std::shared_ptr<Bus> getOrCreate(const MotorConfig& config);

    ~Bus();

    Bus(const Bus&) = delete;
    Bus& operator=(const Bus&) = delete;

    // Add a drive to the chain. Must be called before start(); throws if the bus
    // has already started or the node id is already registered.
    void registerNode(const MotorConfig& config);

    // Boot the chain once. Idempotent: the first call constructs the
    // application on the loop thread (all registered nodes) and resets the
    // master; later calls (e.g. from sibling drives) are no-ops. Blocks until
    // the application is constructed and rethrows any construction error.
    void start();

    // De-energise a single node (graceful stop) without tearing the bus down.
    void stopNode(uint8_t node_id);

    // Graceful-stop every drive and join the loop thread. Idempotent; normally
    // reached from teardown, but tools can call it explicitly to keep signal
    // handling alive while the shutdown completes.
    void shutdown();

    // Immediate escape hatch for a stuck graceful shutdown. This stops the Lely
    // context/loop directly; the caller still owns joining through shutdown().
    // Returns false if the application has not been constructed yet.
    bool forceStop();

    bool running() const;

    // Route a call to a node's MotorDriver on the loop thread. No-op if the bus
    // is not running or the node is absent.
    void postToDriver(uint8_t node_id, std::function<void(stablecops::lely::MotorDriver&)> fn);

    // Route a call to a node's MotorDriver on the loop thread and wait for it to
    // finish. Exceptions thrown by the callback are rethrown on the caller's
    // thread. Use this for synchronous SDO/object access from application code.
    void invokeOnDriver(uint8_t node_id, std::function<void(stablecops::lely::MotorDriver&)> fn);

    ds402::Feedback feedback(uint8_t node_id) const;
    bool feedbackLive(uint8_t node_id) const;
    stablecops::lely::CyclicStats cyclicStats() const;

    const std::string& interface() const { return interface_; }

private:
    explicit Bus(const MotorConfig& config);

    // Validate that config's bus-level fields match this bus; throws otherwise.
    void validateBusMatch(const MotorConfig& config) const;
    std::string interface_;
    // Bus-level fields snapshot from the first drive, used to validate siblings.
    MotorConfig bus_template_;
    std::vector<MotorConfig> node_configs_;
    mutable std::mutex mutex_;

    std::atomic<CanopenApplication*> app_{nullptr};
    std::thread loop_thread_;
    std::atomic<bool> running_{false};
    bool started_{false};
};

}  // namespace stablecops::app
