#include "stablecops/app/MotorDrive.hpp"

#include <utility>

#include "stablecops/app/Bus.hpp"
#include "stablecops/lely/MotorDriver.hpp"

namespace stablecops::app {

MotorDrive::MotorDrive(MotorConfig config) : config_(std::move(config)), node_id_(config_.node_id) {
    // Join (or create) the hidden shared bus for this interface and register
    // this node before anyone starts the chain.
    bus_ = Bus::getOrCreate(config_);
    bus_->registerNode(config_);
}

MotorDrive::~MotorDrive() {
    stop();
    // Releasing bus_ here drops this drive's reference; when the last drive on
    // the interface is destroyed the bus tears itself down (graceful stop +
    // join) in its destructor.
}

void MotorDrive::start() {
    bus_->start();
}

void MotorDrive::stop() {
    if (bus_ && bus_->running()) {
        bus_->stopNode(node_id_);
    }
}

void MotorDrive::shutdownBus() {
    if (bus_) {
        bus_->shutdown();
    }
}

bool MotorDrive::forceStopBus() {
    if (bus_) {
        return bus_->forceStop();
    }
    return false;
}

bool MotorDrive::running() const {
    return bus_ && bus_->running();
}

ds402::Feedback MotorDrive::feedback() const {
    return bus_->feedback(node_id_);
}

bool MotorDrive::feedbackLive() const {
    return bus_->feedbackLive(node_id_);
}

stablecops::lely::CyclicStats MotorDrive::cyclicStats() const {
    return bus_->cyclicStats();
}

double MotorDrive::positionDegrees() const {
    if (config_.counts_per_rev == 0) {
        return 0.0;
    }
    return static_cast<double>(feedback().position) / static_cast<double>(config_.counts_per_rev) *
           360.0;
}

double MotorDrive::positionRadians() const {
    if (config_.counts_per_rev == 0) {
        return 0.0;
    }
    constexpr double kTwoPi = 6.283185307179586;
    return static_cast<double>(feedback().position) / static_cast<double>(config_.counts_per_rev) *
           kTwoPi;
}

bool MotorDrive::enabled() const {
    return feedbackLive() && feedback().state == ds402::State::OperationEnabled;
}

bool MotorDrive::faulted() const {
    const auto fb = feedback();
    return fb.state == ds402::State::Fault || fb.state == ds402::State::FaultReactionActive ||
           fb.error_code != 0;
}

uint16_t MotorDrive::errorCode() const {
    return feedback().error_code;
}

void MotorDrive::resetFault() {
    bus_->postToDriver(node_id_,
                       [](stablecops::lely::MotorDriver& motor) { motor.requestFaultReset(); });
}

void MotorDrive::enableOperation(bool hold_position) {
    bus_->invokeOnDriver(node_id_, [hold_position](stablecops::lely::MotorDriver& motor) {
        motor.requestEnableOperation(hold_position);
    });
}

void MotorDrive::setOperationMode(ds402::OperationMode mode) {
    bus_->invokeOnDriver(node_id_, [mode](stablecops::lely::MotorDriver& motor) {
        motor.requestOperationMode(mode);
    });
}

int64_t MotorDrive::readObject(uint16_t index, uint8_t subindex, ObjectDataType type) const {
    int64_t value = 0;
    bus_->invokeOnDriver(node_id_, [&](stablecops::lely::MotorDriver& motor) {
        switch (type) {
            case ObjectDataType::U8:
                value = motor.readU8(index, subindex);
                break;
            case ObjectDataType::I8:
                value = static_cast<int>(static_cast<int8_t>(motor.readU8(index, subindex)));
                break;
            case ObjectDataType::U16:
                value = motor.readU16(index, subindex);
                break;
            case ObjectDataType::I16:
                value = static_cast<int16_t>(motor.readU16(index, subindex));
                break;
            case ObjectDataType::U32:
                value = motor.readU32(index, subindex);
                break;
            case ObjectDataType::I32:
                value = motor.readI32(index, subindex);
                break;
        }
    });
    return value;
}

void MotorDrive::writeObject(uint16_t index, uint8_t subindex, ObjectDataType type, int64_t value) {
    bus_->invokeOnDriver(node_id_,
                         [index, subindex, type, value](stablecops::lely::MotorDriver& motor) {
                             switch (type) {
                                 case ObjectDataType::U8:
                                 case ObjectDataType::I8:
                                     motor.writeU8(index, subindex, static_cast<uint8_t>(value));
                                     break;
                                 case ObjectDataType::U16:
                                 case ObjectDataType::I16:
                                     motor.writeU16(index, subindex, static_cast<uint16_t>(value));
                                     break;
                                 case ObjectDataType::U32:
                                     motor.writeU32(index, subindex, static_cast<uint32_t>(value));
                                     break;
                                 case ObjectDataType::I32:
                                     motor.writeI32(index, subindex, static_cast<int32_t>(value));
                                     break;
                             }
                         });
}

void MotorDrive::commandPosition(int32_t counts) {
    bus_->postToDriver(node_id_, [counts](stablecops::lely::MotorDriver& motor) {
        motor.drive().setCspTargetPosition(counts);
    });
}

void MotorDrive::commandVelocity(int32_t units) {
    bus_->postToDriver(node_id_, [units](stablecops::lely::MotorDriver& motor) {
        motor.drive().setCsvTargetVelocity(units);
    });
}

void MotorDrive::commandTorque(int16_t units) {
    bus_->postToDriver(node_id_, [units](stablecops::lely::MotorDriver& motor) {
        motor.drive().setCstTargetTorque(units);
    });
}

void MotorDrive::moveToPosition(int32_t counts, bool relative) {
    bus_->postToDriver(node_id_, [counts, relative](stablecops::lely::MotorDriver& motor) {
        motor.requestProfileMove(counts, relative);
    });
}

void MotorDrive::startHoming(const ds402::HomingConfig& config) {
    bus_->invokeOnDriver(node_id_, [&config](stablecops::lely::MotorDriver& motor) {
        motor.requestHoming(config);
    });
}

ds402::HomingPhase MotorDrive::homingPhase() const {
    ds402::HomingPhase phase = ds402::HomingPhase::Idle;
    bus_->invokeOnDriver(node_id_, [&phase](stablecops::lely::MotorDriver& motor) {
        phase = motor.homingPhase();
    });
    return phase;
}

ds402::HomingResult MotorDrive::homingResult() const {
    ds402::HomingResult result;
    bus_->invokeOnDriver(node_id_, [&result](stablecops::lely::MotorDriver& motor) {
        result = motor.homingResult();
    });
    return result;
}

}  // namespace stablecops::app
