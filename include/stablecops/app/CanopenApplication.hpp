#pragma once

#include <memory>

#include "stablecops/app/MotorConfig.hpp"
#include "stablecops/lely/MotorDriver.hpp"

namespace stablecops::app {

class CanopenApplication {
public:
    explicit CanopenApplication(const MotorConfig& config);
    ~CanopenApplication();

    CanopenApplication(const CanopenApplication&) = delete;
    CanopenApplication& operator=(const CanopenApplication&) = delete;

    stablecops::lely::MotorDriver& motor();

    void resetMaster();
    void run();
    void stop();

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace stablecops::app
