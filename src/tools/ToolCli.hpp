#pragma once

// CLI parsing helpers shared by the stableCOPS command-line tools.

#include <cstdint>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "stablecops/ds402/State.hpp"

namespace stablecops::tools {

inline std::optional<ds402::OperationMode> parseOperationMode(const std::string& name) {
    using ds402::OperationMode;
    if (name == "csp") {
        return OperationMode::CyclicSynchronousPosition;
    }
    if (name == "csv") {
        return OperationMode::CyclicSynchronousVelocity;
    }
    if (name == "cst") {
        return OperationMode::CyclicSynchronousTorque;
    }
    if (name == "pp") {
        return OperationMode::ProfilePosition;
    }
    if (name == "pv") {
        return OperationMode::ProfileVelocity;
    }
    if (name == "pt") {
        return OperationMode::ProfileTorque;
    }
    return std::nullopt;
}

// Strict unsigned parse: accepts decimal or 0x-prefixed hex, requires the whole
// token to be consumed, and rejects values above max_value.
inline uint64_t parseUnsignedText(const std::string& text, uint64_t max_value) {
    std::size_t consumed = 0;
    const auto value = std::stoull(text, &consumed, 0);
    if (consumed != text.size() || value > max_value) {
        throw std::invalid_argument("numeric value out of range: " + text);
    }
    return value;
}

// Comma-separated CANopen node id list ("1,2,3"), each id in 1..127.
inline std::optional<std::vector<uint8_t>> parseNodeList(const std::string& spec) {
    std::vector<uint8_t> nodes;
    std::stringstream stream(spec);
    std::string token;
    while (std::getline(stream, token, ',')) {
        if (token.empty()) {
            continue;
        }
        try {
            const auto value = parseUnsignedText(token, 127);
            if (value == 0) {
                return std::nullopt;
            }
            nodes.push_back(static_cast<uint8_t>(value));
        } catch (...) {
            return std::nullopt;
        }
    }
    if (nodes.empty()) {
        return std::nullopt;
    }
    return nodes;
}

}  // namespace stablecops::tools
