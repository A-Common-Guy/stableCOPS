#include "stablecops/config/PdoMap.hpp"

#include <fstream>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

namespace stablecops::config {

namespace {

using nlohmann::json;

// summary.json stores object/PDO indices as hex strings ("0x1400") and COB-IDs
// as plain integers. Accept either form defensively.
uint32_t parseUnsigned(const json& value) {
    if (value.is_number_unsigned()) {
        return value.get<uint32_t>();
    }
    if (value.is_number_integer()) {
        return static_cast<uint32_t>(value.get<int64_t>());
    }
    if (value.is_string()) {
        return static_cast<uint32_t>(
            std::stoul(value.get<std::string>(), nullptr, 0));
    }
    throw std::runtime_error("PDO map: expected a numeric or hex-string value");
}

PdoMappedObject parseEntry(const json& entry) {
    PdoMappedObject object;
    object.index = static_cast<uint16_t>(parseUnsigned(entry.at("index")));
    object.subindex = static_cast<uint8_t>(parseUnsigned(entry.at("subindex")));
    object.bit_length =
        static_cast<uint8_t>(parseUnsigned(entry.at("bit_length")));
    if (entry.contains("name") && entry.at("name").is_string()) {
        object.name = entry.at("name").get<std::string>();
    }
    return object;
}

PdoChannel parseChannel(const json& channel) {
    PdoChannel result;
    result.comm_index =
        static_cast<uint16_t>(parseUnsigned(channel.at("communication_index")));
    result.map_index =
        static_cast<uint16_t>(parseUnsigned(channel.at("mapping_index")));
    result.cob_id = parseUnsigned(channel.at("cob_id"));
    result.transmission_type =
        static_cast<uint8_t>(parseUnsigned(channel.at("transmission_type")));
    if (channel.contains("entries")) {
        for (const auto& entry : channel.at("entries")) {
            result.entries.push_back(parseEntry(entry));
        }
    }
    return result;
}

std::vector<PdoChannel> parseChannels(const json& mappings, const char* key) {
    std::vector<PdoChannel> channels;
    if (!mappings.contains(key)) {
        return channels;
    }
    for (const auto& channel : mappings.at(key)) {
        channels.push_back(parseChannel(channel));
    }
    return channels;
}

}  // namespace

PdoMap loadPdoMapFromSummary(const std::string& summary_path) {
    std::ifstream stream(summary_path);
    if (!stream) {
        throw std::runtime_error("PDO map: cannot open summary file '" +
                                 summary_path + "'");
    }

    json document;
    try {
        stream >> document;
    } catch (const json::exception& exception) {
        throw std::runtime_error("PDO map: failed to parse '" + summary_path +
                                 "': " + exception.what());
    }

    if (!document.contains("pdo_mappings")) {
        throw std::runtime_error("PDO map: '" + summary_path +
                                 "' has no 'pdo_mappings' section");
    }

    const auto& mappings = document.at("pdo_mappings");
    PdoMap map;
    map.rpdo = parseChannels(mappings, "rpdo");
    map.tpdo = parseChannels(mappings, "tpdo");
    return map;
}

}  // namespace stablecops::config
