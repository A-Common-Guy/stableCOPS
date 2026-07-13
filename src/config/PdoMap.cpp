#include "stablecops/config/PdoMap.hpp"

#include <fstream>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

namespace stablecops::config {

namespace {

using nlohmann::json;

constexpr uint32_t kCobIdBaseMask = 0x7FFU;
constexpr uint32_t kCobIdNodeMask = 0x7FU;
constexpr uint32_t kCobIdFlagMask = ~kCobIdBaseMask;

struct CobIdRebase {
    uint8_t from_node_id{0};
    uint8_t to_node_id{0};
};

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
        return static_cast<uint32_t>(std::stoul(value.get<std::string>(), nullptr, 0));
    }
    throw std::runtime_error("PDO map: expected a numeric or hex-string value");
}

PdoMappedObject parseEntry(const json& entry) {
    PdoMappedObject object;
    object.index = static_cast<uint16_t>(parseUnsigned(entry.at("index")));
    object.subindex = static_cast<uint8_t>(parseUnsigned(entry.at("subindex")));
    object.bit_length = static_cast<uint8_t>(parseUnsigned(entry.at("bit_length")));
    if (entry.contains("name") && entry.at("name").is_string()) {
        object.name = entry.at("name").get<std::string>();
    }
    return object;
}

PdoChannel parseChannel(const json& channel) {
    PdoChannel result;
    result.comm_index = static_cast<uint16_t>(parseUnsigned(channel.at("communication_index")));
    result.map_index = static_cast<uint16_t>(parseUnsigned(channel.at("mapping_index")));
    result.cob_id = parseUnsigned(channel.at("cob_id"));
    result.transmission_type = static_cast<uint8_t>(parseUnsigned(channel.at("transmission_type")));
    if (channel.contains("entries")) {
        for (const auto& entry : channel.at("entries")) {
            result.entries.push_back(parseEntry(entry));
        }
    }
    return result;
}

PdoChannel parseChannelForNode(const json& channel, uint8_t node_id) {
    PdoChannel result = parseChannel(channel);
    if (channel.value("cob_id_node_relative", false) && channel.contains("cob_id_offset")) {
        result.cob_id = parseUnsigned(channel.at("cob_id_offset")) + node_id;
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

std::vector<PdoChannel> parseChannelsForNode(const json& mappings, const char* key,
                                             uint8_t node_id) {
    std::vector<PdoChannel> channels;
    if (!mappings.contains(key)) {
        return channels;
    }
    for (const auto& channel : mappings.at(key)) {
        channels.push_back(parseChannelForNode(channel, node_id));
    }
    return channels;
}

bool hasNodeRelativeCobIdMetadata(const json& mappings) {
    for (const char* key : {"rpdo", "tpdo"}) {
        if (!mappings.contains(key)) {
            continue;
        }
        for (const auto& channel : mappings.at(key)) {
            if (channel.value("cob_id_node_relative", false)) {
                return true;
            }
        }
    }
    return false;
}

uint8_t parseRepresentativeNodeId(const json& document) {
    if (document.contains("node_id")) {
        return static_cast<uint8_t>(parseUnsigned(document.at("node_id")));
    }
    if (document.contains("node_ids") && document.at("node_ids").is_array() &&
        !document.at("node_ids").empty()) {
        return static_cast<uint8_t>(parseUnsigned(document.at("node_ids").front()));
    }
    throw std::runtime_error("PDO map: summary has no representative node id");
}

uint32_t rebaseNodeRelativeCobId(uint32_t cob_id, CobIdRebase rebase) {
    if (rebase.from_node_id == rebase.to_node_id) {
        return cob_id;
    }

    const uint32_t base_cob_id = cob_id & kCobIdBaseMask;
    if ((base_cob_id & kCobIdNodeMask) != rebase.from_node_id) {
        return cob_id;
    }

    const uint32_t shifted_base =
        base_cob_id - static_cast<uint32_t>(rebase.from_node_id) + rebase.to_node_id;
    if (shifted_base > kCobIdBaseMask) {
        throw std::runtime_error("PDO map: rebased COB-ID exceeds 11-bit range");
    }

    return (cob_id & kCobIdFlagMask) | shifted_base;
}

void rebaseNodeRelativeCobIds(PdoMap& map, CobIdRebase rebase) {
    for (auto* channels : {&map.rpdo, &map.tpdo}) {
        for (auto& channel : *channels) {
            channel.cob_id = rebaseNodeRelativeCobId(channel.cob_id, rebase);
        }
    }
}

json loadSummaryDocument(const std::string& summary_path) {
    std::ifstream stream(summary_path);
    if (!stream) {
        throw std::runtime_error("PDO map: cannot open summary file '" + summary_path + "'");
    }
    try {
        json document;
        stream >> document;
        return document;
    } catch (const json::exception& exception) {
        throw std::runtime_error("PDO map: failed to parse '" + summary_path +
                                 "': " + exception.what());
    }
}

const json& requirePdoMappings(const json& document, const std::string& summary_path) {
    if (!document.contains("pdo_mappings")) {
        throw std::runtime_error("PDO map: '" + summary_path + "' has no 'pdo_mappings' section");
    }
    return document.at("pdo_mappings");
}

}  // namespace

PdoMap loadPdoMapFromSummary(const std::string& summary_path) {
    const auto document = loadSummaryDocument(summary_path);
    const auto& mappings = requirePdoMappings(document, summary_path);
    PdoMap map;
    map.rpdo = parseChannels(mappings, "rpdo");
    map.tpdo = parseChannels(mappings, "tpdo");
    return map;
}

PdoMap loadPdoMapFromSummary(const std::string& summary_path, uint8_t node_id) {
    const auto document = loadSummaryDocument(summary_path);
    const auto& mappings = requirePdoMappings(document, summary_path);
    PdoMap map;
    if (hasNodeRelativeCobIdMetadata(mappings)) {
        map.rpdo = parseChannelsForNode(mappings, "rpdo", node_id);
        map.tpdo = parseChannelsForNode(mappings, "tpdo", node_id);
    } else {
        // Older summaries carry no per-node metadata: shift the representative
        // node's COB-IDs to this node id instead.
        map.rpdo = parseChannels(mappings, "rpdo");
        map.tpdo = parseChannels(mappings, "tpdo");
        rebaseNodeRelativeCobIds(map, CobIdRebase{parseRepresentativeNodeId(document), node_id});
    }
    return map;
}

std::vector<uint8_t> loadNodeIdsFromSummary(const std::string& summary_path) {
    const auto document = loadSummaryDocument(summary_path);
    std::vector<uint8_t> node_ids;
    if (document.contains("node_ids") && document.at("node_ids").is_array()) {
        for (const auto& value : document.at("node_ids")) {
            node_ids.push_back(static_cast<uint8_t>(parseUnsigned(value)));
        }
    } else if (document.contains("node_id")) {
        node_ids.push_back(static_cast<uint8_t>(parseUnsigned(document.at("node_id"))));
    }
    return node_ids;
}

}  // namespace stablecops::config
