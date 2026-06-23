#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace stablecops::config {

// One object mapped into a PDO, matching an entry in the generated
// summary.json "pdo_mappings" section.
struct PdoMappedObject {
    uint16_t index{0};
    uint8_t subindex{0};
    uint8_t bit_length{0};
    std::string name;
};

// A single PDO communication channel (one RxPDO or TxPDO of the drive).
struct PdoChannel {
    uint16_t comm_index{0};  // 0x14xx (RxPDO) / 0x18xx (TxPDO)
    uint16_t map_index{0};   // 0x16xx (RxPDO) / 0x1Axx (TxPDO)
    uint32_t cob_id{0};      // raw COB-ID, including the valid (bit 31) and
                             // RTR (bit 30) flags as stored in the EDS/summary
    uint8_t transmission_type{0};
    std::vector<PdoMappedObject> entries;

    // A channel is active (enabled) when its COB-ID valid bit (bit 31) is clear.
    bool active() const { return (cob_id & 0x80000000u) == 0; }

    // The 11-bit CAN identifier, stripped of the valid/RTR flags.
    uint16_t baseCobId() const { return static_cast<uint16_t>(cob_id & 0x7FFu); }
};

// Drive-side PDO layout, named from the drive's perspective:
//   - rpdo[] are the drive's RxPDOs (the master transmits these to the drive),
//   - tpdo[] are the drive's TxPDOs (the master receives these from the drive).
// This mirrors the object indices reported in summary.json so the runtime and
// the generated dcf/master.dcf stay driven by the same source.
struct PdoMap {
    std::vector<PdoChannel> rpdo;
    std::vector<PdoChannel> tpdo;
};

// Load the PDO layout from a generated summary.json. Throws std::runtime_error
// if the file cannot be opened or does not contain a "pdo_mappings" section.
PdoMap loadPdoMapFromSummary(const std::string& summary_path);

}  // namespace stablecops::config
