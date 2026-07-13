#include "stablecops/ds402/Diagnostics.hpp"

#include <iomanip>
#include <sstream>

namespace stablecops::ds402 {

namespace {

std::string toHex(uint32_t value, int width) {
    std::ostringstream stream;
    stream << "0x" << std::uppercase << std::hex << std::setw(width) << std::setfill('0') << value;
    return stream.str();
}

}  // namespace

std::string describeErrorRegister(uint8_t error_register) {
    if (error_register == 0) {
        return "none";
    }
    // Bit meanings per manual 0x1001 (CiA301 error register).
    static constexpr const char* kBits[8] = {
        "generic",         // bit0
        "current",         // bit1
        "voltage",         // bit2
        "temperature",     // bit3
        "communication",   // bit4
        "device-profile",  // bit5
        "reserved",        // bit6
        "manufacturer",    // bit7
    };
    std::string out;
    for (int bit = 0; bit < 8; ++bit) {
        if ((error_register & (1U << bit)) != 0) {
            if (!out.empty()) {
                out += ", ";
            }
            out += kBits[bit];
        }
    }
    return out;
}

std::string describeDeviceFault(uint16_t fault_code) {
    if (fault_code == 0) {
        return "no fault";
    }

    struct Entry {
        uint16_t code;
        const char* name;
        const char* type;
    };
    // Manual Table 4-2 "Device Error Codes": fieldbus fault code (0x603F) ->
    // fault name and category. The EMCY emergency error code shares this space.
    static constexpr Entry kEntries[] = {
        {0x2320, "IPM Bottom Leg Short Circuit", "Drive Fault"},
        {0x2330, "UVW to Ground Short Circuit", "Drive Fault"},
        {0x5210, "Current Sensor Error", "Drive Fault"},
        {0x5211, "Current Sensor Error at Enable", "Drive Fault"},
        {0x6321, "EEPROM Write Failure", "Drive Fault"},
        {0x2222, "Hardware Over-Current", "Drive Fault"},
        {0x2221, "Software Over-Current", "Drive Fault"},
        {0x4210, "Motor Over-Temperature", "Drive Fault"},
        {0x4310, "Drive MOSFET Over-Temperature", "Drive Fault"},
        {0x2350, "Drive Overload", "Drive Fault"},
        {0x7121, "Motor Stall", "Drive Fault"},
        {0x2351, "Motor Overload", "Drive Fault"},
        {0x7122, "Motor Phase Loss", "Drive Fault"},
        {0x3210, "DC Bus Over-Voltage", "Drive Fault"},
        {0x3220, "DC Bus Under-Voltage", "Drive Fault"},
        {0xFF04, "Internal Error", "Drive Fault"},
        {0xFF10, "Drive Parameter Initialization Error", "Drive Fault"},
        {0x6322, "MCU Parameter Verification Error", "Drive Fault"},
        {0x6323, "EEPROM Version Error", "Drive Fault"},
        {0x6320, "Parameter EEPROM Error", "Drive Fault"},
        {0xFF24, "Please Save Parameters and Power Cycle", "Drive Fault"},
        {0xFF03, "Please Perform Configuration", "Drive Fault"},
        {0x7510, "Primary Encoder Communication Loss", "Motor/Encoder Fault"},
        {0x7521, "Encoder Model Initialization Failure", "Motor/Encoder Fault"},
        {0x7515, "Primary Encoder CRC Error", "Motor/Encoder Fault"},
        {0x7511, "Secondary Encoder Communication Loss", "Motor/Encoder Fault"},
        {0x7513, "Secondary Encoder CRC Error", "Motor/Encoder Fault"},
        {0x7514, "Dual Encoder Position Deviation Excessive", "Motor/Encoder Fault"},
        {0xFF11, "Motor Parameter Loading Error", "Motor/Encoder Fault"},
        {0x8611, "Position Error Excess", "Application Fault"},
        {0x8400, "Velocity Error Excess", "Application Fault"},
        {0x8401, "Over-Speed Fault", "Application Fault"},
        {0xFF35, "Electronic Gear Ratio Calculation Overflow", "Application Fault"},
        {0xFF36, "Electronic Gear Ratio Out of Range", "Application Fault"},
        // Documented only in the manual's per-fault diagnostics (4.2), not in
        // Table 4-2 itself.
        {0xFF34, "Command Interpolation Cycle Not Supported", "Application Fault"},
        {0xFF33, "PDO Configuration Exceeds Quantity Limit", "Application Fault"},
    };

    for (const auto& entry : kEntries) {
        if (entry.code == fault_code) {
            return std::string(entry.name) + " [" + entry.type + "]";
        }
    }
    return "unknown device fault (" + toHex(fault_code, 4) + ")";
}

std::string describeAbortCode(uint32_t abort_code) {
    struct Entry {
        uint32_t code;
        const char* text;
    };
    // Manual Table 4-1 "CANopen Communication Error Codes" (SDO abort codes).
    static constexpr Entry kEntries[] = {
        {0x00000000, "no error"},
        {0x05030000, "toggle bit not alternated"},
        {0x05040000, "SDO protocol timeout"},
        {0x05040001, "invalid or unknown command specifier"},
        {0x05040004, "CRC error"},
        {0x06010000, "unsupported access to an object"},
        {0x06010001, "attempt to read a write-only object"},
        {0x06010002, "attempt to write a read-only object"},
        {0x06010003, "sub-index must be 0 to be writable"},
        {0x06010004, "object is not SDO-accessible (PDO mapped only)"},
        {0x06020000, "object does not exist in the object dictionary"},
        {0x06040041, "object cannot be mapped to a PDO"},
        {0x06040042, "mapped objects exceed the PDO length"},
        {0x06040043, "general parameter incompatibility"},
        {0x06040047, "general internal incompatibility in the device"},
        {0x06060000, "access failed due to a hardware error"},
        {0x06070010, "data type / service parameter length mismatch"},
        {0x06070012, "service parameter too long"},
        {0x06070013, "service parameter too short"},
        {0x06090011, "sub-index does not exist"},
        {0x06090030, "parameter value out of range"},
        {0x08000000, "unspecified general error"},
        {0x08000020, "data cannot be transferred or stored to the application"},
        {0x08000022, "device state prevents data transfer or storage"},
        {0x0F00FFBE, "password error"},
        {0x0F00FFBF, "illegal (non-existent) command code"},
        {0x0F00FFC0, "device is in an invalid NMT state for the request"},
    };

    for (const auto& entry : kEntries) {
        if (entry.code == abort_code) {
            return entry.text;
        }
    }
    return "unknown abort code (" + toHex(abort_code, 8) + ")";
}

}  // namespace stablecops::ds402
