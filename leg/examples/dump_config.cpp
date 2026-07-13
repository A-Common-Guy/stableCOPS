// dump_config -- read the configuration parameters of both drives in a leg over
// SDO and print them side by side (optionally to CSV), with an engineering-unit
// conversion so the raw registers can be compared against the datasheet.
//
// This is a READ-ONLY tool. It boots the shared CAN bus without energising the
// motors or reconfiguring their PDOs, then performs blocking SDO uploads for a
// curated set of identity, DS402, feedback/scaling, tuning (Kt, Kp/Ki/Kd),
// profile, limit/saturation, comms and vendor objects on each node. Each object
// is read independently, so a single unsupported object just shows FAILED
// instead of aborting the dump.
//
// UNITS. The EYou-RP drive mixes unit systems: vendor objects (0x2xxx) use
// scaled engineering units while CiA402 motion objects (0x60xx) use raw encoder
// counts. The conversions below were verified against the RP90L datasheet
// (Kt=0.16 Nm/Arms, peak current 49 Arms, peak torque ~120 Nm, 48 V, rotor
// inertia 9.33e-5 kg*m^2). They depend on the machine constants passed via
// --gear-ratio / --rated-torque / --rated-current / --counts-per-rev, which
// default to the RP90L.
//
// Run:
//   sudo ./canup.sh
//   build/examples/dump_config --can can0
//   build/examples/dump_config --can can0 --csv leg/output/motor_config.csv

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "stablecops/app/MotorConfig.hpp"
#include "stablecops/app/MotorDrive.hpp"

namespace {

using stablecops::app::ObjectDataType;

// Engineering-unit conversion applied to a raw register value. See the RP90L
// verification notes in the file header for how each factor was derived.
enum class Unit : std::uint8_t {
    None,                  // raw only (indices, flags, counts, dimensionless)
    TorquePermilleRated,   // Nm     = raw/1000 * rated_torque_nm
    CurrentPermilleRated,  // Arms   = raw/1000 * rated_current_arms
    CurrentMilliAmpPeak,   // Arms   = raw / (sqrt(2)*1000)
    MotorTorqueMilliNm,    // Nm     = raw/1000            (vendor 0x2007, motor shaft)
    TorqueMilliNm,         // Nm     = raw/1000            (CiA402 0x6076 rated torque)
    CurrentMilliAmp,       // A      = raw/1000            (CiA402 0x6075 rated current)
    PositionCounts,        // deg    = raw/counts_per_rev * 360
    VelocityCountsPerSec,  // rpm    = raw/counts_per_rev * 60
    AccelCountsPerSec2,    // rpm/s  = raw/counts_per_rev * 60
    MotorRpm,              // rad/s  = raw * 2pi/60        (value already rpm)
    VoltageMilliVolt,      // V      = raw/1000
    InertiaNano,           // kg*m^2 = raw * 1e-9
    RatioMilli,            // x1     = raw/1000
    TorqueConstant,        // Nm/Arms= raw * sqrt(2) * 1e-5
    GainMilli,             // x1     = raw/1000           (MIT-mode gains)
};

// Machine constants used by the conversions (RP90L defaults).
struct MachineConstants {
    double gear_ratio = 0.0;          // 0 = from the motor profile (runtime.gear_ratio)
    double rated_torque_nm = 35.0;    // output-shaft rated torque
    double rated_current_arms = 15.0; // phase rated current (rms)
    double counts_per_rev = 0.0;      // 0 = from the motor profile (runtime.counts_per_rev)
};

// One configuration object to read. `section` starts a new titled group when it
// is non-empty; `hex` also renders the value in hex (handy for identity codes,
// COB-IDs and bitfields); `unit` selects the engineering conversion.
struct Param {
    const char* section;
    const char* name;
    uint16_t index;
    uint8_t subindex;
    ObjectDataType type;
    bool hex;
    Unit unit;
};

// Curated parameter set. The standard CiA402 + PDO objects mirror
// MotorDriver::inspectNode(); the vendor motor-nameplate, control-loop tuning
// (Kt, Kp/Ki/Kd, feedforward) and limit/saturation objects come from the
// EYou-RP EDS (eds/EDS files/EuServo_RP_EDS_（RP）V1.4.eds). EDS DataType maps
// to ObjectDataType as: 0x02->I8, 0x03->I16, 0x04->I32, 0x05->U8, 0x06->U16,
// 0x07->U32.
const std::vector<Param> kParams = {
    {"identity", "vendor id", 0x1018, 0x01, ObjectDataType::U32, true, Unit::None},
    {"", "product code", 0x1018, 0x02, ObjectDataType::U32, true, Unit::None},
    {"", "revision", 0x1018, 0x03, ObjectDataType::U32, true, Unit::None},
    {"", "serial number", 0x1018, 0x04, ObjectDataType::U32, true, Unit::None},

    {"motor nameplate", "motor power", 0x2005, 0x00, ObjectDataType::U32, false, Unit::None},
    {"", "motor rated speed", 0x2006, 0x00, ObjectDataType::U32, false, Unit::MotorRpm},
    {"", "motor rated torque", 0x2007, 0x00, ObjectDataType::I32, false, Unit::MotorTorqueMilliNm},
    {"", "motor rated current", 0x2008, 0x00, ObjectDataType::I32, false, Unit::CurrentMilliAmpPeak},
    {"", "motor max speed", 0x2009, 0x00, ObjectDataType::U32, false, Unit::MotorRpm},
    {"", "motor max torque", 0x200A, 0x00, ObjectDataType::I32, false, Unit::MotorTorqueMilliNm},
    {"", "motor peak current", 0x200B, 0x00, ObjectDataType::I32, false, Unit::CurrentMilliAmpPeak},
    {"", "motor inertia", 0x200C, 0x00, ObjectDataType::I32, false, Unit::InertiaNano},
    {"", "load/motor inertia ratio", 0x200F, 0x00, ObjectDataType::I32, false, Unit::RatioMilli},
    {"", "motor poles", 0x2011, 0x00, ObjectDataType::U16, false, Unit::None},
    {"", "motor inductance", 0x2012, 0x00, ObjectDataType::I32, false, Unit::None},
    {"", "motor resistance", 0x2013, 0x00, ObjectDataType::I32, false, Unit::None},
    {"", "back-EMF coeff (Ke)", 0x2014, 0x00, ObjectDataType::I32, false, Unit::None},
    {"", "torque constant (Kt)", 0x2015, 0x00, ObjectDataType::I32, false, Unit::TorqueConstant},
    {"", "motor rated voltage", 0x2018, 0x00, ObjectDataType::I32, false, Unit::VoltageMilliVolt},

    {"DS402 status", "statusword", 0x6041, 0x00, ObjectDataType::U16, true, Unit::None},
    {"", "error code", 0x603F, 0x00, ObjectDataType::U16, true, Unit::None},
    {"", "error register", 0x1001, 0x00, ObjectDataType::U8, true, Unit::None},
    {"", "supported modes", 0x6502, 0x00, ObjectDataType::U32, true, Unit::None},
    {"", "commanded mode", 0x6060, 0x00, ObjectDataType::I8, false, Unit::None},
    {"", "displayed mode", 0x6061, 0x00, ObjectDataType::I8, false, Unit::None},
    {"", "position actual", 0x6064, 0x00, ObjectDataType::I32, false, Unit::PositionCounts},
    {"", "velocity actual", 0x606C, 0x00, ObjectDataType::I32, false, Unit::VelocityCountsPerSec},
    {"", "torque actual", 0x6077, 0x00, ObjectDataType::I16, false, Unit::TorquePermilleRated},

    // The per-mille base for ALL torque/current commands. Torque objects
    // (0x6071/0x6072/0x6074/0x6077) are given in 1/1000 of rated torque
    // (0x6076); current objects (0x6073) in 1/1000 of rated current (0x6075).
    // Read these to confirm what --rated-torque / --rated-current should be.
    {"torque/current reference", "rated torque (per-mille base)", 0x6076, 0x00, ObjectDataType::U32, false, Unit::TorqueMilliNm},
    {"", "rated current (per-mille base)", 0x6075, 0x00, ObjectDataType::U32, false, Unit::CurrentMilliAmp},
    {"", "target torque (commanded)", 0x6071, 0x00, ObjectDataType::I16, false, Unit::TorquePermilleRated},
    {"", "torque demand", 0x6074, 0x00, ObjectDataType::I16, false, Unit::TorquePermilleRated},

    {"position feedback & scaling", "position demand", 0x6062, 0x00, ObjectDataType::I32, false, Unit::PositionCounts},
    {"", "position feedback", 0x6063, 0x00, ObjectDataType::I32, false, Unit::PositionCounts},
    {"", "encoder single-turn (primary)", 0x276F, 0x00, ObjectDataType::I32, false, Unit::None},
    {"", "encoder single-turn (3)", 0x2772, 0x00, ObjectDataType::I32, false, Unit::None},
    {"", "second feedback direction", 0x2219, 0x00, ObjectDataType::U8, false, Unit::None},
    {"", "second feedback mode", 0x221A, 0x00, ObjectDataType::U8, false, Unit::None},
    {"", "encoder increments", 0x608F, 0x01, ObjectDataType::U32, false, Unit::None},
    {"", "motor revolutions (resolution)", 0x608F, 0x02, ObjectDataType::U32, false, Unit::None},
    {"", "gear ratio: motor revolutions", 0x6091, 0x01, ObjectDataType::U32, false, Unit::None},
    {"", "gear ratio: shaft revolutions", 0x6091, 0x02, ObjectDataType::U32, false, Unit::None},

    {"position loop (tuning)", "position loop gain (Kp)", 0x222A, 0x00, ObjectDataType::I32, false, Unit::None},
    {"", "position vel feedforward gain", 0x222B, 0x00, ObjectDataType::I32, false, Unit::None},
    {"", "position vel FF filter freq", 0x222C, 0x00, ObjectDataType::U32, false, Unit::None},
    {"", "torque feedforward gain", 0x222D, 0x00, ObjectDataType::I32, false, Unit::None},
    {"", "MIT loop Kp", 0x2226, 0x00, ObjectDataType::I32, false, Unit::GainMilli},
    {"", "MIT loop Kd", 0x2227, 0x00, ObjectDataType::I32, false, Unit::GainMilli},
    {"", "rigidity level", 0x2228, 0x00, ObjectDataType::U16, false, Unit::None},

    {"velocity loop (tuning)", "velocity loop bandwidth", 0x2303, 0x00, ObjectDataType::I32, false, Unit::None},
    {"", "velocity proportional gain (Kp)", 0x2304, 0x00, ObjectDataType::I32, false, Unit::None},
    {"", "velocity integral time const", 0x2305, 0x00, ObjectDataType::I32, false, Unit::None},
    {"", "velocity integral gain (Ki)", 0x2306, 0x00, ObjectDataType::I32, false, Unit::None},
    {"", "velocity feedforward gain", 0x2307, 0x00, ObjectDataType::I32, false, Unit::None},
    {"", "velocity->current FF gain", 0x2308, 0x00, ObjectDataType::I32, false, Unit::None},

    {"current loop (tuning)", "current loop bandwidth", 0x2410, 0x00, ObjectDataType::I32, false, Unit::None},
    {"", "current proportional gain (Kp)", 0x2412, 0x00, ObjectDataType::I32, false, Unit::None},
    {"", "current integral gain (Ki)", 0x2413, 0x00, ObjectDataType::I32, false, Unit::None},
    {"", "current anti-saturation gain", 0x2415, 0x00, ObjectDataType::I32, false, Unit::None},
    {"", "current voltage FF coeff", 0x241B, 0x00, ObjectDataType::I32, false, Unit::None},

    {"profile", "profile velocity", 0x6081, 0x00, ObjectDataType::U32, false, Unit::VelocityCountsPerSec},
    {"", "profile acceleration", 0x6083, 0x00, ObjectDataType::U32, false, Unit::AccelCountsPerSec2},
    {"", "profile deceleration", 0x6084, 0x00, ObjectDataType::U32, false, Unit::AccelCountsPerSec2},
    {"", "torque slope", 0x6087, 0x00, ObjectDataType::U32, false, Unit::None},
    {"", "max profile velocity", 0x607F, 0x00, ObjectDataType::U32, false, Unit::VelocityCountsPerSec},

    {"limits / saturation", "max torque", 0x6072, 0x00, ObjectDataType::I16, false, Unit::TorquePermilleRated},
    {"", "max current", 0x6073, 0x00, ObjectDataType::U16, false, Unit::CurrentPermilleRated},
    {"", "positive torque limit", 0x60E0, 0x00, ObjectDataType::U16, false, Unit::TorquePermilleRated},
    {"", "negative torque limit", 0x60E1, 0x00, ObjectDataType::U16, false, Unit::TorquePermilleRated},
    {"", "maximum current", 0x2408, 0x00, ObjectDataType::I32, false, Unit::CurrentMilliAmpPeak},
    {"", "max positive current", 0x2409, 0x00, ObjectDataType::I32, false, Unit::CurrentMilliAmpPeak},
    {"", "max negative current", 0x240A, 0x00, ObjectDataType::I32, false, Unit::CurrentMilliAmpPeak},
    {"", "torque-mode velocity limit", 0x240D, 0x00, ObjectDataType::I32, false, Unit::VelocityCountsPerSec},
    {"", "positive current limit (actual)", 0x2735, 0x00, ObjectDataType::I32, false, Unit::CurrentMilliAmpPeak},
    {"", "negative current limit (actual)", 0x2736, 0x00, ObjectDataType::I32, false, Unit::CurrentMilliAmpPeak},
    {"", "maximum velocity", 0x2301, 0x00, ObjectDataType::I32, false, Unit::MotorRpm},
    {"", "over velocity threshold", 0x250B, 0x00, ObjectDataType::U32, false, Unit::MotorRpm},
    {"", "stall velocity", 0x2517, 0x00, ObjectDataType::U32, false, Unit::MotorRpm},
    {"", "stall current", 0x2518, 0x00, ObjectDataType::I32, false, Unit::CurrentMilliAmpPeak},
    {"", "position limiting mode", 0x2532, 0x00, ObjectDataType::U8, false, Unit::None},
    {"", "min position limit (607D:01)", 0x607D, 0x01, ObjectDataType::I32, false, Unit::PositionCounts},
    {"", "max position limit (607D:02)", 0x607D, 0x02, ObjectDataType::I32, false, Unit::PositionCounts},
    {"", "sw position limit max (vendor)", 0x2534, 0x00, ObjectDataType::I32, false, Unit::PositionCounts},
    {"", "sw position limit min (vendor)", 0x2535, 0x00, ObjectDataType::I32, false, Unit::PositionCounts},

    {"comms / sync", "producer heartbeat time", 0x1017, 0x00, ObjectDataType::U16, false, Unit::None},
    {"", "SYNC COB-ID", 0x1005, 0x00, ObjectDataType::U32, true, Unit::None},

    {"disable / brake", "disable mode", 0x2103, 0x00, ObjectDataType::U8, false, Unit::None},
    {"", "active disable speed threshold", 0x2104, 0x00, ObjectDataType::U32, false, Unit::None},
    {"", "active disable delay", 0x2105, 0x00, ObjectDataType::U32, false, Unit::None},
    {"", "active disable time", 0x2106, 0x00, ObjectDataType::U16, false, Unit::None},
    {"", "active disable deceleration", 0x2107, 0x00, ObjectDataType::U32, false, Unit::None},
    {"", "active disable deceleration time", 0x2108, 0x00, ObjectDataType::U16, false, Unit::None},
    {"", "quick stop option code", 0x605A, 0x00, ObjectDataType::U16, false, Unit::None},
    {"", "shutdown option code", 0x605B, 0x00, ObjectDataType::U16, false, Unit::None},
    {"", "disable operation option code", 0x605C, 0x00, ObjectDataType::U16, false, Unit::None},
    {"", "quick stop deceleration", 0x6085, 0x00, ObjectDataType::U32, false, Unit::AccelCountsPerSec2},
    {"", "digital outputs (physical)", 0x60FE, 0x01, ObjectDataType::U32, true, Unit::None},
    {"", "digital outputs (mask)", 0x60FE, 0x02, ObjectDataType::U32, true, Unit::None},

    {"vendor runtime monitors", "drive enable status", 0x2706, 0x00, ObjectDataType::U8, false, Unit::None},
    {"", "system state machine state", 0x2709, 0x00, ObjectDataType::U8, false, Unit::None},
    {"", "motor run loop state", 0x270A, 0x00, ObjectDataType::U8, false, Unit::None},
};

int hexDigits(ObjectDataType type) {
    switch (type) {
        case ObjectDataType::U8:
        case ObjectDataType::I8:
            return 2;
        case ObjectDataType::U16:
        case ObjectDataType::I16:
            return 4;
        default:
            return 8;
    }
}

uint32_t hexMask(ObjectDataType type) {
    switch (type) {
        case ObjectDataType::U8:
        case ObjectDataType::I8:
            return 0xFFU;
        case ObjectDataType::U16:
        case ObjectDataType::I16:
            return 0xFFFFU;
        default:
            return 0xFFFFFFFFU;
    }
}

// Human-readable "index:sub" tag, e.g. "0x6091:02".
std::string objectTag(const Param& p) {
    std::ostringstream os;
    os << "0x" << std::hex << std::uppercase << std::setw(4) << std::setfill('0') << p.index << ":"
       << std::setw(2) << std::setfill('0') << static_cast<int>(p.subindex);
    return os.str();
}

// Compact number: trims trailing zeros so "6.67" instead of "6.670000".
std::string trim(double value) {
    std::ostringstream os;
    os << std::fixed << std::setprecision(3) << value;
    std::string s = os.str();
    if (s.find('.') != std::string::npos) {
        s.erase(s.find_last_not_of('0') + 1);
        if (!s.empty() && s.back() == '.') {
            s.pop_back();
        }
    }
    return s;
}

// Convert a raw register value to its engineering representation, or "" when
// the object has no conversion (Unit::None).
std::string engineering(int64_t raw, Unit unit, const MachineConstants& c) {
    const double v = static_cast<double>(raw);
    constexpr double kSqrt2 = 1.4142135623730951;
    switch (unit) {
        case Unit::None:
            return "";
        case Unit::TorquePermilleRated:
            return trim(v / 1000.0 * c.rated_torque_nm) + " Nm";
        case Unit::CurrentPermilleRated:
            return trim(v / 1000.0 * c.rated_current_arms) + " Arms";
        case Unit::CurrentMilliAmpPeak:
            return trim(v / (kSqrt2 * 1000.0)) + " Arms";
        case Unit::MotorTorqueMilliNm:
            return trim(v / 1000.0) + " Nm(motor)";
        case Unit::TorqueMilliNm:
            return trim(v / 1000.0) + " Nm";
        case Unit::CurrentMilliAmp:
            return trim(v / 1000.0) + " A";
        case Unit::PositionCounts:
            return trim(v / c.counts_per_rev * 360.0) + " deg";
        case Unit::VelocityCountsPerSec:
            return trim(v / c.counts_per_rev * 60.0) + " rpm";
        case Unit::AccelCountsPerSec2:
            return trim(v / c.counts_per_rev * 60.0) + " rpm/s";
        case Unit::MotorRpm:
            return trim(v / c.gear_ratio) + " rpm(out)";
        case Unit::VoltageMilliVolt:
            return trim(v / 1000.0) + " V";
        case Unit::InertiaNano:
            return trim(v * 1e-9) + " kg*m^2";
        case Unit::RatioMilli:
            return trim(v / 1000.0);
        case Unit::TorqueConstant:
            return trim(v * kSqrt2 * 1e-5) + " Nm/Arms";
        case Unit::GainMilli:
            return trim(v / 1000.0);
    }
    return "";
}

// Reads one object. On success fills `raw` (decimal string) and `eng`
// (engineering string, may be empty) and returns a display string; on an SDO
// abort returns a FAILED marker.
std::string readValue(const stablecops::app::MotorDrive& drive, const Param& p,
                      const MachineConstants& c, std::string& raw, std::string& eng) {
    try {
        const int64_t value = drive.readObject(p.index, p.subindex, p.type);
        raw = std::to_string(value);
        eng = engineering(value, p.unit, c);
        std::ostringstream os;
        os << value;
        if (p.hex) {
            os << " (0x" << std::hex << std::uppercase << std::setw(hexDigits(p.type))
               << std::setfill('0') << (static_cast<uint32_t>(value) & hexMask(p.type)) << ")";
        }
        return os.str();
    } catch (const std::exception& exception) {
        raw = "";
        eng = "";
        return std::string("FAILED: ") + exception.what();
    }
}

void printUsage(const char* argv0) {
    std::cerr << "usage: " << argv0
              << " [--can iface] [--top-node N] [--bottom-node N] "
                 "[--master-node 127] [--dcf path] [--summary path] "
                 "[--boot-ms N] [--csv path] [--gear-ratio R] [--rated-torque Nm] "
                 "[--rated-current Arms] [--counts-per-rev N]\n";
}

}  // namespace

int main(int argc, char** argv) {
    std::string can_interface = "can0";
    uint8_t top_node = 1;
    uint8_t bottom_node = 2;
    int boot_ms = 1000;
    std::string csv_path;
    MachineConstants constants;
    bool rated_torque_overridden = false;
    bool rated_current_overridden = false;

    stablecops::app::MotorConfig base;  // library defaults: read-only, no cyclic

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto next = [&](const char* name) -> std::string {
            if (i + 1 >= argc) {
                std::cerr << "missing value for " << name << '\n';
                std::exit(EXIT_FAILURE);
            }
            return argv[++i];
        };
        if (arg == "--can") {
            can_interface = next("--can");
        } else if (arg == "--top-node") {
            top_node = static_cast<uint8_t>(std::stoi(next("--top-node")));
        } else if (arg == "--bottom-node") {
            bottom_node = static_cast<uint8_t>(std::stoi(next("--bottom-node")));
        } else if (arg == "--master-node") {
            base.master_node_id = static_cast<uint8_t>(std::stoi(next("--master-node")));
        } else if (arg == "--dcf") {
            base.master_dcf_path = next("--dcf");
        } else if (arg == "--summary") {
            base.summary_path = next("--summary");
        } else if (arg == "--boot-ms") {
            boot_ms = std::stoi(next("--boot-ms"));
        } else if (arg == "--csv") {
            csv_path = next("--csv");
        } else if (arg == "--gear-ratio") {
            constants.gear_ratio = std::stod(next("--gear-ratio"));
        } else if (arg == "--rated-torque") {
            constants.rated_torque_nm = std::stod(next("--rated-torque"));
            rated_torque_overridden = true;
        } else if (arg == "--rated-current") {
            constants.rated_current_arms = std::stod(next("--rated-current"));
            rated_current_overridden = true;
        } else if (arg == "--counts-per-rev") {
            constants.counts_per_rev = std::stod(next("--counts-per-rev"));
        } else {
            printUsage(argv[0]);
            return EXIT_FAILURE;
        }
    }

    base.can_interface = can_interface;

    std::cout << "reading config from " << can_interface << " (top node " << static_cast<int>(top_node)
              << ", bottom node " << static_cast<int>(bottom_node) << ")\n";

    // Construct both drives before starting the shared bus, then boot once.
    stablecops::app::MotorConfig top_config = base;
    top_config.node_id = top_node;
    stablecops::app::MotorConfig bottom_config = base;
    bottom_config.node_id = bottom_node;

    stablecops::app::MotorDrive top(top_config);
    stablecops::app::MotorDrive bottom(bottom_config);

    // Gear ratio / encoder scaling come from the motor profile unless pinned on
    // the CLI; fall back to the RP90L values if the profile omits them so the
    // dump stays readable (the affected columns are display-only conversions).
    if (constants.gear_ratio <= 0.0) {
        constants.gear_ratio = top.config().gear_ratio > 0.0 ? top.config().gear_ratio : 21.913;
    }
    if (constants.counts_per_rev <= 0.0) {
        constants.counts_per_rev = static_cast<double>(top.config().counts_per_rev);
    }
    std::cout << "constants: gear=" << constants.gear_ratio
              << ", rated torque=" << constants.rated_torque_nm << " Nm"
              << ", rated current=" << constants.rated_current_arms << " Arms"
              << ", counts/rev=" << constants.counts_per_rev << '\n';

    try {
        top.start();
    } catch (const std::exception& exception) {
        std::cerr << "CAN start failed: " << exception.what() << '\n';
        return EXIT_FAILURE;
    }

    // No cyclic feedback is configured (read-only), so give the master a moment
    // to finish NMT boot before hammering the nodes with SDO uploads.
    std::this_thread::sleep_for(std::chrono::milliseconds(boot_ms));

    // Ground the per-mille torque/current conversions in what the drive itself
    // reports (CiA402 0x6076 rated torque in mNm, 0x6075 rated current in mA),
    // unless the caller pinned a value on the CLI. This is the authoritative
    // scaling base for every torque/current command, so the decoded Nm/A match
    // exactly what the drive does rather than a datasheet assumption.
    if (!rated_torque_overridden) {
        try {
            const int64_t rated_mNm =
                top.readObject(0x6076, 0x00, ObjectDataType::U32);
            if (rated_mNm > 0) {
                constants.rated_torque_nm = static_cast<double>(rated_mNm) / 1000.0;
                std::cout << "per-mille torque base from drive 0x6076 = " << rated_mNm
                          << " mNm -> rated torque " << constants.rated_torque_nm << " Nm\n";
            }
        } catch (const std::exception& exception) {
            std::cout << "could not read 0x6076 rated torque (" << exception.what()
                      << "); using --rated-torque " << constants.rated_torque_nm << " Nm\n";
        }
    }
    if (!rated_current_overridden) {
        try {
            const int64_t rated_mA = top.readObject(0x6075, 0x00, ObjectDataType::U32);
            if (rated_mA > 0) {
                constants.rated_current_arms = static_cast<double>(rated_mA) / 1000.0;
                std::cout << "per-mille current base from drive 0x6075 = " << rated_mA
                          << " mA -> rated current " << constants.rated_current_arms << " A\n";
            }
        } catch (const std::exception& exception) {
            std::cout << "could not read 0x6075 rated current (" << exception.what()
                      << "); using --rated-current " << constants.rated_current_arms << " A\n";
        }
    }

    const std::vector<std::pair<std::string, const stablecops::app::MotorDrive*>> drives = {
        {"top (node " + std::to_string(top_node) + ")", &top},
        {"bottom (node " + std::to_string(bottom_node) + ")", &bottom},
    };

    // Console table: name | object | per node (raw + decoded).
    constexpr int kNameWidth = 32;
    constexpr int kTagWidth = 12;
    constexpr int kRawWidth = 16;
    constexpr int kEngWidth = 16;
    const int line_width =
        kNameWidth + kTagWidth + ((kRawWidth + kEngWidth) * static_cast<int>(drives.size()));

    std::cout << '\n' << std::left << std::setw(kNameWidth) << "parameter" << std::setw(kTagWidth)
              << "object";
    for (const auto& drive : drives) {
        std::cout << std::setw(kRawWidth) << (drive.first + " raw") << std::setw(kEngWidth) << "value";
    }
    std::cout << '\n' << std::string(line_width, '-') << '\n';

    std::ofstream csv;
    if (!csv_path.empty()) {
        csv.open(csv_path);
        if (!csv) {
            std::cerr << "could not open CSV file for writing: " << csv_path << '\n';
        } else {
            csv << "section,parameter,index,subindex";
            for (const auto& drive : drives) {
                csv << ',' << drive.first << " raw," << drive.first << " value";
            }
            csv << '\n';
        }
    }

    std::string current_section;
    for (const auto& param : kParams) {
        if (param.section[0] != '\0') {
            current_section = param.section;
            std::cout << '\n' << "[" << current_section << "]\n";
        }

        std::cout << std::left << std::setw(kNameWidth) << param.name << std::setw(kTagWidth)
                  << objectTag(param);

        std::vector<std::pair<std::string, std::string>> node_values;  // (raw, eng)
        for (const auto& drive : drives) {
            std::string raw;
            std::string eng;
            const std::string formatted = readValue(*drive.second, param, constants, raw, eng);
            node_values.emplace_back(raw, eng);
            std::cout << std::setw(kRawWidth) << formatted << std::setw(kEngWidth) << eng;
        }
        std::cout << '\n';

        if (csv.is_open()) {
            std::ostringstream index_hex;
            index_hex << "0x" << std::hex << std::uppercase << std::setw(4) << std::setfill('0')
                      << param.index;
            csv << current_section << ',' << param.name << ',' << index_hex.str() << ','
                << static_cast<int>(param.subindex);
            for (const auto& value : node_values) {
                csv << ',' << value.first << ",\"" << value.second << '"';
            }
            csv << '\n';
        }
    }

    if (csv.is_open()) {
        std::cout << "\nwrote CSV to " << csv_path << '\n';
    }

    // Drives tear the bus down as they go out of scope.
    return EXIT_SUCCESS;
}
