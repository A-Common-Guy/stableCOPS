#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "nlohmann/json.hpp"
#include "stablecops/app/MotorConfig.hpp"
#include "stablecops/app/MotorDrive.hpp"
#include "stablecops/ds402/State.hpp"

namespace {

using json = nlohmann::json;

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
std::atomic<bool> g_stop_requested{false};
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
std::atomic<unsigned> g_signal_count{0};

constexpr auto kForcedShutdownExitDelay = std::chrono::milliseconds{750};

void handleSignal(int /*signal*/) {
    g_signal_count.fetch_add(1, std::memory_order_relaxed);
    g_stop_requested.store(true);
}

std::optional<stablecops::ds402::OperationMode> parseOperationMode(const std::string& name) {
    using stablecops::ds402::OperationMode;
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

std::string homingPhaseName(stablecops::ds402::HomingPhase phase) {
    using stablecops::ds402::HomingPhase;
    switch (phase) {
        case HomingPhase::Idle:
            return "idle";
        case HomingPhase::SearchNegative:
            return "search-negative";
        case HomingPhase::BackoffNegative:
            return "backoff-negative";
        case HomingPhase::SearchPositive:
            return "search-positive";
        case HomingPhase::MoveToCenter:
            return "move-to-center";
        case HomingPhase::WaitAtCenter:
            return "wait-at-center";
        case HomingPhase::ZeroAtCenter:
            return "zero-at-center";
        case HomingPhase::Done:
            return "done";
        case HomingPhase::Failed:
            return "failed";
    }
    return "unknown";
}

std::string trim(const std::string& value) {
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return {};
    }
    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

std::string lower(std::string value) {
    for (char& ch : value) {
        if (ch >= 'A' && ch <= 'Z') {
            ch = static_cast<char>(ch - 'A' + 'a');
        }
    }
    return value;
}

uint64_t parseUnsignedText(const std::string& text, uint64_t max_value) {
    std::size_t consumed = 0;
    const auto value = std::stoull(text, &consumed, 0);
    if (consumed != text.size() || value > max_value) {
        throw std::invalid_argument("numeric value out of range: " + text);
    }
    return value;
}

std::optional<std::vector<uint8_t>> parseNodeList(const std::string& spec) {
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

int64_t parseSignedText(const std::string& text) {
    std::size_t consumed = 0;
    const auto value = std::stoll(text, &consumed, 0);
    if (consumed != text.size()) {
        throw std::invalid_argument("invalid numeric value: " + text);
    }
    return value;
}

uint16_t parseIndex(const json& body) {
    if (!body.contains("index")) {
        throw std::invalid_argument("missing index");
    }
    if (body["index"].is_string()) {
        return static_cast<uint16_t>(parseUnsignedText(body["index"].get<std::string>(), 0xFFFF));
    }
    return static_cast<uint16_t>(body.at("index").get<uint32_t>());
}

uint8_t parseSubindex(const json& body) {
    if (!body.contains("subindex")) {
        return 0;
    }
    if (body["subindex"].is_string()) {
        return static_cast<uint8_t>(parseUnsignedText(body["subindex"].get<std::string>(), 0xFF));
    }
    return static_cast<uint8_t>(body.at("subindex").get<uint32_t>());
}

std::optional<uint8_t> parseNodeId(const json& body) {
    if (!body.contains("node")) {
        return std::nullopt;
    }
    if (body["node"].is_string()) {
        const auto value = parseUnsignedText(body["node"].get<std::string>(), 127);
        if (value == 0) {
            throw std::invalid_argument("node id must be in range 1..127");
        }
        return static_cast<uint8_t>(value);
    }
    const auto value = body.at("node").get<uint32_t>();
    if (value == 0 || value > 127) {
        throw std::invalid_argument("node id must be in range 1..127");
    }
    return static_cast<uint8_t>(value);
}

int64_t parseBodyValue(const json& body) {
    if (!body.contains("value")) {
        throw std::invalid_argument("missing value");
    }
    if (body["value"].is_string()) {
        return parseSignedText(body["value"].get<std::string>());
    }
    return body.at("value").get<int64_t>();
}

stablecops::app::ObjectDataType parseObjectType(const json& body) {
    const auto type = lower(body.value("type", "u32"));
    using stablecops::app::ObjectDataType;
    if (type == "u8") {
        return ObjectDataType::U8;
    }
    if (type == "i8") {
        return ObjectDataType::I8;
    }
    if (type == "u16") {
        return ObjectDataType::U16;
    }
    if (type == "i16") {
        return ObjectDataType::I16;
    }
    if (type == "u32") {
        return ObjectDataType::U32;
    }
    if (type == "i32") {
        return ObjectDataType::I32;
    }
    throw std::invalid_argument("unknown object type: " + type);
}

std::string objectTypeName(stablecops::app::ObjectDataType type) {
    using stablecops::app::ObjectDataType;
    switch (type) {
        case ObjectDataType::U8:
            return "u8";
        case ObjectDataType::I8:
            return "i8";
        case ObjectDataType::U16:
            return "u16";
        case ObjectDataType::I16:
            return "i16";
        case ObjectDataType::U32:
            return "u32";
        case ObjectDataType::I32:
            return "i32";
    }
    return "unknown";
}

std::string hexValue(uint64_t value, int width) {
    std::ostringstream stream;
    stream << "0x" << std::uppercase << std::hex << std::setw(width) << std::setfill('0') << value;
    return stream.str();
}

std::string objectValueHex(int64_t value, stablecops::app::ObjectDataType type) {
    using stablecops::app::ObjectDataType;
    switch (type) {
        case ObjectDataType::U8:
        case ObjectDataType::I8:
            return hexValue(static_cast<uint8_t>(value), 2);
        case ObjectDataType::U16:
        case ObjectDataType::I16:
            return hexValue(static_cast<uint16_t>(value), 4);
        case ObjectDataType::U32:
        case ObjectDataType::I32:
            return hexValue(static_cast<uint32_t>(value), 8);
    }
    return "0x0";
}

std::string dataTypeToUiType(const std::string& data_type) {
    const auto code = static_cast<uint32_t>(parseUnsignedText(data_type, 0xFFFF));
    switch (code) {
        case 0x0002:
            return "i8";
        case 0x0003:
            return "i16";
        case 0x0004:
            return "i32";
        case 0x0005:
            return "u8";
        case 0x0006:
            return "u16";
        case 0x0007:
            return "u32";
        default:
            return "unsupported";
    }
}

std::string dataTypeName(const std::string& data_type) {
    const auto code = static_cast<uint32_t>(parseUnsignedText(data_type, 0xFFFF));
    switch (code) {
        case 0x0001:
            return "BOOLEAN";
        case 0x0002:
            return "INTEGER8";
        case 0x0003:
            return "INTEGER16";
        case 0x0004:
            return "INTEGER32";
        case 0x0005:
            return "UNSIGNED8";
        case 0x0006:
            return "UNSIGNED16";
        case 0x0007:
            return "UNSIGNED32";
        case 0x0008:
            return "REAL32";
        case 0x0009:
            return "VISIBLE_STRING";
        case 0x000A:
            return "OCTET_STRING";
        case 0x000B:
            return "UNICODE_STRING";
        case 0x0011:
            return "REAL64";
        default:
            return data_type;
    }
}

std::string objectCategory(uint16_t index) {
    if (index < 0x2000) {
        return "Communication / standard";
    }
    if (index < 0x6000) {
        return "Manufacturer";
    }
    if (index < 0x7000) {
        return "CiA402 / profile";
    }
    return "Other";
}

bool parseEdsSectionName(const std::string& section, uint16_t& index, uint8_t& subindex) {
    if (section.size() == 4) {
        index = static_cast<uint16_t>(parseUnsignedText("0x" + section, 0xFFFF));
        subindex = 0;
        return true;
    }
    const auto sub_pos = section.find("sub");
    if (sub_pos == 4 && section.size() > 7) {
        index = static_cast<uint16_t>(parseUnsignedText("0x" + section.substr(0, 4), 0xFFFF));
        subindex =
            static_cast<uint8_t>(parseUnsignedText("0x" + section.substr(sub_pos + 3), 0xFF));
        return true;
    }
    return false;
}

using EdsSection = std::map<std::string, std::string>;

std::map<std::string, EdsSection> readEdsSections(const std::string& path) {
    std::ifstream file(path);
    if (!file) {
        throw std::runtime_error("failed to open EDS object dictionary: " + path);
    }

    std::map<std::string, EdsSection> sections;
    std::string current;
    std::string line;
    while (std::getline(file, line)) {
        line = trim(line);
        if (line.empty() || line[0] == ';') {
            continue;
        }
        if (line.front() == '[' && line.back() == ']') {
            current = line.substr(1, line.size() - 2);
            sections[current];
            continue;
        }
        const auto equals = line.find('=');
        if (current.empty() || equals == std::string::npos) {
            continue;
        }
        const auto key = trim(line.substr(0, equals));
        const auto value = trim(line.substr(equals + 1));
        sections[current][key] = value;
    }
    return sections;
}

json builtinObjects() {
    return json::array({
        {{"name", "Statusword"},
         {"index", "0x6041"},
         {"subindex", 0},
         {"type", "u16"},
         {"access", "read"},
         {"note", "TPDO cached when feedback is live"}},
        {{"name", "Mode request"},
         {"index", "0x6060"},
         {"subindex", 0},
         {"type", "i8"},
         {"access", "read/write"},
         {"note", "Best selected at daemon boot with --mode"}},
        {{"name", "Mode display"},
         {"index", "0x6061"},
         {"subindex", 0},
         {"type", "i8"},
         {"access", "read"},
         {"note", "TPDO cached when feedback is live"}},
        {{"name", "Position actual value"},
         {"index", "0x6064"},
         {"subindex", 0},
         {"type", "i32"},
         {"access", "read"},
         {"note", "Output-shaft counts"}},
        {{"name", "Velocity actual value"},
         {"index", "0x606C"},
         {"subindex", 0},
         {"type", "i32"},
         {"access", "read"},
         {"note", "Drive velocity units"}},
        {{"name", "Torque actual value"},
         {"index", "0x6077"},
         {"subindex", 0},
         {"type", "i16"},
         {"access", "read"},
         {"note", "Drive torque units"}},
        {{"name", "Error code"},
         {"index", "0x603F"},
         {"subindex", 0},
         {"type", "u16"},
         {"access", "read"},
         {"note", "CiA402 error code"}},
        {{"name", "Profile velocity"},
         {"index", "0x6081"},
         {"subindex", 0},
         {"type", "u32"},
         {"access", "read/write"},
         {"note", "Profile-position cruise speed"}},
        {{"name", "Profile acceleration"},
         {"index", "0x6083"},
         {"subindex", 0},
         {"type", "u32"},
         {"access", "read/write"},
         {"note", "Profile ramp"}},
        {{"name", "Profile deceleration"},
         {"index", "0x6084"},
         {"subindex", 0},
         {"type", "u32"},
         {"access", "read/write"},
         {"note", "Profile ramp"}},
        {{"name", "Torque slope"},
         {"index", "0x6087"},
         {"subindex", 0},
         {"type", "u32"},
         {"access", "read/write"},
         {"note", "Profile torque ramp"}},
        {{"name", "Encoder single-turn primary"},
         {"index", "0x276F"},
         {"subindex", 0},
         {"type", "i32"},
         {"access", "read"},
         {"note", "Vendor raw encoder monitor"}},
        {{"name", "Encoder single-turn 3"},
         {"index", "0x2772"},
         {"subindex", 0},
         {"type", "i32"},
         {"access", "read"},
         {"note", "Vendor raw encoder monitor"}},
        {{"name", "Encoder increments"},
         {"index", "0x608F"},
         {"subindex", 1},
         {"type", "u32"},
         {"access", "read"},
         {"note", "Scaling numerator"}},
        {{"name", "Gear ratio shaft revolutions"},
         {"index", "0x6091"},
         {"subindex", 2},
         {"type", "u32"},
         {"access", "read"},
         {"note", "Default counts/rev reference"}},
    });
}

json loadObjectDictionaryFromEds(const std::string& path) {
    json objects = json::array();
    for (const auto& [section_name, fields] : readEdsSections(path)) {
        uint16_t index = 0;
        uint8_t subindex = 0;
        if (!parseEdsSectionName(section_name, index, subindex)) {
            continue;
        }
        const auto name_it = fields.find("ParameterName");
        const auto type_it = fields.find("DataType");
        const auto access_it = fields.find("AccessType");
        if (name_it == fields.end() || type_it == fields.end() || access_it == fields.end()) {
            continue;
        }

        const auto pdo_it = fields.find("PDOMapping");
        const auto default_it = fields.find("DefaultValue");
        const auto ui_type = dataTypeToUiType(type_it->second);
        objects.push_back({
            {"name", name_it->second},
            {"index", hexValue(index, 4)},
            {"subindex", subindex},
            {"type", ui_type},
            {"data_type", dataTypeName(type_it->second)},
            {"access", access_it->second},
            {"pdo_mapping", pdo_it != fields.end() && pdo_it->second != "0"},
            {"default", default_it != fields.end() ? default_it->second : ""},
            {"category", objectCategory(index)},
            {"note",
             ui_type == "unsupported"
                 ? "Listed from EDS/manual, but this simple UI only reads/writes integer objects"
                 : "Listed from EYou RP EDS object dictionary"},
        });
    }
    if (objects.empty()) {
        throw std::runtime_error("EDS object dictionary did not contain readable object entries");
    }
    return objects;
}

std::string edsPathFromSummary(const std::string& summary_path) {
    std::ifstream file(summary_path);
    if (!file) {
        return "generated/canopen/euservo_rp/euservo_rp.normalized.eds";
    }
    const auto summary = json::parse(file);
    return summary.value("normalized_eds",
                         "generated/canopen/euservo_rp/euservo_rp.normalized.eds");
}

struct HttpRequest {
    std::string method;
    std::string path;
    std::map<std::string, std::string> headers;
    std::string body;
};

struct HttpResponse {
    int status{200};
    std::string content_type{"application/json"};
    std::string body;
};

std::string reasonPhrase(int status) {
    switch (status) {
        case 200:
            return "OK";
        case 400:
            return "Bad Request";
        case 404:
            return "Not Found";
        case 405:
            return "Method Not Allowed";
        case 500:
            return "Internal Server Error";
        default:
            return "Error";
    }
}

HttpResponse jsonResponse(const json& body, int status = 200) {
    return {status, "application/json", body.dump(2)};
}

HttpResponse errorResponse(int status, const std::string& message) {
    return jsonResponse({{"ok", false}, {"error", message}}, status);
}

std::optional<HttpRequest> readHttpRequest(int fd) {
    std::string data;
    char buffer[4096];
    while (data.find("\r\n\r\n") == std::string::npos) {
        const auto n = ::recv(fd, buffer, sizeof(buffer), 0);
        if (n <= 0) {
            return std::nullopt;
        }
        data.append(buffer, static_cast<std::size_t>(n));
        if (data.size() > 1024 * 1024) {
            throw std::runtime_error("request too large");
        }
    }

    const auto header_end = data.find("\r\n\r\n");
    std::istringstream headers(data.substr(0, header_end));
    std::string request_line;
    std::getline(headers, request_line);
    request_line = trim(request_line);

    std::istringstream request_parts(request_line);
    HttpRequest request;
    std::string target;
    request_parts >> request.method >> target;
    const auto query = target.find('?');
    request.path = target.substr(0, query);

    std::string line;
    while (std::getline(headers, line)) {
        const auto colon = line.find(':');
        if (colon == std::string::npos) {
            continue;
        }
        request.headers[lower(trim(line.substr(0, colon)))] = trim(line.substr(colon + 1));
    }

    std::size_t content_length = 0;
    const auto content_length_it = request.headers.find("content-length");
    if (content_length_it != request.headers.end()) {
        content_length =
            static_cast<std::size_t>(parseUnsignedText(content_length_it->second, 1024 * 1024));
    }

    request.body = data.substr(header_end + 4);
    while (request.body.size() < content_length) {
        const auto n = ::recv(fd, buffer, sizeof(buffer), 0);
        if (n <= 0) {
            throw std::runtime_error("connection closed while reading body");
        }
        request.body.append(buffer, static_cast<std::size_t>(n));
    }
    if (request.body.size() > content_length) {
        request.body.resize(content_length);
    }

    return request;
}

void sendHttpResponse(int fd, const HttpResponse& response) {
    std::ostringstream stream;
    stream << "HTTP/1.1 " << response.status << ' ' << reasonPhrase(response.status)
           << "\r\nContent-Type: " << response.content_type
           << "\r\nContent-Length: " << response.body.size()
           << "\r\nConnection: close\r\nAccess-Control-Allow-Origin: *\r\n\r\n"
           << response.body;
    const auto wire = stream.str();
    const char* cursor = wire.data();
    std::size_t remaining = wire.size();
    while (remaining > 0) {
        const auto sent = ::send(fd, cursor, remaining, 0);
        if (sent <= 0) {
            return;
        }
        cursor += sent;
        remaining -= static_cast<std::size_t>(sent);
    }
}

class HttpServer {
public:
    using Handler = std::function<HttpResponse(const HttpRequest&)>;

    HttpServer(std::string host, uint16_t port, Handler handler)
        : host_(std::move(host)), port_(port), handler_(std::move(handler)) {}

    void run() {
        const int listen_fd = ::socket(AF_INET, SOCK_STREAM, 0);
        if (listen_fd < 0) {
            throw std::runtime_error("socket() failed");
        }

        int reuse = 1;
        ::setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_port = htons(port_);
        if (host_ == "0.0.0.0") {
            address.sin_addr.s_addr = INADDR_ANY;
        } else if (::inet_pton(AF_INET, host_.c_str(), &address.sin_addr) != 1) {
            ::close(listen_fd);
            throw std::runtime_error("invalid IPv4 bind address: " + host_);
        }

        if (::bind(listen_fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
            const std::string error = std::strerror(errno);
            ::close(listen_fd);
            throw std::runtime_error("bind() failed: " + error);
        }
        if (::listen(listen_fd, 16) < 0) {
            const std::string error = std::strerror(errno);
            ::close(listen_fd);
            throw std::runtime_error("listen() failed: " + error);
        }

        std::cout << "commissioning UI: http://" << host_ << ':' << port_ << "/\n";
        while (!g_stop_requested.load()) {
            pollfd pfd{listen_fd, POLLIN, 0};
            const int ready = ::poll(&pfd, 1, 250);
            if (ready <= 0) {
                continue;
            }
            const int client_fd = ::accept(listen_fd, nullptr, nullptr);
            if (client_fd < 0) {
                continue;
            }
            try {
                if (const auto request = readHttpRequest(client_fd)) {
                    sendHttpResponse(client_fd, handler_(*request));
                }
            } catch (const std::exception& exception) {
                sendHttpResponse(client_fd, errorResponse(400, exception.what()));
            }
            ::close(client_fd);
        }

        ::close(listen_fd);
    }

private:
    std::string host_;
    uint16_t port_;
    Handler handler_;
};

const char* indexHtml() {
    return R"HTML(<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>stableCOPS Commissioning</title>
  <style>
    :root { color-scheme: dark; font-family: Inter, system-ui, sans-serif; background: #111827; color: #e5e7eb; }
    body { margin: 0; padding: 24px; }
    h1 { margin: 0 0 8px; font-size: 26px; }
    h2 { margin: 0 0 12px; font-size: 18px; }
    .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(320px, 1fr)); gap: 16px; }
    .card { background: #1f2937; border: 1px solid #374151; border-radius: 12px; padding: 16px; box-shadow: 0 10px 30px #0004; }
    .row { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; align-items: center; margin: 6px 0; }
    label { color: #9ca3af; font-size: 13px; }
    input, select, button { border-radius: 8px; border: 1px solid #4b5563; padding: 9px 10px; background: #111827; color: #f9fafb; }
    button { cursor: pointer; background: #2563eb; border-color: #3b82f6; font-weight: 600; }
    button.warn { background: #b45309; border-color: #d97706; }
    button.danger { background: #b91c1c; border-color: #ef4444; }
    button.secondary { background: #374151; border-color: #6b7280; }
    .status { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 8px; }
    .metric { background: #111827; border-radius: 8px; padding: 10px; }
    .metric span { display: block; color: #9ca3af; font-size: 12px; }
    .metric strong { display: block; margin-top: 4px; font-size: 15px; overflow-wrap: anywhere; }
    .ok { color: #34d399; }
    .bad { color: #f87171; }
    .note { color: #9ca3af; font-size: 13px; line-height: 1.45; }
    pre { white-space: pre-wrap; background: #030712; border-radius: 8px; padding: 12px; min-height: 60px; max-height: 220px; overflow: auto; }
    .actions { display: flex; flex-wrap: wrap; gap: 8px; margin-top: 12px; }
  </style>
</head>
<body>
  <h1>stableCOPS Commissioning</h1>
  <p class="note">Local browser UI for CANopen/CiA402 bring-up. Motion buttons use the existing MotorDrive/CiA402 path; SDO fields are for parameters and diagnostics.</p>
  <div class="grid">
    <section class="card">
      <h2>Drive Status</h2>
      <div class="row"><label>Target node</label><select id="node"></select></div>
      <div id="status" class="status"></div>
      <div class="actions">
        <button onclick="post('/api/enable', {hold:true})">Enable + Hold</button>
        <button class="secondary" onclick="post('/api/enable', {hold:false})">Enable</button>
        <button class="warn" onclick="post('/api/fault-reset', {})">Fault Reset</button>
        <button class="danger" onclick="post('/api/stop', {})">Stop</button>
      </div>
    </section>
    <section class="card">
      <h2>Motion</h2>
      <div class="row"><label>Operation mode (0x6060)</label><select id="mode"><option value="csp">CSP - cyclic synchronous position</option><option value="csv">CSV - cyclic synchronous velocity</option><option value="cst">CST - cyclic synchronous torque</option><option value="pp">PP - profile position</option><option value="pv">PV - profile velocity</option><option value="pt">PT - profile torque</option></select></div>
      <div class="row"><label>Command</label><select id="moveType" onchange="syncModeFromMove()"><option value="csp">CSP target position</option><option value="csp-relative">CSP relative step</option><option value="pp">Profile position absolute</option><option value="pp-relative">Profile position relative</option><option value="velocity">Velocity target</option><option value="torque">Torque target</option></select></div>
      <div class="row"><label>Value</label><input id="moveValue" value="0"></div>
      <div class="actions"><button class="secondary" onclick="setMode()">Send Mode (0x6060)</button><button onclick="move()">Send Motion Command</button></div>
      <p class="note">Send the operation mode before enabling or before the first setpoint. The drive can reject runtime mode writes depending on state; the daemon reports that SDO error instead of hiding it.</p>
    </section>
    <section class="card">
      <h2>Homing</h2>
      <div class="row"><label>Search velocity</label><input id="homeSearchVelocity" value="2000"></div>
      <div class="row"><label>Approach / center velocity</label><input id="homeApproachVelocity" value="500"></div>
      <div class="row"><label>Torque threshold</label><input id="homeTorque" value="120"></div>
      <div class="row"><label>Backoff distance</label><input id="homeBackoff" value="2000"></div>
      <div class="row"><label>Save zero to NVM</label><select id="homeSaveNvm"><option value="true">yes</option><option value="false">no</option></select></div>
      <div class="actions"><button onclick="startHoming()">Start Homing</button></div>
      <p class="note">Use the safe sequence first: Stop, wait for switch-on-disabled, Send Mode CSV, Enable, then Start Homing.</p>
    </section>
    <section class="card">
      <h2>Object Dictionary</h2>
      <div class="row"><label>Search register</label><input id="objectSearch" placeholder="6064, velocity, encoder..." oninput="renderObjectOptions()"></div>
      <div class="row"><label>Common object</label><select id="commonObjects" onchange="pickObject()"></select></div>
      <div class="row"><label>Index</label><input id="index" value="0x6064"></div>
      <div class="row"><label>Subindex</label><input id="subindex" value="0"></div>
      <div class="row"><label>Type</label><select id="type"><option>i32</option><option>u32</option><option>i16</option><option>u16</option><option>i8</option><option>u8</option><option>unsupported</option></select></div>
      <div class="row"><label>Write value</label><input id="writeValue" value="0"></div>
      <div class="actions">
        <button onclick="sdoRead()">Read</button>
        <button class="warn" onclick="sdoWrite()">Write</button>
      </div>
      <p id="objectInfo" class="note">Object dictionary entries are loaded from the EYou RP EDS generated from the vendor data/manual.</p>
      <p class="note">On this RP firmware, target/controlword objects are PDO driven once mapped; use Motion and Enable controls for those instead of manual SDO writes.</p>
    </section>
    <section class="card">
      <h2>Result</h2>
      <pre id="log">Ready.</pre>
    </section>
  </div>
  <script>
    let objects = [];
    let nodes = [];
    const $ = id => document.getElementById(id);
    async function api(path, options = {}) {
      const response = await fetch(path, options);
      const text = await response.text();
      const data = text ? JSON.parse(text) : {};
      if (!response.ok) throw new Error(data.error || text || response.statusText);
      return data;
    }
    async function post(path, body) {
      try {
        body = {node: selectedNode(), ...body};
        const data = await api(path, {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body)});
        log(data);
        await refresh();
      } catch (error) { log({ok:false, error: error.message}); }
    }
    function requestObject() {
      return {index: $('index').value, subindex: $('subindex').value, type: $('type').value};
    }
    async function sdoRead() { await post('/api/sdo/read', requestObject()); }
    async function sdoWrite() { await post('/api/sdo/write', {...requestObject(), value: $('writeValue').value}); }
    async function setMode() { await post('/api/mode', {mode: $('mode').value}); }
    function modeForMove(kind) {
      if (kind.startsWith('csp')) return 'csp';
      if (kind.startsWith('pp')) return 'pp';
      if (kind === 'velocity') return 'csv';
      if (kind === 'torque') return 'cst';
      return 'csp';
    }
    function syncModeFromMove() { $('mode').value = modeForMove($('moveType').value); }
    async function move() {
      const kind = $('moveType').value;
      const body = {type: kind.replace('-relative', ''), value: $('moveValue').value, relative: kind.endsWith('-relative')};
      await post('/api/move', body);
    }
    async function startHoming() {
      await post('/api/home/start', {
        search_velocity: $('homeSearchVelocity').value,
        approach_velocity: $('homeApproachVelocity').value,
        threshold_torque: $('homeTorque').value,
        backoff_distance: $('homeBackoff').value,
        save_zero_to_nvm: $('homeSaveNvm').value === 'true'
      });
    }
    function metric(label, value, cls = '') {
      return `<div class="metric"><span>${label}</span><strong class="${cls}">${value}</strong></div>`;
    }
    function selectedNode() {
      return Number($('node').value || (nodes[0] && nodes[0].node_id) || 1);
    }
    function updateNodeSelect(nextNodes) {
      const previous = String(selectedNode());
      nodes = nextNodes || [];
      const options = nodes.map(n => `<option value="${n.node_id}">Node ${n.node_id}${n.feedback_live ? ' [live]' : ' [stale]'}</option>`).join('');
      if ($('node').innerHTML !== options) $('node').innerHTML = options;
      if (nodes.some(n => String(n.node_id) === previous)) $('node').value = previous;
    }
    async function refresh() {
      try {
        const s = await api('/api/status');
        updateNodeSelect(s.nodes);
        const node = s.nodes.find(n => n.node_id === selectedNode()) || s.nodes[0];
        if (!node) throw new Error('no nodes configured');
        $('status').innerHTML = [
          metric('Feedback', node.feedback_live ? 'live' : 'stale', node.feedback_live ? 'ok' : 'bad'),
          metric('CiA402', node.enabled ? 'operation enabled' : node.feedback.state, node.enabled ? 'ok' : ''),
          metric('Mode', node.feedback.mode),
          metric('Position', `${node.feedback.position} counts`),
          metric('Angle', `${node.feedback.position_degrees.toFixed(3)} deg`),
          metric('Velocity', node.feedback.velocity),
          metric('Torque', node.feedback.torque),
          metric('Homing', node.homing.phase, node.homing.result.success ? 'ok' : ''),
          metric('Error', node.feedback.error_code_hex, node.faulted ? 'bad' : 'ok')
        ].join('');
      } catch (error) {
        $('status').innerHTML = metric('Server', error.message, 'bad');
      }
    }
    function objectMatches(o, q) {
      if (!q) return true;
      const text = `${o.index} ${o.subindex} ${o.name} ${o.access} ${o.data_type} ${o.category}`.toLowerCase();
      return text.includes(q);
    }
    function renderObjectOptions() {
      const q = $('objectSearch').value.trim().toLowerCase();
      const visible = objects.map((o, i) => ({o, i})).filter(x => objectMatches(x.o, q)).slice(0, 300);
      $('commonObjects').innerHTML = visible.map(({o, i}) => `<option value="${i}">${o.index}:${o.subindex} ${o.name} [${o.access}, ${o.data_type}]</option>`).join('');
      pickObject();
    }
    async function loadObjects() {
      objects = (await api('/api/objects')).objects;
      renderObjectOptions();
    }
    function pickObject() {
      const o = objects[Number($('commonObjects').value)];
      if (!o) return;
      $('index').value = o.index;
      $('subindex').value = o.subindex;
      $('type').value = o.type;
      $('objectInfo').textContent = `${o.name} | ${o.category} | access=${o.access} | data=${o.data_type} | PDO=${o.pdo_mapping ? 'yes' : 'no'} | default=${o.default || '-'} | ${o.note || ''}`;
    }
    function log(value) { $('log').textContent = JSON.stringify(value, null, 2); }
    loadObjects().then(refresh);
    setInterval(refresh, 500);
  </script>
</body>
</html>)HTML";
}

json feedbackJson(uint8_t node_id, stablecops::app::MotorDrive& drive) {
    const auto feedback = drive.feedback();
    const auto stats = drive.cyclicStats();
    const auto homing_phase = drive.homingPhase();
    const auto homing_result = drive.homingResult();
    return {
        {"node_id", node_id},
        {"running", drive.running()},
        {"feedback_live", drive.feedbackLive()},
        {"enabled", drive.enabled()},
        {"faulted", drive.faulted()},
        {"feedback",
         {{"statusword", feedback.statusword},
          {"statusword_hex", hexValue(feedback.statusword, 4)},
          {"state", stablecops::ds402::toString(feedback.state)},
          {"mode", stablecops::ds402::toString(feedback.mode)},
          {"position", feedback.position},
          {"position_degrees", drive.positionDegrees()},
          {"position_radians", drive.positionRadians()},
          {"velocity", feedback.velocity},
          {"torque", feedback.torque},
          {"error_code", feedback.error_code},
          {"error_code_hex", hexValue(feedback.error_code, 4)}}},
        {"cyclic_stats",
         {{"cycles", stats.cycles},
          {"last_us", stats.last_us},
          {"min_us", stats.min_us},
          {"max_us", stats.max_us},
          {"mean_us", stats.mean_us},
          {"max_jitter_us", stats.max_jitter_us}}},
        {"homing",
         {{"phase", homingPhaseName(homing_phase)},
          {"result",
           {{"success", homing_result.success},
            {"lower_limit_position", homing_result.lower_limit_position},
            {"upper_limit_position", homing_result.upper_limit_position},
            {"center_position", homing_result.center_position},
            {"travel", homing_result.travel}}}}},
    };
}

using DriveMap = std::map<uint8_t, stablecops::app::MotorDrive*>;

json allFeedbackJson(const DriveMap& drives) {
    json nodes = json::array();
    for (const auto& [node_id, drive] : drives) {
        nodes.push_back(feedbackJson(node_id, *drive));
    }
    return {{"ok", true}, {"nodes", nodes}};
}

int32_t checkedI32(int64_t value, const char* field) {
    if (value < std::numeric_limits<int32_t>::min() ||
        value > std::numeric_limits<int32_t>::max()) {
        throw std::invalid_argument(std::string(field) + " is outside int32 range");
    }
    return static_cast<int32_t>(value);
}

int16_t checkedI16(int64_t value, const char* field) {
    if (value < std::numeric_limits<int16_t>::min() ||
        value > std::numeric_limits<int16_t>::max()) {
        throw std::invalid_argument(std::string(field) + " is outside int16 range");
    }
    return static_cast<int16_t>(value);
}

int64_t optionalSigned(const json& body, const char* field, int64_t fallback) {
    if (!body.contains(field)) {
        return fallback;
    }
    if (body[field].is_string()) {
        return parseSignedText(body[field].get<std::string>());
    }
    return body.at(field).get<int64_t>();
}

std::chrono::milliseconds optionalMilliseconds(const json& body,
                                               const char* field,
                                               std::chrono::milliseconds fallback) {
    return std::chrono::milliseconds{optionalSigned(body, field, fallback.count())};
}

bool optionalBool(const json& body, const char* field, bool fallback) {
    if (!body.contains(field)) {
        return fallback;
    }
    if (body[field].is_boolean()) {
        return body[field].get<bool>();
    }
    if (body[field].is_string()) {
        const auto value = lower(body[field].get<std::string>());
        if (value == "true" || value == "1" || value == "yes") {
            return true;
        }
        if (value == "false" || value == "0" || value == "no") {
            return false;
        }
    }
    throw std::invalid_argument(std::string(field) + " must be a boolean");
}

stablecops::ds402::HomingConfig parseHomingConfig(const json& body) {
    stablecops::ds402::HomingConfig config;
    config.search_velocity =
        checkedI32(optionalSigned(body, "search_velocity", config.search_velocity),
                   "search_velocity");
    config.approach_velocity =
        checkedI32(optionalSigned(body, "approach_velocity", config.approach_velocity),
                   "approach_velocity");
    config.backoff_distance =
        checkedI32(optionalSigned(body, "backoff_distance", config.backoff_distance),
                   "backoff_distance");
    config.center_tolerance =
        checkedI32(optionalSigned(body, "center_tolerance", config.center_tolerance),
                   "center_tolerance");
    config.min_travel =
        checkedI32(optionalSigned(body, "min_travel", config.min_travel), "min_travel");
    config.max_travel =
        checkedI32(optionalSigned(body, "max_travel", config.max_travel), "max_travel");
    config.home_offset =
        checkedI32(optionalSigned(body, "home_offset", config.home_offset), "home_offset");
    config.threshold_torque =
        checkedI16(optionalSigned(body, "threshold_torque", config.threshold_torque),
                   "threshold_torque");
    config.stopped_velocity =
        checkedI32(optionalSigned(body, "stopped_velocity", config.stopped_velocity),
                   "stopped_velocity");
    config.contact_dwell = optionalMilliseconds(body, "contact_dwell_ms", config.contact_dwell);
    config.settle_time = optionalMilliseconds(body, "settle_time_ms", config.settle_time);
    config.timeout = optionalMilliseconds(body, "timeout_ms", config.timeout);
    config.save_zero_to_nvm =
        optionalBool(body, "save_zero_to_nvm", config.save_zero_to_nvm);
    return config;
}

void checkPositionStep(int64_t current, int64_t target, int32_t max_step) {
    const auto delta = target - current;
    if (std::llabs(delta) > max_step) {
        throw std::invalid_argument("requested position step exceeds max-position-step");
    }
}

class CommissioningApi {
public:
    CommissioningApi(DriveMap drives, int32_t max_position_step, json object_catalog)
        : drives_(std::move(drives)),
          max_position_step_(max_position_step),
          object_catalog_(std::move(object_catalog)) {
        if (drives_.empty()) {
            throw std::invalid_argument("CommissioningApi requires at least one drive");
        }
    }

    HttpResponse operator()(const HttpRequest& request) {
        try {
            if (request.method == "GET" && request.path == "/") {
                return {200, "text/html; charset=utf-8", indexHtml()};
            }
            if (request.method == "GET" && request.path == "/api/status") {
                return jsonResponse(allFeedbackJson(drives_));
            }
            if (request.method == "GET" && request.path == "/api/objects") {
                return jsonResponse({{"ok", true}, {"objects", object_catalog_}});
            }
            if (request.method != "POST") {
                return errorResponse(404, "unknown endpoint");
            }

            const json body = request.body.empty() ? json::object() : json::parse(request.body);
            auto& drive = targetDrive(body);
            if (request.path == "/api/enable") {
                drive.enableOperation(body.value("hold", true));
                return jsonResponse(
                    {{"ok", true}, {"node", targetNode(body)}, {"action", "enable"}});
            }
            if (request.path == "/api/stop") {
                drive.stop();
                return jsonResponse({{"ok", true}, {"node", targetNode(body)}, {"action", "stop"}});
            }
            if (request.path == "/api/fault-reset") {
                drive.resetFault();
                return jsonResponse(
                    {{"ok", true}, {"node", targetNode(body)}, {"action", "fault-reset"}});
            }
            if (request.path == "/api/mode") {
                if (!body.contains("mode") || !body["mode"].is_string()) {
                    return errorResponse(400, "missing mode");
                }
                const auto mode_name = body["mode"].get<std::string>();
                const auto mode = parseOperationMode(mode_name);
                if (!mode) {
                    return errorResponse(400, "unknown mode: " + mode_name);
                }
                drive.setOperationMode(*mode);
                return jsonResponse({{"ok", true},
                                     {"node", targetNode(body)},
                                     {"action", "mode"},
                                     {"mode", mode_name},
                                     {"mode_text", stablecops::ds402::toString(*mode)}});
            }
            if (request.path == "/api/sdo/read") {
                const auto index = parseIndex(body);
                const auto subindex = parseSubindex(body);
                const auto type = parseObjectType(body);
                const auto value = drive.readObject(index, subindex, type);
                return jsonResponse({{"ok", true},
                                     {"node", targetNode(body)},
                                     {"index", hexValue(index, 4)},
                                     {"subindex", subindex},
                                     {"type", objectTypeName(type)},
                                     {"value", value},
                                     {"hex", objectValueHex(value, type)}});
            }
            if (request.path == "/api/sdo/write") {
                const auto index = parseIndex(body);
                const auto subindex = parseSubindex(body);
                const auto type = parseObjectType(body);
                const auto value = parseBodyValue(body);
                drive.writeObject(index, subindex, type, value);
                return jsonResponse({{"ok", true},
                                     {"node", targetNode(body)},
                                     {"index", hexValue(index, 4)},
                                     {"subindex", subindex},
                                     {"type", objectTypeName(type)},
                                     {"value", value},
                                     {"hex", objectValueHex(value, type)}});
            }
            if (request.path == "/api/move") {
                return move(body);
            }
            if (request.path == "/api/home/start") {
                return startHoming(body);
            }
            return errorResponse(404, "unknown endpoint");
        } catch (const json::exception& exception) {
            return errorResponse(400, exception.what());
        } catch (const std::exception& exception) {
            return errorResponse(500, exception.what());
        }
    }

private:
    uint8_t targetNode(const json& body) const {
        if (const auto requested = parseNodeId(body)) {
            if (drives_.find(*requested) == drives_.end()) {
                throw std::invalid_argument("node is not configured in this daemon");
            }
            return *requested;
        }
        return drives_.begin()->first;
    }

    stablecops::app::MotorDrive& targetDrive(const json& body) const {
        const auto node_id = targetNode(body);
        return *drives_.at(node_id);
    }

    HttpResponse move(const json& body) {
        auto& drive = targetDrive(body);
        const auto node_id = targetNode(body);
        const auto type = lower(body.value("type", "csp"));
        const auto value = parseBodyValue(body);
        const bool relative = body.value("relative", false);
        if (type == "csp") {
            const auto feedback = drive.feedback();
            const int64_t target =
                relative ? static_cast<int64_t>(feedback.position) + value : value;
            checkPositionStep(feedback.position, target, max_position_step_);
            drive.commandPosition(checkedI32(target, "target"));
            return jsonResponse({{"ok", true},
                                 {"node", node_id},
                                 {"action", relative ? "csp-relative" : "csp"},
                                 {"target", target}});
        }
        if (type == "pp") {
            drive.moveToPosition(checkedI32(value, "target"), relative);
            return jsonResponse({{"ok", true},
                                 {"node", node_id},
                                 {"action", relative ? "pp-relative" : "pp"},
                                 {"target", value}});
        }
        if (type == "velocity") {
            drive.commandVelocity(checkedI32(value, "velocity"));
            return jsonResponse(
                {{"ok", true}, {"node", node_id}, {"action", "velocity"}, {"target", value}});
        }
        if (type == "torque") {
            drive.commandTorque(checkedI16(value, "torque"));
            return jsonResponse(
                {{"ok", true}, {"node", node_id}, {"action", "torque"}, {"target", value}});
        }
        return errorResponse(400, "unknown move type: " + type);
    }

    HttpResponse startHoming(const json& body) {
        auto& drive = targetDrive(body);
        const auto node_id = targetNode(body);
        const auto config = parseHomingConfig(body);
        drive.startHoming(config);
        return jsonResponse({{"ok", true},
                             {"node", node_id},
                             {"action", "home-start"},
                             {"phase", homingPhaseName(drive.homingPhase())}});
    }

    DriveMap drives_;
    int32_t max_position_step_;
    json object_catalog_;
};

}  // namespace

int main(int argc, char** argv) {
    stablecops::app::MotorConfig config;
    config.monitor_on_boot = true;
    config.operation_mode = stablecops::ds402::OperationMode::CyclicSynchronousPosition;
    std::vector<uint8_t> node_ids;

    std::string host{"127.0.0.1"};
    std::string eds_path;
    uint16_t port = 8765;

    const auto print_usage = [] {
        std::cerr << "usage: stablecops_commissiond [--host 127.0.0.1] [--port 8765] "
                     "[--can can0] [--dcf dcf/master.dcf] "
                     "[--summary generated/.../<name>.summary.json] [--eds path] "
                     "[--master-node 127] [--node 1] [--nodes 1,2,3] "
                     "[--mode csp|csv|cst|pp|pv|pt] "
                     "[--profile-velocity n] [--profile-accel n] [--profile-decel n] "
                     "[--torque-slope n] [--max-position-step counts] "
                     "[--feedback-timeout ms] [--counts-per-rev n] "
                     "[--sync-period-us 1000] [--enable] [--hold-position]\n";
    };

    try {
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--host" && i + 1 < argc) {
                host = argv[++i];
            } else if (arg == "--port" && i + 1 < argc) {
                const auto parsed = parseUnsignedText(argv[++i], 65535);
                port = static_cast<uint16_t>(parsed);
            } else if (arg == "--can" && i + 1 < argc) {
                config.can_interface = argv[++i];
            } else if (arg == "--dcf" && i + 1 < argc) {
                config.master_dcf_path = argv[++i];
            } else if (arg == "--summary" && i + 1 < argc) {
                config.summary_path = argv[++i];
            } else if (arg == "--eds" && i + 1 < argc) {
                eds_path = argv[++i];
            } else if (arg == "--master-node" && i + 1 < argc) {
                config.master_node_id = static_cast<uint8_t>(parseUnsignedText(argv[++i], 127));
            } else if (arg == "--node" && i + 1 < argc) {
                config.node_id = static_cast<uint8_t>(parseUnsignedText(argv[++i], 127));
            } else if (arg == "--nodes" && i + 1 < argc) {
                auto parsed = parseNodeList(argv[++i]);
                if (!parsed) {
                    print_usage();
                    return EXIT_FAILURE;
                }
                node_ids = *parsed;
            } else if (arg == "--mode" && i + 1 < argc) {
                const auto mode = parseOperationMode(argv[++i]);
                if (!mode) {
                    print_usage();
                    return EXIT_FAILURE;
                }
                config.operation_mode = mode;
            } else if (arg == "--profile-velocity" && i + 1 < argc) {
                config.profile_velocity = static_cast<uint32_t>(
                    parseUnsignedText(argv[++i], std::numeric_limits<uint32_t>::max()));
            } else if (arg == "--profile-accel" && i + 1 < argc) {
                config.profile_acceleration = static_cast<uint32_t>(
                    parseUnsignedText(argv[++i], std::numeric_limits<uint32_t>::max()));
            } else if (arg == "--profile-decel" && i + 1 < argc) {
                config.profile_deceleration = static_cast<uint32_t>(
                    parseUnsignedText(argv[++i], std::numeric_limits<uint32_t>::max()));
            } else if (arg == "--torque-slope" && i + 1 < argc) {
                config.torque_slope = static_cast<uint32_t>(
                    parseUnsignedText(argv[++i], std::numeric_limits<uint32_t>::max()));
            } else if (arg == "--max-position-step" && i + 1 < argc) {
                config.max_position_step = static_cast<int32_t>(
                    parseUnsignedText(argv[++i], std::numeric_limits<int32_t>::max()));
            } else if (arg == "--feedback-timeout" && i + 1 < argc) {
                config.feedback_timeout = std::chrono::milliseconds{
                    static_cast<long>(parseUnsignedText(argv[++i], 60000))};
            } else if (arg == "--counts-per-rev" && i + 1 < argc) {
                config.counts_per_rev = static_cast<uint32_t>(
                    parseUnsignedText(argv[++i], std::numeric_limits<uint32_t>::max()));
            } else if (arg == "--sync-period-us" && i + 1 < argc) {
                config.sync_period_us = static_cast<uint32_t>(
                    parseUnsignedText(argv[++i], std::numeric_limits<uint32_t>::max()));
            } else if (arg == "--enable") {
                config.enable_on_boot = true;
            } else if (arg == "--hold-position") {
                config.enable_on_boot = true;
                config.hold_position_on_boot = true;
            } else {
                print_usage();
                return EXIT_FAILURE;
            }
        }

        std::signal(SIGINT, handleSignal);
        std::signal(SIGTERM, handleSignal);

        if (node_ids.empty()) {
            node_ids.push_back(config.node_id);
        }
        if (eds_path.empty()) {
            eds_path = edsPathFromSummary(config.summary_path);
        }

        json object_catalog;
        try {
            object_catalog = loadObjectDictionaryFromEds(eds_path);
        } catch (const std::exception& exception) {
            std::cerr << "warning: " << exception.what()
                      << "; falling back to built-in commissioning object list\n";
            object_catalog = builtinObjects();
        }

        std::cout << "stableCOPS commissioning daemon\n"
                  << "CAN interface: " << config.can_interface << '\n'
                  << "Node IDs: ";
        for (std::size_t i = 0; i < node_ids.size(); ++i) {
            std::cout << static_cast<int>(node_ids[i]) << (i + 1 < node_ids.size() ? "," : "");
        }
        std::cout << '\n'
                  << "Mode at boot: " << stablecops::ds402::toString(*config.operation_mode) << '\n'
                  << "Monitor on boot: yes\n"
                  << "Master DCF: " << config.master_dcf_path << '\n'
                  << "PDO summary: " << config.summary_path << '\n'
                  << "Object dictionary: " << eds_path << " (" << object_catalog.size()
                  << " entries)\n";

        std::vector<std::unique_ptr<stablecops::app::MotorDrive>> drives;
        DriveMap drive_map;
        for (uint8_t node_id : node_ids) {
            stablecops::app::MotorConfig node_config = config;
            node_config.node_id = node_id;
            auto drive = std::make_unique<stablecops::app::MotorDrive>(node_config);
            drive_map[node_id] = drive.get();
            drives.push_back(std::move(drive));
        }

        std::atomic<bool> force_watcher_done{false};
        std::thread force_watcher([&] {
            bool logged = false;
            bool force_sent = false;
            auto force_sent_at = std::chrono::steady_clock::time_point{};
            while (!force_watcher_done.load(std::memory_order_acquire)) {
                const auto signals = g_signal_count.load(std::memory_order_relaxed);
                if (signals >= 3) {
                    std::cerr << "\nthird signal; exiting immediately\n";
                    std::_Exit(128 + SIGINT);
                }
                if (signals >= 2) {
                    if (!logged) {
                        std::cerr << "\nsecond signal; forcing CANopen bus shutdown\n";
                        logged = true;
                    }
                    if (!force_sent) {
                        force_sent = drives.front()->forceStopBus();
                        if (force_sent) {
                            force_sent_at = std::chrono::steady_clock::now();
                        }
                    } else if (std::chrono::steady_clock::now() - force_sent_at >=
                               kForcedShutdownExitDelay) {
                        std::cerr << "forced CANopen shutdown did not complete; exiting process\n";
                        std::_Exit(128 + SIGINT);
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds{50});
            }
        });
        const auto stop_force_watcher = [&] {
            force_watcher_done.store(true, std::memory_order_release);
            if (force_watcher.joinable()) {
                force_watcher.join();
            }
        };

        try {
            drives.front()->start();

            CommissioningApi api(drive_map, config.max_position_step, object_catalog);
            HttpServer server(host, port, api);
            server.run();
            if (g_stop_requested.load(std::memory_order_acquire)) {
                std::cout << "shutdown requested; disabling drives and stopping bus...\n";
            }
            drives.front()->shutdownBus();
            stop_force_watcher();
        } catch (...) {
            drives.front()->forceStopBus();
            drives.front()->shutdownBus();
            stop_force_watcher();
            throw;
        }
        return EXIT_SUCCESS;
    } catch (const std::exception& exception) {
        std::cerr << "stablecops_commissiond: " << exception.what() << '\n';
        return EXIT_FAILURE;
    }
}
