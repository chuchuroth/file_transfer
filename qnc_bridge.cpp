/**
 * qnc_bridge.cpp
 *
 * QNC firmware bridge: subscribes to gripper commands from the robot computer,
 * forwards them to RS485 field devices (Modbus RTU), and publishes gripper
 * state feedback over DDS.
 *
 * DDS topics:
 *   SUBSCRIBE  qnc/gripper/command
 *   PUBLISH    qnc/gripper/state
 *
 * Environment variables:
 *   QNC_SERIAL_PORT      Primary RS485 device   (default: /dev/gripper_ag160)
 *   QNC_SERIAL_PORT_2    Secondary RS485 device (default: /dev/gripper_cgc80)
 *   QNC_SERIAL_PORT_3    Tertiary RS485 device  (default: /dev/gripper_dh56)
 *   QNC_SLAVE_ID_1       DDS slave_id routed to port 1 (default: 1)
 *   QNC_SLAVE_ID_2       DDS slave_id routed to port 2 (default: 2)
 *   QNC_SLAVE_ID_3       DDS slave_id routed to port 3 (default: 3)
 *   QNC_MODBUS_ADDR_1    Modbus wire address for port 1 frames (default: 1)
 *   QNC_MODBUS_ADDR_2    Modbus wire address for port 2 frames (default: 1)
 *   QNC_MODBUS_ADDR_3    Modbus wire address for port 3 frames (default: 1)
 *   QNC_DE_RE_GPIO       sysfs GPIO number for DE/RE line, or "none"
 *                        (default: none — auto-direction XY-485 HAT)
 *   QNC_DOMAIN_ID        FastDDS domain id    (default: 0)
 *   QNC_STATS_INTERVAL   print stats every N write ops (default: 10)
 *
 * Build:
 *   cmake -DBUILD_WITH_DDS=ON .. && cmake --build . -t qnc_bridge
 *
 * Run (udev symlinks installed — no env vars needed):
 *   QNC_DE_RE_GPIO=none ./build/qnc_bridge
 */

#include <algorithm>
#include <atomic>
#include <chrono>
#include <map>
#include <mutex>
#include <set>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <numeric>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <thread>
#include <tuple>
#include <unistd.h>
#include <vector>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <linux/serial.h>

#include <nlohmann/json.hpp>

#include "modbus_rtu_common.hpp"

// Generated from Gripper*.idl
#include "gripper/GripperCommand.hpp"
#include "gripper/GripperCommandPubSubTypes.hpp"
#include "gripper/GripperState.hpp"
#include "gripper/GripperStatePubSubTypes.hpp"

// NeuraSync abstraction layer (replaces raw FastDDS usage)
#include <neura_sync/neura_sync_node.hpp>
#include <neura_sync/participant_factory.hpp>
#include <neura_sync/publisher.hpp>
#include <neura_sync/publisher_factory.hpp>
#include <neura_sync/subscriber.hpp>
#include <neura_sync/subscriber_factory.hpp>

// NeuraSync namespace alias
namespace edds = eprosima::fastdds::dds;
using json = nlohmann::json;

namespace {
constexpr int32_t EC_SUCCESS = 0;
constexpr int32_t EC_TIMEOUT = -2;
constexpr int32_t EC_DISCONNECTED = -3;
} // namespace

// ---------------------------------------------------------------------------
// Signal handling
// ---------------------------------------------------------------------------

static std::atomic<bool> g_running{true};

static void sig_handler(int) { g_running.store(false); }

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

using modbus_rtu::crc16;

static std::string iso_now()
{
    auto now = std::chrono::system_clock::now();
    auto tt  = std::chrono::system_clock::to_time_t(now);
    auto ms  = std::chrono::duration_cast<std::chrono::milliseconds>(
                   now.time_since_epoch()) % 1000;
    char buf[32];
    struct tm tm_info;
    gmtime_r(&tt, &tm_info);
    strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", &tm_info);
    char full[40];
    snprintf(full, sizeof(full), "%s.%03ldZ", buf, (long)ms.count());
    return full;
}

static std::string to_lower_copy(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

static std::vector<std::string> descriptor_aliases(const std::string& raw_id)
{
    std::vector<std::string> out;
    if (raw_id.empty()) {
        return out;
    }

    const std::string id = to_lower_copy(raw_id);
    out.push_back(id);

    const std::string prefix = "gripper_";
    if (id.rfind(prefix, 0) == 0) {
        const std::string no_prefix = id.substr(prefix.size());
        if (!no_prefix.empty()) {
            out.push_back(no_prefix);
        }
    } else {
        out.push_back(prefix + id);
    }

    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
}

// ---------------------------------------------------------------------------
// ModbusRtu
//
// Protocol-agnostic Modbus RTU master over RS485.
// Inherits serial port, GPIO, framing, and transact from modbus_rtu::Base
// (include/modbus_rtu_common.hpp).  Adds per-call slave_id write/read API.
// ---------------------------------------------------------------------------

class ModbusRtu : public modbus_rtu::Base {
public:
    /**
     * Open the serial port at 115200 8N1.
     * Throws std::runtime_error on failure.
     *
     * @param port        serial device (e.g. /dev/ttyAMA0)
     * @param de_re_gpio  sysfs GPIO number for the DE/RE line, or < 0 for auto
     */
    explicit ModbusRtu(const std::string& port, int de_re_gpio = -1)
    {
        port_ = port;
        fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0)
            throw std::runtime_error("Failed to open RS485 port: " + port);

        configure_port();
        if (de_re_gpio >= 0)
            setup_de_re_gpio(de_re_gpio);

        std::cout << "\u2713 RS485 open: " << port << "\n";
    }

    ~ModbusRtu() { close(); }

    ModbusRtu(const ModbusRtu&) = delete;
    ModbusRtu& operator=(const ModbusRtu&) = delete;

    const std::string& port() const { return port_; }

    void close()
    {
        de_re_rx();
        if (de_re_fd_ >= 0) { ::close(de_re_fd_); de_re_fd_ = -1; }
        if (fd_ >= 0)       { ::close(fd_);        fd_ = -1; }
    }

    /**
     * FC06 — Write Single Register.
     *
     * @param slave_id   Modbus slave address (1-247)
     * @param reg_addr   Register address (0x0000-0xFFFF)
     * @param value      16-bit value to write
     * @param timeout_ms Response timeout in ms; 0 = fire-and-forget (no response read)
     * @returns true on success (or when fire_and_forget)
     */
    bool write_register(uint8_t slave_id, uint16_t reg_addr, uint16_t value,
                        int timeout_ms = 500)
    {
        std::vector<uint8_t> pdu = {
            slave_id,
            0x06,
            static_cast<uint8_t>(reg_addr >> 8),
            static_cast<uint8_t>(reg_addr & 0xFF),
            static_cast<uint8_t>(value >> 8),
            static_cast<uint8_t>(value & 0xFF)
        };
        auto frame = build_frame(pdu);

        if (timeout_ms == 0) {
            tcflush(fd_, TCIOFLUSH);
            de_re_tx();
            if (::write(fd_, frame.data(), frame.size()) < 0) { /* best effort */ }
            tcdrain(fd_);
            usleep(500);
            de_re_rx();
            return true;
        }

        std::vector<uint8_t> resp = transact(frame, 8, timeout_ms);
        return validate_response(resp, slave_id, 0x06);
    }

    /**
     * FC04 — Read Input Registers.
     */
    bool read_input_registers(uint8_t slave_id, uint16_t start_reg,
                               uint16_t count, std::vector<uint16_t>& out,
                               int timeout_ms = 1000)
    {
        return read_registers(slave_id, 0x04, start_reg, count, out, timeout_ms);
    }

    /**
     * FC03 — Read Holding Registers.
     */
    bool read_holding_registers(uint8_t slave_id, uint16_t start_reg,
                                 uint16_t count, std::vector<uint16_t>& out,
                                 int timeout_ms = 1000)
    {
        return read_registers(slave_id, 0x03, start_reg, count, out, timeout_ms);
    }

private:
    bool read_registers(uint8_t slave_id, uint8_t fc, uint16_t start_reg,
                        uint16_t count, std::vector<uint16_t>& out, int timeout_ms)
    {
        std::vector<uint8_t> pdu = {
            slave_id, fc,
            static_cast<uint8_t>(start_reg >> 8),
            static_cast<uint8_t>(start_reg & 0xFF),
            static_cast<uint8_t>(count >> 8),
            static_cast<uint8_t>(count & 0xFF)
        };
        auto frame = build_frame(pdu);

        size_t expected = 3 + count * 2 + 2;
        std::vector<uint8_t> resp = transact(frame, expected, timeout_ms);

        if (!validate_response(resp, slave_id, fc))
            return false;
        if (resp.size() < expected)
            return false;

        out.clear();
        out.reserve(count);
        for (uint16_t i = 0; i < count; ++i)
            out.push_back((static_cast<uint16_t>(resp[3 + i*2]) << 8)
                          | resp[4 + i*2]);
        return true;
    }
};

struct RegisterRef {
    uint16_t address{0};
    uint8_t fc{6};
};

struct ScaleRule {
    double raw_min{0.0};
    double raw_max{1.0};
    bool invert{false};
};

struct GripperDescriptor {
    std::string device_id;
    uint8_t slave_id{1};
    bool supports_force_control{false};
    bool supports_speed_control{false};
    bool has_multi_joint{false};
    size_t dof_count{1};

    std::vector<RegisterRef> position_write;
    std::vector<RegisterRef> position_read;
    RegisterRef status_motion;
    RegisterRef status_grasp;
    RegisterRef status_fault;
    uint16_t status_base{0};
    uint16_t status_stride{1};
    size_t status_dof_count{0};

    std::vector<RegisterRef> force_write;
    std::vector<RegisterRef> speed_write;

    ScaleRule position_scale;
    ScaleRule force_scale;
    ScaleRule speed_scale;
};

class DescriptorLoader {
public:
    static std::map<std::string, GripperDescriptor> load(const std::string& path)
    {
        std::ifstream ifs(path);
        if (!ifs.is_open()) {
            throw std::runtime_error("Failed to open gripper descriptor: " + path);
        }
        json j;
        ifs >> j;

        GripperDescriptor d;
        const auto& dev = j.at("device");
        if (dev.contains("device_id")) {
            d.device_id = dev.at("device_id").get<std::string>();
        } else if (dev.contains("model")) {
            d.device_id = dev.at("model").get<std::string>();
        } else {
            throw std::runtime_error("Descriptor missing device.device_id/model");
        }

        if (j.contains("connection") && j.at("connection").contains("slave_id")) {
            d.slave_id = static_cast<uint8_t>(j.at("connection").at("slave_id").get<int>());
        } else if (dev.contains("slave_id")) {
            d.slave_id = static_cast<uint8_t>(dev.at("slave_id").get<int>());
        }

        if (j.contains("capabilities")) {
            const auto& caps = j.at("capabilities");
            d.supports_force_control = caps.value("supports_force_control", false);
            d.supports_speed_control = caps.value("supports_speed_control", false);
            d.has_multi_joint = caps.value("has_multi_joint", false);
            d.dof_count = static_cast<size_t>(caps.value("joint_count", d.has_multi_joint ? 6 : 1));
        }

        // New schema: explicit joints[] array with per-DOF mappings.
        if (j.contains("joints") && j.at("joints").is_array()) {
            for (const auto& joint : j.at("joints")) {
                if (joint.contains("position")) {
                    if (joint.at("position").contains("write")) {
                        d.position_write.push_back(parse_reg(joint.at("position").at("write")));
                    }
                    if (joint.at("position").contains("read")) {
                        d.position_read.push_back(parse_reg(joint.at("position").at("read")));
                    }
                    if (joint.at("position").contains("scale")) {
                        d.position_scale = parse_scale(joint.at("position").at("scale"));
                    }
                }

                if (joint.contains("force") && joint.at("force").contains("write")) {
                    d.force_write.push_back(parse_reg(joint.at("force").at("write")));
                    if (joint.at("force").contains("scale")) {
                        d.force_scale = parse_scale(joint.at("force").at("scale"));
                    }
                }

                if (joint.contains("speed") && joint.at("speed").contains("write")) {
                    d.speed_write.push_back(parse_reg(joint.at("speed").at("write")));
                    if (joint.at("speed").contains("scale")) {
                        d.speed_scale = parse_scale(joint.at("speed").at("scale"));
                    }
                }
            }

            if (j.contains("normalization")) {
                const auto& norm = j.at("normalization");
                if (norm.contains("position")) d.position_scale = parse_scale(norm.at("position"));
                if (norm.contains("force")) d.force_scale = parse_scale(norm.at("force"));
                if (norm.contains("speed")) d.speed_scale = parse_scale(norm.at("speed"));
            }

            if (j.contains("register_map") && j.at("register_map").contains("status")) {
                const auto& st = j.at("register_map").at("status");
                if (st.contains("joint_state") && st.at("joint_state").is_array() && !st.at("joint_state").empty()) {
                    d.status_base = static_cast<uint16_t>(st.at("joint_state").at(0).at("address").get<int>());
                    d.status_dof_count = st.at("joint_state").size();
                    if (st.at("joint_state").size() > 1) {
                        const int first = st.at("joint_state").at(0).at("address").get<int>();
                        const int second = st.at("joint_state").at(1).at("address").get<int>();
                        d.status_stride = static_cast<uint16_t>(std::max(1, second - first));
                    }
                    d.status_motion = parse_reg(st.at("joint_state").at(0));
                    d.status_grasp = d.status_motion;
                }
            }
        }

        // Legacy schema fallback.
        if (d.position_write.empty() && j.contains("register_map")) {
            const auto& regs = j.at("register_map");
            if (regs.contains("position") && regs.at("position").contains("write")) {
                for (const auto& r : regs.at("position").at("write")) {
                    d.position_write.push_back(parse_reg(r));
                }
            }
            if (regs.contains("position") && regs.at("position").contains("read")) {
                for (const auto& r : regs.at("position").at("read")) {
                    d.position_read.push_back(parse_reg(r));
                }
            }
            if (regs.contains("status")) {
                if (regs.at("status").contains("motion")) d.status_motion = parse_reg(regs.at("status").at("motion"));
                if (regs.at("status").contains("grasp")) d.status_grasp = parse_reg(regs.at("status").at("grasp"));
                if (regs.at("status").contains("fault")) d.status_fault = parse_reg(regs.at("status").at("fault"));
            }

            if (regs.contains("force") && regs.at("force").contains("write")) {
                for (const auto& r : regs.at("force").at("write")) {
                    d.force_write.push_back(parse_reg(r));
                }
            }
            if (regs.contains("speed") && regs.at("speed").contains("write")) {
                for (const auto& r : regs.at("speed").at("write")) {
                    d.speed_write.push_back(parse_reg(r));
                }
            }

            if (j.contains("scaling")) {
                const auto& sc = j.at("scaling");
                if (sc.contains("position")) d.position_scale = parse_scale(sc.at("position"));
                if (sc.contains("force")) d.force_scale = parse_scale(sc.at("force"));
                if (sc.contains("speed")) d.speed_scale = parse_scale(sc.at("speed"));
            }
        }

        if (d.position_write.empty()) {
            throw std::runtime_error("Descriptor has no position write registers");
        }
        if (d.position_read.empty()) {
            d.position_read.push_back(d.position_write.front());
        }
        if (d.status_grasp.address == 0 && d.status_motion.address != 0) {
            d.status_grasp = d.status_motion;
        }
        if (d.dof_count == 0) {
            d.dof_count = std::max<size_t>(1, d.position_write.size());
        }
        if (!d.has_multi_joint && d.dof_count > 1) {
            d.has_multi_joint = true;
        }

        std::map<std::string, GripperDescriptor> by_id;
        for (const auto& alias : descriptor_aliases(d.device_id)) {
            by_id[alias] = d;
        }
        return by_id;
    }

private:
    static RegisterRef parse_reg(const json& j)
    {
        RegisterRef r;
        r.address = static_cast<uint16_t>(j.at("address").get<int>());
        r.fc = static_cast<uint8_t>(j.at("fc").get<int>());
        return r;
    }

    static ScaleRule parse_scale(const json& j)
    {
        ScaleRule s;
        s.raw_min = j.at("raw_min").get<double>();
        s.raw_max = j.at("raw_max").get<double>();
        if (j.contains("invert")) {
            s.invert = j.at("invert").get<bool>();
        }
        return s;
    }
};

class CommandMapper {
public:
    struct WriteAction {
        uint16_t reg{0};
        uint16_t value{0};
        uint32_t timeout_ms{500};
    };

    static std::vector<WriteAction> map(
        const qnc::gripper::GripperCommand& cmd,
        const GripperDescriptor& d)
    {
        const auto [ovr_pos, ovr_force, ovr_speed] = parse_overrides(cmd.tag().c_str());
        const size_t dof = std::max<size_t>(1, d.dof_count);
        const auto pos = expand(ovr_pos, cmd.position(), dof);
        const auto force = expand(ovr_force, cmd.max_force(), dof);
        const auto speed = expand(ovr_speed, cmd.max_speed(), dof);

        std::vector<WriteAction> out;
        for (size_t i = 0; i < std::min(d.position_write.size(), dof); ++i) {
            out.push_back(make_write_action(d.position_write[i], to_raw(pos[i], d.position_scale), cmd));
        }

        if (d.supports_force_control) {
            for (size_t i = 0; i < std::min(d.force_write.size(), dof); ++i) {
                out.push_back(make_write_action(d.force_write[i], to_raw(force[i], d.force_scale), cmd));
            }
        }
        if (d.supports_speed_control) {
            for (size_t i = 0; i < std::min(d.speed_write.size(), dof); ++i) {
                out.push_back(make_write_action(d.speed_write[i], to_raw(speed[i], d.speed_scale), cmd));
            }
        }
        return out;
    }

private:
    static WriteAction make_write_action(
        const RegisterRef& reg,
        uint16_t value,
        const qnc::gripper::GripperCommand& cmd)
    {
        WriteAction w;
        w.reg = reg.address;
        w.value = value;
        w.timeout_ms = cmd.timeout() > 0.0f ? static_cast<uint32_t>(cmd.timeout() * 1000.0f) : 500;
        return w;
    }

    static uint16_t to_raw(float norm, const ScaleRule& s)
    {
        const double n = s.invert ? (1.0 - clamp01(norm)) : clamp01(norm);
        return static_cast<uint16_t>(s.raw_min + n * (s.raw_max - s.raw_min));
    }

    static double clamp01(double v)
    {
        return std::max(0.0, std::min(1.0, v));
    }

    static std::vector<float> expand(
        const std::optional<std::vector<float>>& maybe_vec,
        float fallback_scalar,
        size_t n)
    {
        if (maybe_vec.has_value()) {
            if (maybe_vec->size() == n) {
                return *maybe_vec;
            }
            if (maybe_vec->size() == 1) {
                return std::vector<float>(n, maybe_vec->front());
            }
        }
        return std::vector<float>(n, fallback_scalar);
    }

    // Assumption: GripperCommand has no dedicated JSON payload field, so
    // advanced overrides are optionally encoded in `tag` as a JSON object.
    static std::tuple<std::optional<std::vector<float>>, std::optional<std::vector<float>>, std::optional<std::vector<float>>>
    parse_overrides(const std::string& tag)
    {
        if (tag.empty() || tag.front() != '{') {
            return {std::nullopt, std::nullopt, std::nullopt};
        }
        try {
            json j = json::parse(tag);
            std::optional<std::vector<float>> p, f, s;
            if (j.contains("position")) p = parse_float_or_array(j.at("position"));
            if (j.contains("force")) f = parse_float_or_array(j.at("force"));
            if (j.contains("speed")) s = parse_float_or_array(j.at("speed"));
            return {p, f, s};
        } catch (...) {
            return {std::nullopt, std::nullopt, std::nullopt};
        }
    }

    static std::vector<float> parse_float_or_array(const json& value)
    {
        if (value.is_number()) {
            return {value.get<float>()};
        }
        if (!value.is_array()) {
            return {};
        }
        std::vector<float> out;
        out.reserve(value.size());
        for (const auto& it : value) {
            out.push_back(it.get<float>());
        }
        return out;
    }
};

class StateMapper {
public:
    static qnc::gripper::GripperState map(
        const GripperDescriptor& d,
        uint16_t raw_pos,
        uint16_t raw_motion,
        uint16_t raw_grasp,
        int32_t err,
        uint32_t latency_us,
        const std::string& tag)
    {
        qnc::gripper::GripperState st;
        st.device_id(d.device_id.c_str());
        st.position(static_cast<float>(to_norm(raw_pos, d.position_scale)));
        st.force(0.0f);
        st.speed(0.0f);
        st.status(err < 0
            ? qnc::gripper::GripperStatus::STATUS_ERROR
            : (raw_motion == 0 ? qnc::gripper::GripperStatus::STATUS_MOVING
                               : qnc::gripper::GripperStatus::STATUS_IDLE));
        st.grasp_result(raw_grasp == 2
            ? qnc::gripper::GraspResult::GRASP_SUCCESS
            : qnc::gripper::GraspResult::GRASP_IN_PROGRESS);
        st.error_code(err);
        st.message(err < 0 ? "modbus/read error" : "ok");
        st.last_command_tag(tag.c_str());
        st.last_command_latency_us(latency_us);
        st.timestamp(iso_now());
        return st;
    }

private:
    static double to_norm(uint16_t raw, const ScaleRule& s)
    {
        const double denom = s.raw_max - s.raw_min;
        if (denom == 0.0) return 0.0;
        const double n = (static_cast<double>(raw) - s.raw_min) / denom;
        const double out = s.invert ? (1.0 - n) : n;
        return std::max(0.0, std::min(1.0, out));
    }
};

// ---------------------------------------------------------------------------
// QncBridge
//
// Subscribes to DDS write_cmd / read_cmd, forwards to ModbusRtu,
// publishes response + stats.
//
// Dual-port support: two ModbusRtu instances, each mapped to a slave_id.
// Commands arriving with slave_id == slave_id1_ go to modbus1_;
// commands with slave_id == slave_id2_ go to modbus2_.
// ---------------------------------------------------------------------------

class QncBridge {
public:
    /**
     * @param modbus1         Primary RS485 interface (nullptr = DDS-only)
     * @param slave_id1       DDS slave_id routed to modbus1_
     * @param modbus_addr1    Modbus wire address written into RS485 frames for port1
     * @param modbus2         Secondary RS485 interface (nullptr = unused)
     * @param slave_id2       DDS slave_id routed to modbus2_
     * @param modbus_addr2    Modbus wire address written into RS485 frames for port2
     * @param modbus3         Tertiary RS485 interface (nullptr = unused)
     * @param slave_id3       DDS slave_id routed to modbus3_
     * @param modbus_addr3    Modbus wire address written into RS485 frames for port3
     * @param domain_id       FastDDS domain (default 0)
     * @param stats_interval  publish stats every N write operations
     *
     * DDS slave_id and Modbus wire address are intentionally separate:
     *   - slave_id selects WHICH port to use (routing)
     *   - modbus_addr is the address byte in the RS485 frame
     * All grippers typically have Modbus address 1 on the wire even though
     * they are addressed as slave_id 1, 2, and 3 in DDS.
     */
    QncBridge(ModbusRtu* modbus1, uint8_t slave_id1, uint8_t modbus_addr1,
              ModbusRtu* modbus2, uint8_t slave_id2, uint8_t modbus_addr2,
              ModbusRtu* modbus3, uint8_t slave_id3, uint8_t modbus_addr3,
              int domain_id = 0, int stats_interval = 10,
              int init_wait_ms1 = 5500,   // AG-160: 160 mm stroke ~4.5 s calibration
              int init_wait_ms2 = 3500,   // CGC-80:  95 mm stroke ~2.5 s calibration
              int init_wait_ms3 = 3500,   // DH-5-6
              const std::string& gripper_descriptor = "",
              bool dry_run = false)
        : modbus1_(modbus1), slave_id1_(slave_id1), modbus_addr1_(modbus_addr1),
          modbus2_(modbus2), slave_id2_(slave_id2), modbus_addr2_(modbus_addr2),
          modbus3_(modbus3), slave_id3_(slave_id3), modbus_addr3_(modbus_addr3),
            stats_interval_(stats_interval),
            dry_run_(dry_run)
    {
        if (modbus1_) port_init_wait_ms_[modbus1_] = init_wait_ms1;
        if (modbus2_) port_init_wait_ms_[modbus2_] = init_wait_ms2;
        if (modbus3_) port_init_wait_ms_[modbus3_] = init_wait_ms3;

        // Create DomainParticipant via NeuraSync ParticipantFactory
        auto* part_factory = neura::sync::ParticipantFactory::getInstance();
        participant_ = part_factory->createDefaultParticipant(domain_id);
        if (!participant_)
            throw std::runtime_error("NeuraSync: failed to create DomainParticipant");

        // NeuraSyncNode manages the lifecycle of all publishers/subscribers.
        node_ = std::make_shared<neura::sync::NeuraSyncNode>(participant_);

        auto pub_factory = neura::sync::PublisherFactory::getInstance(participant_);
        auto sub_factory = neura::sync::SubscriberFactory::getInstance(participant_);

        // Generic gripper command ingress on the fixed high-level topic.
        // Use CriticalData (RELIABLE) to match all upstream command publishers
        // (gripper_control_dds, gripper_runtime_validator, cgc80_validation all
        // publish with createCriticalDataPublisher/RELIABLE).  A BEST_EFFORT
        // subscription would still match, but commands could be silently lost.
        gc_sub_ = sub_factory->createCriticalDataSubscriber<
                qnc::gripper::GripperCommand, qnc::gripper::GripperCommandPubSubType>(
                "gc_sub", "qnc/gripper/command",
                [this](const qnc::gripper::GripperCommand& msg) {
                    std::lock_guard<std::mutex> lk(cmd_mutex_);
                    gripper_cmd_queue_.push_back(msg);
                });
        node_->add_subscriber(gc_sub_);

        // State publisher: CriticalData profile (RELIABLE, SYNC).
        constexpr int64_t max_blocking_ns = 500000000; // 500 ms
        node_->add_publisher(
            pub_factory->createCriticalDataPublisher<
                qnc::gripper::GripperState, qnc::gripper::GripperStatePubSubType>(
                "gs_pub", "qnc/gripper/state", max_blocking_ns));

        if (!gripper_descriptor.empty()) {
            try {
                gripper_descriptors_ = DescriptorLoader::load(gripper_descriptor);
                std::cout << "[Bridge] Loaded gripper descriptor: " << gripper_descriptor << "\n";
            } catch (const std::exception& e) {
                std::cerr << "[Bridge] gripper descriptor load failed: " << e.what() << "\n";
            }
        }

        start_time_ = std::chrono::steady_clock::now();

        std::cout << "[NeuraSync] QncBridge active on domain " << domain_id << "\n"
                  << "      SUB: qnc/gripper/command\n"
                  << "      PUB: qnc/gripper/state\n";
        if (modbus1_)
            std::cout << "      RS485 port1 dds_slave_id=" << (int)slave_id1_
                      << "  modbus_wire_addr=" << (int)modbus_addr1_ << "\n";
        if (modbus2_)
            std::cout << "      RS485 port2 dds_slave_id=" << (int)slave_id2_
                      << "  modbus_wire_addr=" << (int)modbus_addr2_ << "\n";
        if (modbus3_)
            std::cout << "      RS485 port3 dds_slave_id=" << (int)slave_id3_
                      << "  modbus_wire_addr=" << (int)modbus_addr3_ << "\n";
    }

    ~QncBridge()
    {
        // NeuraSyncNode destructor cleans up all publishers, subscribers,
        // and associated DDS entities and participant ownership.
        node_.reset();
        participant_ = nullptr;
    }

    QncBridge(const QncBridge&) = delete;
    QncBridge& operator=(const QncBridge&) = delete;

    /**
    * Block until the robot platform's gripper command publisher is matched.
     * DDS peer discovery is complete) or until timeout_ms elapses.
     * Returns true if matched, false on timeout.
     */
    bool wait_for_publisher_match(int timeout_ms = 10000)
    {
        const int poll_ms  = 100;
        int       elapsed  = 0;
        std::cout << "[Bridge] Waiting for robot publisher match..." << std::flush;

        // Access the gripper command subscriber's DataReader to check for matched
        // publications (peer discovery).
        auto sub_base = gc_sub_;
        if (!sub_base || !sub_base->get_data_reader()) {
            std::cout << " subscriber not found\n" << std::flush;
            return false;
        }

        while (elapsed < timeout_ms) {
            edds::SubscriptionMatchedStatus status;
            sub_base->get_data_reader()->get_subscription_matched_status(status);
            if (status.current_count > 0) {
                std::cout << " matched (" << elapsed << " ms)\n" << std::flush;
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(poll_ms));
            elapsed += poll_ms;
        }
        std::cout << " timeout (" << timeout_ms << " ms) — proceeding anyway\n" << std::flush;
        return false;
    }

    /**
    * Block until g_running is false, processing all incoming gripper commands.
     */
    void spin()
    {
        std::cout << "[Bridge] Waiting for commands (Ctrl-C to stop)...\n" << std::flush;

        while (g_running.load()) {
            bool processed = false;
            processed |= drain_gripper_commands();

            if (!processed)
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        publish_stats("shutdown");
        std::cout << "\n[Bridge] Stopped. tx=" << tx_count_
                  << " rx=" << rx_count_
                  << " err=" << err_count_ << "\n";
    }

private:
    // --- NeuraSync plumbing -------------------------------------------------
    edds::DomainParticipant* participant_ = nullptr;
    std::shared_ptr<neura::sync::NeuraSyncNode> node_;
    std::shared_ptr<neura::sync::Subscriber<
        qnc::gripper::GripperCommand, qnc::gripper::GripperCommandPubSubType>> gc_sub_;

    // Callback->queue bridge: callbacks happen on DDS listener threads;
    // serial IO remains single-threaded in spin().
    std::mutex cmd_mutex_;
    std::vector<qnc::gripper::GripperCommand> gripper_cmd_queue_;

    std::map<std::string, GripperDescriptor> gripper_descriptors_;

    // --- State --------------------------------------------------------------
    ModbusRtu*  modbus1_      = nullptr;  // primary port
    uint8_t     slave_id1_    = 1;        // DDS slave_id routing key for port1
    uint8_t     modbus_addr1_ = 1;        // Modbus wire address for port1 frames
    ModbusRtu*  modbus2_      = nullptr;  // secondary port (optional)
    uint8_t     slave_id2_    = 2;        // DDS slave_id routing key for port2
    uint8_t     modbus_addr2_ = 1;        // Modbus wire address for port2 frames
    ModbusRtu*  modbus3_      = nullptr;  // tertiary port (optional)
    uint8_t     slave_id3_    = 3;        // DDS slave_id routing key for port3
    uint8_t     modbus_addr3_ = 1;        // Modbus wire address for port3 frames
    int         stats_interval_  = 10;
    uint64_t    tx_count_        = 0;
    uint64_t    rx_count_        = 0;
    uint64_t    err_count_       = 0;
    long        last_err_        = 0;
    std::chrono::steady_clock::time_point start_time_;
    bool        dry_run_         = false;

    // Per-port regular-init (val=1) guard duration in ms.
    std::map<ModbusRtu*, int> port_init_wait_ms_;

    std::pair<ModbusRtu*, uint8_t> route(uint8_t dds_slave_id) const
    {
        if (modbus1_ && dds_slave_id == slave_id1_) return {modbus1_, modbus_addr1_};
        if (modbus2_ && dds_slave_id == slave_id2_) return {modbus2_, modbus_addr2_};
        if (modbus3_ && dds_slave_id == slave_id3_) return {modbus3_, modbus_addr3_};
        return {nullptr, 0};
    }

    bool drain_gripper_commands()
    {
        std::vector<qnc::gripper::GripperCommand> batch;
        {
            std::lock_guard<std::mutex> lk(cmd_mutex_);
            batch.swap(gripper_cmd_queue_);
        }
        bool any = false;

        for (const auto& cmd : batch) {
            any = true;
            const std::string device_id(cmd.device_id().c_str());

            // Raw transport mode: execute register writes/reads directly from
            // GripperCommand transport fields, bypassing descriptor mapping.
            if (cmd.command_type() == qnc::gripper::GripperCommandType::CMD_RAW_MODBUS
                || !cmd.raw_writes().empty()
                || cmd.raw_read_count() > 0)
            {
                uint8_t dds_slave = static_cast<uint8_t>(cmd.slave_id_hint());
                if (dds_slave == 0) {
                    auto it = gripper_descriptors_.find(device_id);
                    if (it != gripper_descriptors_.end()) {
                        dds_slave = it->second.slave_id;
                    }
                }

                auto [bus, wire_addr] = route(dds_slave);
                const char* port_label = (bus == modbus1_) ? "port1" :
                                         (bus == modbus2_) ? "port2" :
                                         (bus == modbus3_) ? "port3" : "NONE";
                std::cout << "[TRACE] device_id=" << device_id
                          << "  dds_slave=" << (int)dds_slave
                          << "  → " << port_label
                          << "  wire_addr=" << (int)wire_addr
                          << "  writes=" << cmd.raw_writes().size()
                          << "  read_count=" << cmd.raw_read_count()
                          << "  read_fc=" << (int)cmd.raw_read_fc()
                          << "  tag=" << cmd.tag() << "\n" << std::flush;
                if (!bus) {
                    if (dry_run_) {
                        qnc::gripper::GripperState st;
                        st.device_id(cmd.device_id());
                        st.last_command_type_code(static_cast<int32_t>(cmd.command_type()));
                        st.status(qnc::gripper::GripperStatus::STATUS_IDLE);
                        st.grasp_result(qnc::gripper::GraspResult::GRASP_UNKNOWN);
                        st.error_code(EC_SUCCESS);
                        st.bridge_error_code(EC_SUCCESS);
                        st.message("dry_run_no_bus");
                        st.bridge_error_message("");
                        st.last_command_tag(cmd.tag());
                        st.timestamp(iso_now());
                        publish_gripper_state(st);
                        continue;
                    }
                    qnc::gripper::GripperState st;
                    st.device_id(cmd.device_id());
                    st.last_command_type_code(static_cast<int32_t>(cmd.command_type()));
                    st.status(qnc::gripper::GripperStatus::STATUS_ERROR);
                    st.grasp_result(qnc::gripper::GraspResult::GRASP_FAILED_ERROR);
                    st.error_code(EC_DISCONNECTED);
                    st.bridge_error_code(EC_DISCONNECTED);
                    st.message("No route for slave_id");
                    st.bridge_error_message("No port configured for requested slave_id");
                    st.last_command_tag(cmd.tag());
                    st.timestamp(iso_now());
                    publish_gripper_state(st);
                    continue;
                }

                int32_t err = EC_SUCCESS;
                for (const auto& w : cmd.raw_writes()) {
                    ++tx_count_;
                    const bool ok = bus->write_register(wire_addr, w.register_address(), w.value(), 500);
                    std::cout << "[TRACE]   WRITE " << port_label
                              << " addr=" << (int)wire_addr
                              << " reg=0x" << std::hex << w.register_address() << std::dec
                              << " val=" << w.value()
                              << " → " << (ok ? "OK" : "FAIL") << "\n" << std::flush;
                    if (ok) {
                        ++rx_count_;
                    } else {
                        err = EC_TIMEOUT;
                        ++err_count_;
                        last_err_ = err;
                    }
                }

                std::vector<uint16_t> regs;
                if (cmd.raw_read_count() > 0) {
                    const uint16_t count = std::min<uint16_t>(cmd.raw_read_count(), 125);
                    // Enforce FC03 for bridge-level reads to keep transport behavior
                    // stable across devices that misreport FC04 support.
                    const bool read_ok = bus->read_holding_registers(
                        wire_addr, cmd.raw_read_start_register(), count, regs, 1000);
                    std::cout << "[TRACE]   READ " << port_label
                              << " addr=" << (int)wire_addr
                              << " reg=0x" << std::hex << cmd.raw_read_start_register() << std::dec
                              << " count=" << count
                              << " fc=3"
                              << " → " << (read_ok ? "OK" : "FAIL")
                              << " regs=[";
                    for (size_t ri = 0; ri < regs.size(); ++ri) {
                        if (ri) std::cout << ",";
                        std::cout << regs[ri];
                    }
                    std::cout << "]\n" << std::flush;
                    if (read_ok) {
                        ++rx_count_;
                    } else {
                        err = EC_TIMEOUT;
                        ++err_count_;
                        last_err_ = err;
                    }
                }

                qnc::gripper::GripperState st;
                st.device_id(cmd.device_id());
                st.last_command_type_code(static_cast<int32_t>(cmd.command_type()));
                st.position(cmd.position());
                st.force(cmd.max_force());
                st.speed(cmd.max_speed());
                st.status(err == EC_SUCCESS
                    ? qnc::gripper::GripperStatus::STATUS_IDLE
                    : qnc::gripper::GripperStatus::STATUS_ERROR);
                st.grasp_result(err == EC_SUCCESS
                    ? qnc::gripper::GraspResult::GRASP_UNKNOWN
                    : qnc::gripper::GraspResult::GRASP_FAILED_ERROR);
                st.error_code(err);
                st.bridge_error_code(err);
                st.message(err == EC_SUCCESS ? "raw_modbus_ok" : "raw_modbus_error");
                st.bridge_error_message(err == EC_SUCCESS ? "" : "raw transport read/write timeout");
                st.last_command_tag(cmd.tag());
                st.slave_id(dds_slave);
                st.last_command_latency_us(0);
                st.bridge_latency_us(0);
                st.has_transport_snapshot(cmd.raw_read_count() > 0);
                st.transport_snapshot().start_register(cmd.raw_read_start_register());
                st.transport_snapshot().data(regs);
                st.timestamp(iso_now());
                publish_gripper_state(st);
                continue;
            }

            const auto find_descriptor = [&](const std::string& id)
                -> std::map<std::string, GripperDescriptor>::const_iterator {
                for (const auto& key : descriptor_aliases(id)) {
                    auto it = gripper_descriptors_.find(key);
                    if (it != gripper_descriptors_.end()) {
                        return it;
                    }
                }
                return gripper_descriptors_.end();
            };

            auto dit = find_descriptor(device_id);
            if (dit == gripper_descriptors_.end()) {
                std::cerr << "[Gripper] descriptor not found for device_id=" << device_id << "\n";
                if (dry_run_) {
                    qnc::gripper::GripperState st;
                    st.device_id(cmd.device_id());
                    st.last_command_type_code(static_cast<int32_t>(cmd.command_type()));
                    st.status(qnc::gripper::GripperStatus::STATUS_IDLE);
                    st.grasp_result(qnc::gripper::GraspResult::GRASP_UNKNOWN);
                    st.error_code(EC_SUCCESS);
                    st.bridge_error_code(EC_SUCCESS);
                    st.message("dry_run_no_descriptor");
                    st.bridge_error_message("");
                    st.last_command_tag(cmd.tag());
                    st.timestamp(iso_now());
                    publish_gripper_state(st);
                    continue;
                }
                // Publish an explicit error state so subscribers (e.g. gripper_runtime_validator)
                // do not time out waiting for feedback that would never arrive.
                qnc::gripper::GripperState err_st;
                err_st.device_id(cmd.device_id());
                err_st.last_command_type_code(static_cast<int32_t>(cmd.command_type()));
                err_st.status(qnc::gripper::GripperStatus::STATUS_ERROR);
                err_st.grasp_result(qnc::gripper::GraspResult::GRASP_FAILED_ERROR);
                err_st.error_code(EC_DISCONNECTED);
                err_st.bridge_error_code(EC_DISCONNECTED);
                err_st.message("no descriptor for device_id");
                err_st.bridge_error_message("load a gripper descriptor via --descriptor flag");
                err_st.last_command_tag(cmd.tag());
                err_st.timestamp(iso_now());
                publish_gripper_state(err_st);
                ++err_count_;
                continue;
            }
            const auto& desc = dit->second;
            auto [bus, wire_addr] = route(desc.slave_id);
            if (!bus) {
                if (dry_run_) {
                    qnc::gripper::GripperState st;
                    st.device_id(cmd.device_id());
                    st.last_command_type_code(static_cast<int32_t>(cmd.command_type()));
                    st.status(qnc::gripper::GripperStatus::STATUS_IDLE);
                    st.grasp_result(qnc::gripper::GraspResult::GRASP_UNKNOWN);
                    st.error_code(EC_SUCCESS);
                    st.bridge_error_code(EC_SUCCESS);
                    st.message("dry_run_no_route");
                    st.bridge_error_message("");
                    st.last_command_tag(cmd.tag());
                    st.timestamp(iso_now());
                    publish_gripper_state(st);
                    continue;
                }
                std::cerr << "[Gripper] no route for slave_id=" << (int)desc.slave_id << "\n";
                continue;
            }

            const auto writes = CommandMapper::map(cmd, desc);
            std::cout << "[MAP] device_id=" << device_id
                      << " dof_count=" << desc.dof_count
                      << " writes=" << writes.size()
                      << " has_multi_joint=" << (desc.has_multi_joint ? "true" : "false")
                      << " tag=" << cmd.tag() << "\n";
            for (const auto& w : writes) {
                std::cout << "[MAP]   FC06 reg=0x" << std::hex << w.reg << std::dec
                          << " value=" << w.value << "\n";
                bus->write_register(wire_addr, w.reg, w.value, w.timeout_ms);
                ++tx_count_;
            }

            std::vector<uint16_t> pos_values;
            std::vector<uint16_t> dof_states;
            int32_t err = EC_SUCCESS;

            // Enforce FC03 on descriptor-driven reads.
            for (const auto& pref : desc.position_read) {
                std::vector<uint16_t> one;
                if (bus->read_holding_registers(wire_addr, pref.address, 1, one, 800) && !one.empty()) {
                    pos_values.push_back(one[0]);
                } else {
                    err = EC_TIMEOUT;
                }
            }

            if (desc.has_multi_joint && desc.status_dof_count > 0) {
                for (size_t i = 0; i < desc.status_dof_count; ++i) {
                    std::vector<uint16_t> one;
                    const uint16_t addr = static_cast<uint16_t>(desc.status_base + i * desc.status_stride);
                    if (bus->read_holding_registers(wire_addr, addr, 1, one, 800) && !one.empty()) {
                        dof_states.push_back(one[0]);
                    } else {
                        err = EC_TIMEOUT;
                    }
                }
            } else {
                std::vector<uint16_t> one;
                if (bus->read_holding_registers(wire_addr, desc.status_motion.address, 1, one, 800) && !one.empty()) {
                    dof_states.push_back(one[0]);
                } else {
                    err = EC_TIMEOUT;
                }
            }

            if (err != EC_SUCCESS) {
                ++err_count_;
                last_err_ = err;
            } else {
                ++rx_count_;
            }

            const uint16_t raw_pos = pos_values.empty()
                ? 0
                : static_cast<uint16_t>(std::accumulate(pos_values.begin(), pos_values.end(), 0u) / pos_values.size());

            bool any_moving = false;
            bool all_at_target = !dof_states.empty();
            bool any_error = false;
            for (uint16_t s : dof_states) {
                // Typical DH56 semantics: 0=moving, 1=at target/idle, 2=contact, 3=error.
                if (s == 0) any_moving = true;
                if (!(s == 1 || s == 2)) all_at_target = false;
                if (s >= 3) any_error = true;
            }
            const uint16_t raw_motion = any_moving ? 0 : 1;
            const uint16_t raw_grasp = any_error ? 0 : (all_at_target ? 2 : 1);

            std::cout << "[MAP]   status_dofs=" << dof_states.size()
                      << " any_moving=" << (any_moving ? "true" : "false")
                      << " any_error=" << (any_error ? "true" : "false")
                      << " all_at_target=" << (all_at_target ? "true" : "false")
                      << " avg_pos_raw=" << raw_pos << "\n";

            auto state = StateMapper::map(
                desc,
                raw_pos,
                raw_motion,
                raw_grasp,
                err,
                0,
                std::string(cmd.tag().c_str()));
            publish_gripper_state(state);

            if (tx_count_ % static_cast<uint64_t>(stats_interval_) == 0) {
                publish_stats("periodic");
            }
        }
        return any;
    }

    // --- Publish helpers ----------------------------------------------------

    void publish_gripper_state(const qnc::gripper::GripperState& state)
    {
        if (!node_) return;
        node_->set_publisher_buffer<qnc::gripper::GripperState,
                                    qnc::gripper::GripperStatePubSubType>("gs_pub", state);
        node_->publish("gs_pub");
    }

    void publish_stats(const std::string& reason)
    {
        double uptime = std::chrono::duration<double>(
                            std::chrono::steady_clock::now() - start_time_).count();
        std::cout << "[Stats/" << reason << "] tx=" << tx_count_
                  << " rx=" << rx_count_ << " err=" << err_count_
                  << " uptime=" << static_cast<int>(uptime) << "s\n";
    }
};

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main()
{
    std::signal(SIGINT,  sig_handler);
    std::signal(SIGTERM, sig_handler);

    // --- Configuration from environment -------------------------------------
    //
    // Default (no env vars needed for the two QNC grippers on this Pi CM5):
    //   QNC_DE_RE_GPIO=none ./qnc_bridge
    //
    // Stable symlinks /dev/gripper_ag160 and /dev/gripper_cgc80 are created by
    // /etc/udev/rules.d/99-qnc-grippers.rules (keyed by FTDI serial number)
    // so physical USB port order no longer matters.
    //
    // Override individual ports if needed:
    //   QNC_SERIAL_PORT=/dev/ttyUSB0  QNC_SERIAL_PORT_2=/dev/ttyUSB1 ...
    //
    // QNC_SLAVE_ID_N   : DDS slave_id that labels port N in logs (default 1/2/3)
    // QNC_MODBUS_ADDR_N: Modbus address byte in RS485 frames for port N (default 1)
    //
    const char* env_port      = std::getenv("QNC_SERIAL_PORT");
    const char* env_port2     = std::getenv("QNC_SERIAL_PORT_2");
    const char* env_port3     = std::getenv("QNC_SERIAL_PORT_3");
    const char* env_gpio      = std::getenv("QNC_DE_RE_GPIO");
    const char* env_domain    = std::getenv("QNC_DOMAIN_ID");
    const char* env_interval  = std::getenv("QNC_STATS_INTERVAL");
    const char* env_slave1    = std::getenv("QNC_SLAVE_ID_1");
    const char* env_slave2    = std::getenv("QNC_SLAVE_ID_2");
    const char* env_slave3    = std::getenv("QNC_SLAVE_ID_3");
    const char* env_maddr1    = std::getenv("QNC_MODBUS_ADDR_1");
    const char* env_maddr2    = std::getenv("QNC_MODBUS_ADDR_2");
    const char* env_maddr3    = std::getenv("QNC_MODBUS_ADDR_3");

    const std::string serial_port  = env_port  ? env_port  : "/dev/gripper_ag160";
    const std::string serial_port2 = env_port2 ? env_port2 : "/dev/gripper_cgc80";
    const std::string serial_port3 = env_port3 ? env_port3 : "/dev/gripper_dh56";
    const std::string gpio_str     = env_gpio  ? env_gpio  : "none";
    const int domain_id            = env_domain   ? std::atoi(env_domain)   : 0;
    const int stats_interval       = env_interval ? std::atoi(env_interval) : 10;
    const uint8_t slave_id1        = static_cast<uint8_t>(env_slave1 ? std::atoi(env_slave1) : 1);
    const uint8_t slave_id2        = static_cast<uint8_t>(env_slave2 ? std::atoi(env_slave2) : 2);
    const uint8_t slave_id3        = static_cast<uint8_t>(env_slave3 ? std::atoi(env_slave3) : 3);
    const uint8_t modbus_addr1     = static_cast<uint8_t>(env_maddr1 ? std::atoi(env_maddr1) : 1);
    const uint8_t modbus_addr2     = static_cast<uint8_t>(env_maddr2 ? std::atoi(env_maddr2) : 1);
    const uint8_t modbus_addr3     = static_cast<uint8_t>(env_maddr3 ? std::atoi(env_maddr3) : 1);

    const char* env_iwait1   = std::getenv("QNC_INIT_WAIT_MS_1");
    const char* env_iwait2   = std::getenv("QNC_INIT_WAIT_MS_2");
    const char* env_iwait3   = std::getenv("QNC_INIT_WAIT_MS_3");
    const char* env_gdesc    = std::getenv("QNC_GRIPPER_DESCRIPTOR");
    const char* env_dry_run  = std::getenv("QNC_DRY_RUN");
    const int init_wait_ms1  = env_iwait1 ? std::atoi(env_iwait1) : 30000;  // AG-160: 160 mm stroke takes 15-20 s; bg FC04 poll shortens if confirmed sooner
    const int init_wait_ms2  = env_iwait2 ? std::atoi(env_iwait2) : 8000;   // CGC-80: max poll timeout
    const int init_wait_ms3  = env_iwait3 ? std::atoi(env_iwait3) : 8000;   // DH-5-6: max poll timeout
    const std::string gripper_desc_path = env_gdesc ? env_gdesc : "";
    const bool dry_run_mode = env_dry_run && std::string(env_dry_run) != "0";

    int gpio_num = -1;
    if (gpio_str != "none") {
        try { gpio_num = std::stoi(gpio_str); }
        catch (...) {
            std::cerr << "⚠ Invalid QNC_DE_RE_GPIO value '" << gpio_str
                      << "' — DE/RE uncontrolled\n";
        }
    }

    std::cout << "QNC Bridge\n"
              << "  Port 1       : " << serial_port
              << "  (dds_slave_id=" << (int)slave_id1
              << "  modbus_wire_addr=" << (int)modbus_addr1
              << "  init_wait=" << init_wait_ms1 << " ms)\n";
    if (!serial_port2.empty())
        std::cout << "  Port 2       : " << serial_port2
                  << "  (dds_slave_id=" << (int)slave_id2
                  << "  modbus_wire_addr=" << (int)modbus_addr2
                  << "  init_wait=" << init_wait_ms2 << " ms)\n";
    if (!serial_port3.empty())
        std::cout << "  Port 3       : " << serial_port3
                  << "  (dds_slave_id=" << (int)slave_id3
                  << "  modbus_wire_addr=" << (int)modbus_addr3
                  << "  init_wait=" << init_wait_ms3 << " ms)\n";
    std::cout << "  DE/RE GPIO   : " << gpio_str << "\n"
              << "  DDS domain   : " << domain_id << "\n"
              << "  Stats interval: every " << stats_interval << " writes\n"
              << "  Dry-run mode : " << (dry_run_mode ? "enabled" : "disabled") << "\n"
              << "  Gripper descriptor: " << (gripper_desc_path.empty() ? "<disabled>" : gripper_desc_path) << "\n\n";

    // --- Open RS485 port(s) -------------------------------------------------
    // Missing ports are non-fatal: bridge runs DDS-only and returns
    // EC_DISCONNECTED so the robot computer can detect faults via DDS.
    std::unique_ptr<ModbusRtu> modbus1, modbus2, modbus3;
    try {
        modbus1 = std::make_unique<ModbusRtu>(serial_port, gpio_num);
    } catch (const std::exception& e) {
        std::cerr << "⚠ Port 1 unavailable: " << e.what() << "\n";
    }
    if (!serial_port2.empty()) {
        try {
            modbus2 = std::make_unique<ModbusRtu>(serial_port2, gpio_num);
        } catch (const std::exception& e) {
            std::cerr << "⚠ Port 2 unavailable: " << e.what() << "\n";
        }
    }
    if (!serial_port3.empty()) {
        try {
            modbus3 = std::make_unique<ModbusRtu>(serial_port3, gpio_num);
        } catch (const std::exception& e) {
            std::cerr << "⚠ Port 3 unavailable: " << e.what() << "\n";
        }
    }

    // --- Startup RS485 probe ------------------------------------------------
    // For each open port try FC04 read of reg 0x0200 (status) at both the
    // configured modbus_addr and address 1, to confirm physical connectivity
    // and reveal the real Modbus address each gripper answers to.
    auto probe_port = [&](ModbusRtu* bus, const std::string& label, uint8_t cfg_addr) {
        if (!bus) return;
        std::cout << "[Probe] " << label << "\n";
        // Always test address 1 and the configured address
        std::set<uint8_t> addrs = {1, cfg_addr};
        for (uint8_t id : addrs) {
            std::vector<uint16_t> regs;
            bool ok = bus->read_input_registers(id, 0x0200, 3, regs, 500);
            if (ok && regs.size() >= 3) {
                std::cout << "  addr=" << (int)id
                          << "  RESPOND  init_state=" << regs[0]
                          << "  gripper_state=" << regs[1]
                          << "  position=" << regs[2] << "\n";
            } else {
                std::cout << "  addr=" << (int)id << "  no response\n";
            }
        }
    };
    probe_port(modbus1.get(),
               serial_port + "  (dds_slave_id=" + std::to_string(slave_id1)
               + "  configured_modbus_addr=" + std::to_string(modbus_addr1) + ")",
               modbus_addr1);
    probe_port(modbus2.get(),
               serial_port2 + "  (dds_slave_id=" + std::to_string(slave_id2)
               + "  configured_modbus_addr=" + std::to_string(modbus_addr2) + ")",
               modbus_addr2);
    probe_port(modbus3.get(),
               serial_port3 + "  (dds_slave_id=" + std::to_string(slave_id3)
               + "  configured_modbus_addr=" + std::to_string(modbus_addr3) + ")",
               modbus_addr3);
    std::cout << std::flush;

    // --- Start bridge -------------------------------------------------------
    try {
        QncBridge bridge(modbus1.get(), slave_id1, modbus_addr1,
                         modbus2.get(), slave_id2, modbus_addr2,
                         modbus3.get(), slave_id3, modbus_addr3,
                         domain_id, stats_interval,
                         init_wait_ms1, init_wait_ms2, init_wait_ms3,
                         gripper_desc_path,
                         dry_run_mode);

        // Wait for the robot platform's DDS publisher to be discovered before
        // declaring "Waiting for commands". This eliminates the race where
        // inits sent immediately after the bridge starts are missed because
        // DDS peer discovery hadn't completed yet.
        bridge.wait_for_publisher_match(10000);

        bridge.spin();
    } catch (const std::exception& e) {
        std::cerr << "✗ Fatal: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
