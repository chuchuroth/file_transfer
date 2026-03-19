/**
 * qnc_bridge.cpp
 *
 * QNC firmware bridge: subscribes to DDS commands from the robot computer,
 * forwards them to the RS485 field device (gripper / any Modbus RTU slave),
 * and publishes responses + stats back over DDS.
 *
 * DDS topics (QNC-ICD-001 §7.1):
 *   SUBSCRIBE  qnc/modbus/write_cmd  — FC06 write single register
 *   SUBSCRIBE  qnc/modbus/read_cmd   — FC04 read input registers
 *   PUBLISH    qnc/modbus/response   — result / register data
 *   PUBLISH    qnc/modbus/stats      — cumulative TX/RX/error counters
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
 *   QNC_STATS_INTERVAL   publish stats every N write ops (default: 10)
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
#include <set>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <linux/serial.h>

// Generated from ModbusRTUBridge.idl (QNC-ICD-001 §7.1)
#include "ModbusRTUBridge.hpp"
#include "ModbusRTUBridgePubSubTypes.hpp"
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>

using namespace eprosima::fastdds::dds;

// ---------------------------------------------------------------------------
// Signal handling
// ---------------------------------------------------------------------------

static std::atomic<bool> g_running{true};

static void sig_handler(int) { g_running.store(false); }

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static uint16_t crc16(const uint8_t* data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j)
            crc = (crc & 0x0001) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }
    return crc;
}

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

// ---------------------------------------------------------------------------
// ModbusRtu
//
// Protocol-agnostic Modbus RTU master over RS485.
// Supports any slave_id and any register address — does NOT hard-code
// gripper-specific semantics; those live in the robot computer's JSON
// descriptors (QNC-ICD-001 §1.4).
// ---------------------------------------------------------------------------

class ModbusRtu {
public:
    /**
     * Open the serial port at 115200 8N1.
     * Throws std::runtime_error on failure.
     *
     * @param port        serial device (e.g. /dev/ttyAMA0)
     * @param de_re_gpio  sysfs GPIO number for the DE/RE line, or < 0 for auto
     */
    explicit ModbusRtu(const std::string& port, int de_re_gpio = -1)
        : port_(port)
    {
        fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0)
            throw std::runtime_error("Failed to open RS485 port: " + port);

        configure_port();
        if (de_re_gpio >= 0)
            setup_de_re_gpio(de_re_gpio);

        std::cout << "✓ RS485 open: " << port << "\n";
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
     * @param reg_addr   Register address (0x0000–0xFFFF)
     * @param value      16-bit value to write
     * @param timeout_ms Response timeout in ms; 0 = fire-and-forget (no response read)
     * @returns true on success (or when fire_and_forget)
     */
    bool write_register(uint8_t slave_id, uint16_t reg_addr, uint16_t value,
                        int timeout_ms = 500)
    {
        // FC06 PDU: [slave_id, 0x06, addr_hi, addr_lo, val_hi, val_lo]
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
            // Fire-and-forget: send frame and return immediately without waiting
            // for the FC06 echo.  Mirrors the proven path in gripper_control.cpp
            // (write_register with fire_and_forget=true).  tcflush(TCIOFLUSH) on
            // the NEXT write clears any loopback echo bytes before they interfere.
            tcflush(fd_, TCIOFLUSH);
            de_re_tx();
            ::write(fd_, frame.data(), frame.size());
            tcdrain(fd_);   // block until last bit is physically transmitted
            usleep(500);    // 0.5 ms guard: last stop bit fully clocked out
            de_re_rx();
            return true;
        }

        // FC06 echo response: 8 bytes identical to the request (Modbus spec §6.6)
        std::vector<uint8_t> resp = transact(frame, 8, timeout_ms);
        return validate_response(resp, slave_id, 0x06);
    }

    /**
     * FC04 — Read Input Registers.
     *
     * @param slave_id   Modbus slave address (1-247)
     * @param start_reg  First register address
     * @param count      Number of registers to read (1-125)
     * @param out        Populated with register values on success
     * @param timeout_ms Response timeout in ms
     * @returns true on success, false on timeout / CRC error / exception
     */
    bool read_input_registers(uint8_t slave_id, uint16_t start_reg,
                               uint16_t count, std::vector<uint16_t>& out,
                               int timeout_ms = 1000)
    {
        return read_registers(slave_id, 0x04, start_reg, count, out, timeout_ms);
    }

    /**
     * FC03 — Read Holding Registers.
     * Same framing as FC04 but for read/write holding registers.
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

        // Response: [slave_id, fc, byte_count, data..., crc_lo, crc_hi]
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

    int         fd_       = -1;
    int         de_re_fd_ = -1;
    std::string port_;

    // --- Serial port --------------------------------------------------------

    void configure_port()
    {
        termios tty{};
        if (tcgetattr(fd_, &tty) != 0)
            throw std::runtime_error("tcgetattr failed");

        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_cflag |= CREAD | CLOCAL;
        tty.c_iflag  = 0;
        tty.c_oflag  = 0;
        tty.c_lflag  = 0;
        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 0;

        if (tcsetattr(fd_, TCSANOW, &tty) != 0)
            throw std::runtime_error("tcsetattr failed");
    }

    // --- DE/RE GPIO (sysfs legacy) ------------------------------------------

    void setup_de_re_gpio(int sysfs_num)
    {
        { int efd = ::open("/sys/class/gpio/export", O_WRONLY);
          if (efd >= 0) {
              std::string s = std::to_string(sysfs_num);
              ::write(efd, s.c_str(), s.size());
              ::close(efd);
          }}
        usleep(50000);

        std::string dir_path = "/sys/class/gpio/gpio"
                               + std::to_string(sysfs_num) + "/direction";
        int dfd = ::open(dir_path.c_str(), O_WRONLY);
        if (dfd < 0) {
            std::cerr << "⚠ Cannot open GPIO" << sysfs_num
                      << " direction — DE/RE uncontrolled\n";
            return;
        }
        ::write(dfd, "out", 3);
        ::close(dfd);

        std::string val_path = "/sys/class/gpio/gpio"
                               + std::to_string(sysfs_num) + "/value";
        de_re_fd_ = ::open(val_path.c_str(), O_WRONLY);
        if (de_re_fd_ < 0) {
            std::cerr << "⚠ Cannot open GPIO" << sysfs_num
                      << " value — DE/RE uncontrolled\n";
            return;
        }
        ::write(de_re_fd_, "0", 1);  // start in RX mode
        std::cout << "✓ DE/RE GPIO" << sysfs_num << " active\n";
    }

    void de_re_tx() { if (de_re_fd_ >= 0) ::pwrite(de_re_fd_, "1", 1, 0); }
    void de_re_rx() { if (de_re_fd_ >= 0) ::pwrite(de_re_fd_, "0", 1, 0); }

    // --- Modbus RTU framing -------------------------------------------------

    static std::vector<uint8_t> build_frame(const std::vector<uint8_t>& pdu)
    {
        auto frame = pdu;
        uint16_t crc = crc16(frame.data(), frame.size());
        frame.push_back(crc & 0xFF);
        frame.push_back((crc >> 8) & 0xFF);
        return frame;
    }

    std::vector<uint8_t> transact(const std::vector<uint8_t>& frame,
                                  size_t max_bytes,
                                  int timeout_ms = 1000)
    {
        tcflush(fd_, TCIOFLUSH);
        de_re_tx();
        if (::write(fd_, frame.data(), frame.size()) < 0) {
            de_re_rx();
            return {};
        }
        tcdrain(fd_);
        usleep(500);
        de_re_rx();

        std::vector<uint8_t> response;
        response.reserve(max_bytes);

        auto deadline = std::chrono::steady_clock::now()
                        + std::chrono::milliseconds(timeout_ms);

        while (response.size() < max_bytes) {
            auto now       = std::chrono::steady_clock::now();
            auto remaining = std::chrono::duration_cast<std::chrono::microseconds>(
                                 deadline - now).count();
            if (remaining <= 0) break;

            struct timeval tv;
            tv.tv_sec  = remaining / 1'000'000;
            tv.tv_usec = remaining % 1'000'000;

            fd_set rfds;
            FD_ZERO(&rfds);
            FD_SET(fd_, &rfds);

            int sel = ::select(fd_ + 1, &rfds, nullptr, nullptr, &tv);
            if (sel <= 0) break;

            uint8_t tmp[256];
            ssize_t n = ::read(fd_, tmp,
                               std::min(sizeof(tmp), max_bytes - response.size()));
            if (n > 0)
                response.insert(response.end(), tmp, tmp + n);
        }
        return response;
    }

    static bool validate_response(const std::vector<uint8_t>& resp,
                                   uint8_t expected_id, uint8_t expected_fc)
    {
        if (resp.size() < 4) return false;
        if (resp[0] != expected_id) return false;
        if (resp[1] == (expected_fc | 0x80)) {
            std::cerr << "  Modbus exception 0x" << std::hex
                      << static_cast<int>(resp[2]) << std::dec << "\n";
            return false;
        }
        if (resp[1] != expected_fc) return false;

        size_t len     = resp.size();
        uint16_t r_crc = resp[len-2] | (resp[len-1] << 8);
        uint16_t c_crc = crc16(resp.data(), len - 2);
        return r_crc == c_crc;
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
              int init_wait_ms3 = 3500)   // DH-5-6
        : modbus1_(modbus1), slave_id1_(slave_id1), modbus_addr1_(modbus_addr1),
          modbus2_(modbus2), slave_id2_(slave_id2), modbus_addr2_(modbus_addr2),
          modbus3_(modbus3), slave_id3_(slave_id3), modbus_addr3_(modbus_addr3),
          stats_interval_(stats_interval)
    {
        if (modbus1_) port_init_wait_ms_[modbus1_] = init_wait_ms1;
        if (modbus2_) port_init_wait_ms_[modbus2_] = init_wait_ms2;
        if (modbus3_) port_init_wait_ms_[modbus3_] = init_wait_ms3;

        participant_ = DomainParticipantFactory::get_instance()
                           ->create_participant(domain_id, PARTICIPANT_QOS_DEFAULT);
        if (!participant_)
            throw std::runtime_error("DDS: failed to create DomainParticipant");

        // Register types
        TypeSupport wc_type(new qnc::modbus::WriteCommandPubSubType());
        TypeSupport rc_type(new qnc::modbus::ReadCommandPubSubType());
        TypeSupport rsp_type(new qnc::modbus::ResponsePubSubType());
        TypeSupport bs_type(new qnc::modbus::BridgeStatsPubSubType());
        wc_type.register_type(participant_);
        rc_type.register_type(participant_);
        rsp_type.register_type(participant_);
        bs_type.register_type(participant_);

        // Topics
        wc_topic_  = participant_->create_topic("qnc/modbus/write_cmd",
                                                wc_type.get_type_name(), TOPIC_QOS_DEFAULT);
        rc_topic_  = participant_->create_topic("qnc/modbus/read_cmd",
                                                rc_type.get_type_name(), TOPIC_QOS_DEFAULT);
        rsp_topic_ = participant_->create_topic("qnc/modbus/response",
                                                rsp_type.get_type_name(), TOPIC_QOS_DEFAULT);
        bs_topic_  = participant_->create_topic("qnc/modbus/stats",
                                                bs_type.get_type_name(), TOPIC_QOS_DEFAULT);

        // Subscriber for incoming commands
        subscriber_ = participant_->create_subscriber(SUBSCRIBER_QOS_DEFAULT);
        // KEEP_LAST depth=10 so force+speed+position samples (3 per cycle) are
        // never overwritten before drain_write_commands() takes them (Bug 2).
        // TRANSIENT_LOCAL matches the robot-side DataWriter durability so that
        // commands published before the bridge's DataReader is matched are
        // cached by the writer and delivered once discovery completes.
        // NOTE: durability is only effective when the matching DataWriter also
        // declares TRANSIENT_LOCAL — see robot_side/src/gripper_control.cpp.
        DataReaderQos cmd_qos = DATAREADER_QOS_DEFAULT;
        cmd_qos.history().kind    = KEEP_LAST_HISTORY_QOS;
        cmd_qos.history().depth   = 10;
        cmd_qos.durability().kind = TRANSIENT_LOCAL_DURABILITY_QOS;
        wc_reader_  = subscriber_->create_datareader(wc_topic_, cmd_qos);
        rc_reader_  = subscriber_->create_datareader(rc_topic_, cmd_qos);

        // Publisher for responses and stats
        publisher_  = participant_->create_publisher(PUBLISHER_QOS_DEFAULT);
        rsp_writer_ = publisher_->create_datawriter(rsp_topic_, DATAWRITER_QOS_DEFAULT);
        bs_writer_  = publisher_->create_datawriter(bs_topic_,  DATAWRITER_QOS_DEFAULT);

        start_time_ = std::chrono::steady_clock::now();

        std::cout << "[DDS] QncBridge active on domain " << domain_id << "\n"
                  << "      SUB: qnc/modbus/write_cmd  qnc/modbus/read_cmd\n"
                  << "      PUB: qnc/modbus/response   qnc/modbus/stats\n";
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
        if (subscriber_ && wc_reader_)  subscriber_->delete_datareader(wc_reader_);
        if (subscriber_ && rc_reader_)  subscriber_->delete_datareader(rc_reader_);
        if (publisher_  && rsp_writer_) publisher_->delete_datawriter(rsp_writer_);
        if (publisher_  && bs_writer_)  publisher_->delete_datawriter(bs_writer_);
        if (participant_ && subscriber_) participant_->delete_subscriber(subscriber_);
        if (participant_ && publisher_)  participant_->delete_publisher(publisher_);
        if (participant_ && wc_topic_)   participant_->delete_topic(wc_topic_);
        if (participant_ && rc_topic_)   participant_->delete_topic(rc_topic_);
        if (participant_ && rsp_topic_)  participant_->delete_topic(rsp_topic_);
        if (participant_ && bs_topic_)   participant_->delete_topic(bs_topic_);
        if (participant_)
            DomainParticipantFactory::get_instance()->delete_participant(participant_);
    }

    QncBridge(const QncBridge&) = delete;
    QncBridge& operator=(const QncBridge&) = delete;

    /**
     * Block until the robot platform's write_cmd publisher is matched (i.e.
     * DDS peer discovery is complete) or until timeout_ms elapses.
     * Returns true if matched, false on timeout.
     */
    bool wait_for_publisher_match(int timeout_ms = 10000)
    {
        const int poll_ms  = 100;
        int       elapsed  = 0;
        std::cout << "[Bridge] Waiting for robot publisher match..." << std::flush;
        while (elapsed < timeout_ms) {
            std::vector<eprosima::fastdds::dds::InstanceHandle_t> handles;
            wc_reader_->get_matched_publications(handles);
            if (!handles.empty()) {
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
     * Block until g_running is false, processing all incoming DDS commands.
     * Per-slave init windows are tracked in init_wait_map_ inside
     * drain_write_commands() — each gripper calibrates independently.
     */
    void spin()
    {
        std::cout << "[Bridge] Waiting for commands (Ctrl-C to stop)...\n" << std::flush;

        while (g_running.load()) {
            bool processed = false;
            processed |= drain_write_commands();
            processed |= drain_read_commands();

            if (!processed)
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        publish_stats("shutdown");
        std::cout << "\n[Bridge] Stopped. tx=" << tx_count_
                  << " rx=" << rx_count_
                  << " err=" << err_count_ << "\n";
    }

private:
    // --- DDS plumbing -------------------------------------------------------
    DomainParticipant* participant_ = nullptr;
    Subscriber*        subscriber_  = nullptr;
    Publisher*         publisher_   = nullptr;
    Topic* wc_topic_  = nullptr;
    Topic* rc_topic_  = nullptr;
    Topic* rsp_topic_ = nullptr;
    Topic* bs_topic_  = nullptr;
    DataReader* wc_reader_  = nullptr;
    DataReader* rc_reader_  = nullptr;
    DataWriter* rsp_writer_ = nullptr;
    DataWriter* bs_writer_  = nullptr;

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

    // Per-port init calibration windows (keyed by ModbusRtu* port pointer).
    // When a port receives reg=0x0100 (init), its deadline is inserted here.
    // Subsequent write_cmds for that port are discarded until the deadline
    // passes, preventing stale motion commands from firing before the gripper
    // finishes its calibration stroke.
    std::map<ModbusRtu*, std::chrono::steady_clock::time_point> init_wait_map_;
    // Per-port regular-init (val=1) guard duration in ms.
    // AG-160 (160 mm stroke) needs ~5500 ms; CGC-80 (95 mm) needs ~3500 ms.
    std::map<ModbusRtu*, int> port_init_wait_ms_;

    // --- Route DDS slave_id to the matching (bus, wire_addr) pair ----------
    // slave_id=1 → port1, slave_id=2 → port2, slave_id=3 → port3.
    // Returns nullptr for the bus if no port is configured for that slave_id.
    std::pair<ModbusRtu*, uint8_t> route(uint8_t dds_slave_id) const
    {
        if (modbus1_ && dds_slave_id == slave_id1_) return {modbus1_, modbus_addr1_};
        if (modbus2_ && dds_slave_id == slave_id2_) return {modbus2_, modbus_addr2_};
        if (modbus3_ && dds_slave_id == slave_id3_) return {modbus3_, modbus_addr3_};
        return {nullptr, 0};
    }

    // --- Process write_cmd (FC06) -------------------------------------------
    // Route to the single port whose dds_slave_id matches sample.slave_id().
    // Each port has its own init calibration window tracked in init_wait_map_.

    bool drain_write_commands()
    {
        bool any = false;
        qnc::modbus::WriteCommand sample;
        SampleInfo info;

        while (wc_reader_->take_next_sample(&sample, &info) == RETCODE_OK) {
            if (!info.valid_data) continue;
            any = true;

            const uint8_t  dds_slave = static_cast<uint8_t>(sample.slave_id());
            const uint16_t reg = sample.register_address();
            const uint16_t val = sample.value();

            std::cout << "[CMD] write_cmd slave=" << (int)dds_slave
                      << " reg=0x" << std::hex << std::setfill('0') << std::setw(4) << reg << std::dec
                      << " val=" << val << "  tag=" << sample.tag().c_str() << "\n";

            ++tx_count_;

            auto [bus, wire_addr] = route(dds_slave);
            if (!bus) {
                std::cout << "  ⚠ No port configured for slave_id=" << (int)dds_slave << "\n";
            } else {
                // Per-port init calibration window check
                auto it = init_wait_map_.find(bus);
                if (it != init_wait_map_.end()) {
                    if (std::chrono::steady_clock::now() < it->second) {
                        std::cout << "  ⏳ " << bus->port() << " still calibrating, skipping\n";
                    } else {
                        std::cout << "  [init] " << bus->port()
                                  << " calibration complete, ready for motion\n" << std::flush;
                        init_wait_map_.erase(it);
                        std::cout << "  → " << bus->port() << "  modbus_addr=" << (int)wire_addr << "\n";
                        bus->write_register(wire_addr, reg, val, 0);
                    }
                } else {
                    std::cout << "  → " << bus->port() << "  modbus_addr=" << (int)wire_addr << "\n";
                    bus->write_register(wire_addr, reg, val, 0);

                    // Arm per-port calibration window on init command
                    if (reg == 0x0100 && (val == 1 || val == 165)) {
                        auto it_ms = port_init_wait_ms_.find(bus);
                        const int regular_ms = (it_ms != port_init_wait_ms_.end())
                                               ? it_ms->second : 3500;
                        const int wait_ms = (val == 165) ? 8500 : regular_ms;
                        init_wait_map_[bus] = std::chrono::steady_clock::now()
                                             + std::chrono::milliseconds(wait_ms);
                        std::cout << "  [init] " << bus->port()
                                  << " calibration started — discarding write_cmds for "
                                  << wait_ms << " ms\n" << std::flush;
                    }
                }
            }

            if (tx_count_ % static_cast<uint64_t>(stats_interval_) == 0)
                publish_stats("periodic");
        }
        return any;
    }

    // --- Process read_cmd (FC03 / FC04) -------------------------------------
    // Route to the single port whose dds_slave_id matches sample.slave_id().

    bool drain_read_commands()
    {
        bool any = false;
        qnc::modbus::ReadCommand sample;
        SampleInfo info;

        while (rc_reader_->take_next_sample(&sample, &info) == RETCODE_OK) {
            if (!info.valid_data) continue;
            any = true;

            const uint8_t  slave = static_cast<uint8_t>(sample.slave_id());
            const uint16_t start = sample.start_register();
            const uint16_t cnt   = std::min(static_cast<uint16_t>(
                                       sample.count()), static_cast<uint16_t>(125));
            const bool     use_fc04 =
                (sample.function_code() != qnc::FunctionCode::FC_READ_HOLDING_REGISTERS);

            std::cout << "[CMD] read_cmd  slave=" << (int)slave
                      << " reg=0x" << std::hex << std::setfill('0') << std::setw(4) << start << std::dec
                      << " cnt=" << cnt
                      << "  " << (use_fc04 ? "FC04" : "FC03")
                      << "  tag=" << sample.tag().c_str() << "\n";

            auto [bus, wire_addr] = route(slave);
            if (!bus) {
                std::cout << "  ⚠ No port configured for slave_id=" << (int)slave << "\n";
                publish_response_data(slave, start, {}, qnc::EC_DISCONNECTED,
                                      std::string(sample.tag().c_str()));
                continue;
            }

            std::cout << "  → " << bus->port()
                      << "  modbus_addr=" << (int)wire_addr << "\n";
            std::vector<uint16_t> regs;
            long err_code = qnc::EC_SUCCESS;
            bool ok = use_fc04
                ? bus->read_input_registers(wire_addr, start, cnt, regs, 1000)
                : bus->read_holding_registers(wire_addr, start, cnt, regs, 1000);
            if (ok) {
                ++rx_count_;
            } else {
                err_code = qnc::EC_TIMEOUT;
                ++err_count_;
                last_err_ = err_code;
            }
            publish_response_data(slave, start, regs, err_code,
                                  std::string(sample.tag().c_str()));
        }
        return any;
    }

    // --- Publish helpers ----------------------------------------------------

    void publish_response_data(uint8_t slave, uint16_t start,
                                const std::vector<uint16_t>& regs,
                                long err_code, const std::string& tag)
    {
        if (!rsp_writer_) return;
        qnc::modbus::Response msg;
        msg.slave_id(slave);
        msg.start_register(start);
        msg.function_code(qnc::FunctionCode::FC_READ_INPUT_REGISTERS);
        for (auto v : regs) msg.data().push_back(v);
        msg.error_code(err_code);
        msg.tag(tag);
        msg.timestamp(iso_now());
        rsp_writer_->write(&msg);
    }

    void publish_stats(const std::string& reason)
    {
        if (!bs_writer_) return;
        double uptime = std::chrono::duration<double>(
                            std::chrono::steady_clock::now() - start_time_).count();
        qnc::modbus::BridgeStats msg;
        msg.device_id("qnc_bridge_rs485");
        msg.device_connected(modbus1_ != nullptr || modbus2_ != nullptr || modbus3_ != nullptr);
        msg.tx_count(tx_count_);
        msg.rx_count(rx_count_);
        msg.error_count(err_count_);
        msg.last_error_code(last_err_);
        msg.avg_latency_us(0);
        msg.uptime_seconds(uptime);
        msg.timestamp(iso_now());
        bs_writer_->write(&msg);
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
    const int init_wait_ms1  = env_iwait1 ? std::atoi(env_iwait1) : 7500;  // AG-160 default (~6s stroke + margin)
    const int init_wait_ms2  = env_iwait2 ? std::atoi(env_iwait2) : 3500;  // CGC-80 default
    const int init_wait_ms3  = env_iwait3 ? std::atoi(env_iwait3) : 3500;  // DH-5-6 default

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
              << "  Stats interval: every " << stats_interval << " writes\n\n";

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
                         init_wait_ms1, init_wait_ms2, init_wait_ms3);

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
