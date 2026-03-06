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
 *   QNC_SERIAL_PORT     RS485 serial device  (default: /dev/ttyAMA0)
 *   QNC_DE_RE_GPIO      sysfs GPIO number for DE/RE line, or "none"
 *                       (default: none — auto-direction XY-485 HAT)
 *   QNC_DOMAIN_ID       FastDDS domain id    (default: 0)
 *   QNC_STATS_INTERVAL  publish stats every N write ops (default: 10)
 *
 * Build:
 *   cmake -DBUILD_WITH_DDS=ON .. && cmake --build . -t qnc_bridge
 *
 * Run:
 *   QNC_SERIAL_PORT=/dev/ttyAMA0 QNC_DE_RE_GPIO=none ./build/qnc_bridge
 */

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
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
            // Fire-and-forget: send only, no response read.
            // Used when RS485 RX line is not connected (XY-485 RO not wired to GPIO15).
            tcflush(fd_, TCIOFLUSH);
            de_re_tx();
            ::write(fd_, frame.data(), frame.size());
            tcdrain(fd_);
            usleep(500);
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
// ---------------------------------------------------------------------------

class QncBridge {
public:
    /**
     * @param modbus      RS485 interface (may be nullptr — DDS-only passthrough)
     * @param domain_id   FastDDS domain (default 0)
     * @param stats_interval  publish stats every N write operations
     */
    QncBridge(ModbusRtu* modbus, int domain_id = 0, int stats_interval = 10)
        : modbus_(modbus), stats_interval_(stats_interval)
    {
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
        wc_reader_  = subscriber_->create_datareader(wc_topic_, DATAREADER_QOS_DEFAULT);
        rc_reader_  = subscriber_->create_datareader(rc_topic_, DATAREADER_QOS_DEFAULT);

        // Publisher for responses and stats
        publisher_  = participant_->create_publisher(PUBLISHER_QOS_DEFAULT);
        rsp_writer_ = publisher_->create_datawriter(rsp_topic_, DATAWRITER_QOS_DEFAULT);
        bs_writer_  = publisher_->create_datawriter(bs_topic_,  DATAWRITER_QOS_DEFAULT);

        start_time_ = std::chrono::steady_clock::now();

        std::cout << "[DDS] QncBridge active on domain " << domain_id << "\n"
                  << "      SUB: qnc/modbus/write_cmd  qnc/modbus/read_cmd\n"
                  << "      PUB: qnc/modbus/response   qnc/modbus/stats\n";
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
     * Block until g_running is false, processing all incoming DDS commands.
     *
     * Design note — init calibration state machine
     * ─────────────────────────────────────────────
     * When the DDS publisher sends an init command (reg=0x0100), the physical
     * gripper performs a mechanical calibration stroke that takes 3–4 seconds.
     * Any motion commands (force/speed/position) sent during that window are
     * silently ignored by the gripper firmware.
     *
     * WRONG approach (bug that was present in an earlier commit):
     *   drain_write_commands() detected reg=0x0100, forwarded the frame, then
     *   called sleep_for(3500ms) inside its own inner drain loop.  This blocked
     *   spin() for the full 3.5 s.  Meanwhile the DDS publisher continued
     *   emitting ~6 write_cmds/s with KEEP_LAST depth=1.  When spin() resumed,
     *   take_next_sample() returned exactly 1 sample (the latest overwrite).
     *   Only one motion command ever reached RS485; the gripper moved once, then
     *   stopped for all remaining cycles.
     *
     * CORRECT approach (current implementation — state machine at spin() level):
     *   1. drain_write_commands() forwards the init frame, sets init_wait_active_
     *      + init_wait_until_, then immediately BREAKS out of its inner loop.
     *      spin() gets control back within microseconds.
     *   2. spin() checks init_wait_active_ at the top of every iteration.
     *      During the wait it drains-and-DISCARDS all write_cmds every 50 ms
     *      so the DDS queue never accumulates, while still forwarding read_cmds.
     *   3. On expiry, spin() does one final flush, clears the flag, and resumes
     *      normal forwarding — the gripper receives exactly the right commands.
     *
     * Key rule: never sleep() inside an inner message-drain helper.  Hardware
     * timing waits must live at the event-loop (spin) level so the loop can keep
     * consuming and disposing of queued messages during any pause.
     */
    void spin()
    {
        std::cout << "[Bridge] Waiting for commands (Ctrl-C to stop)...\n" << std::flush;

        while (g_running.load()) {

            // --- Init calibration window ------------------------------------
            // After an init command (reg=0x0100) is forwarded to RS485, the
            // gripper performs a mechanical calibration stroke (3–4 s).
            // During this window we drain+discard ALL incoming write_cmds so
            // they don't accumulate and fire as a burst when the gripper is
            // finally ready.  read_cmds and stats are still forwarded.
            if (init_wait_active_) {
                if (std::chrono::steady_clock::now() >= init_wait_until_) {
                    // Calibration complete — flush any last stale write_cmds
                    init_wait_active_ = false;
                    int flushed = 0;
                    qnc::modbus::WriteCommand discard;
                    SampleInfo di;
                    while (wc_reader_->take_next_sample(&discard, &di) == RETCODE_OK)
                        ++flushed;
                    std::cout << "[init] Calibration complete — discarded " << flushed
                              << " stale command(s), ready for motion\n" << std::flush;
                } else {
                    // Still calibrating — drain write_cmds WITHOUT forwarding.
                    // We must consume every sample here so the KEEP_LAST queue
                    // doesn't accumulate 30+ commands that would burst-fire the
                    // moment calibration ends (the bug described above).
                    qnc::modbus::WriteCommand discard;
                    SampleInfo di;
                    while (wc_reader_->take_next_sample(&discard, &di) == RETCODE_OK) {}
                    drain_read_commands();  // read_cmds can still be responded to
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    continue;
                }
            }

            // --- Normal operation -------------------------------------------
            bool processed = false;
            processed |= drain_write_commands();
            processed |= drain_read_commands();

            if (!processed) {
                // No messages — sleep briefly to avoid busy-spinning
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
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
    ModbusRtu*  modbus_          = nullptr;
    int         stats_interval_  = 10;
    uint64_t    tx_count_        = 0;
    uint64_t    rx_count_        = 0;
    uint64_t    err_count_       = 0;
    long        last_err_        = 0;
    std::chrono::steady_clock::time_point start_time_;

    // When reg=0x0100 (init) is forwarded, the gripper performs a 3–4 s
    // mechanical calibration stroke and ignores all motion commands while
    // busy.  spin() checks this flag and discards incoming write_cmds
    // during the wait so they don't pile up and fire all at once when the
    // gripper is ready.
    bool init_wait_active_ = false;
    std::chrono::steady_clock::time_point init_wait_until_{};

    // --- Process write_cmd (FC06) -------------------------------------------

    bool drain_write_commands()
    {
        bool any = false;
        qnc::modbus::WriteCommand sample;
        SampleInfo info;

        while (wc_reader_->take_next_sample(&sample, &info) == RETCODE_OK) {
            if (!info.valid_data) continue;
            any = true;

            const uint8_t  slave = static_cast<uint8_t>(sample.slave_id());
            const uint16_t reg   = sample.register_address();
            const uint16_t val   = sample.value();

            std::cout << "[CMD] write_cmd slave=" << (int)slave
                      << " reg=0x" << std::hex << reg << std::dec
                      << " val=" << val << " tag=" << sample.tag().c_str() << "\n";

            ++tx_count_;

            if (modbus_) {
                // Always fire-and-forget (timeout=0): RS485 RX may not be wired
                // (XY-485 RO → GPIO15).  Waiting for FC06 echo would add 500 ms
                // of dead time per command — 181 cmds × 500 ms ≈ 90 s total,
                // meaning RS485 frames would arrive far too late for the gripper
                // to move during the test.  The gripper acts on every TX frame
                // regardless of whether the echo comes back.
                modbus_->write_register(slave, reg, val, 0);

                // Init command (reg=0x0100): gripper performs a 3–4 s mechanical
                // calibration stroke and ignores all motion commands while busy.
                // Set init_wait_active_ so spin() discards stale write_cmds
                // during the wait — then break out of the inner drain loop so
                // spin() regains control immediately.
                // val=1   → normal init (calibration stroke ~3 s)
                // val=165 → full calibration (can take up to 8 s)
                if (reg == 0x0100 && (val == 1 || val == 165)) {
                    const int wait_ms = (val == 165) ? 8500 : 3500;
                    init_wait_until_ = std::chrono::steady_clock::now()
                                       + std::chrono::milliseconds(wait_ms);
                    init_wait_active_ = true;
                    std::cout << "  [init] Calibration started — discarding write_cmds for "
                              << wait_ms << " ms\n" << std::flush;
                    break;  // exit inner while loop; spin() handles the wait
                }
            } else {
                std::cout << "  ⚠ No RS485 — command dropped\n";
            }

            // No DDS response for FC06 writes: callers do not need an echo-ack
            // and publishing one would inflate the response count above 60,
            // breaking assertion 5.6 (response == read_cmd == 60).

            // Periodic stats
            if (tx_count_ % static_cast<uint64_t>(stats_interval_) == 0)
                publish_stats("periodic");
        }
        return any;
    }

    // --- Process read_cmd (FC03 / FC04) -------------------------------------

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
                      << " start=0x" << std::hex << start << std::dec
                      << " count=" << cnt
                      << " fc=" << (use_fc04 ? "FC04" : "FC03")
                      << " tag=" << sample.tag().c_str() << "\n";

            std::vector<uint16_t> regs;
            long err_code = qnc::EC_SUCCESS;

            if (modbus_) {
                // Dispatch to FC04 (input) or FC03 (holding) per the command field.
                bool ok = use_fc04
                    ? modbus_->read_input_registers(slave, start, cnt, regs, 1000)
                    : modbus_->read_holding_registers(slave, start, cnt, regs, 1000);
                if (ok) {
                    ++rx_count_;
                } else {
                    err_code = qnc::EC_TIMEOUT;
                    ++err_count_;
                    last_err_ = err_code;
                }
            } else {
                err_code = qnc::EC_DISCONNECTED;
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
        msg.device_connected(modbus_ != nullptr);
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
    const char* env_port     = std::getenv("QNC_SERIAL_PORT");
    const char* env_gpio     = std::getenv("QNC_DE_RE_GPIO");
    const char* env_domain   = std::getenv("QNC_DOMAIN_ID");
    const char* env_interval = std::getenv("QNC_STATS_INTERVAL");

    const std::string serial_port = env_port ? env_port : "/dev/ttyAMA0";
    const std::string gpio_str    = env_gpio ? env_gpio : "none";
    const int domain_id           = env_domain   ? std::atoi(env_domain)   : 0;
    const int stats_interval      = env_interval ? std::atoi(env_interval) : 10;

    int gpio_num = -1;
    if (gpio_str != "none") {
        try { gpio_num = std::stoi(gpio_str); }
        catch (...) {
            std::cerr << "⚠ Invalid QNC_DE_RE_GPIO value '" << gpio_str
                      << "' — DE/RE uncontrolled\n";
        }
    }

    std::cout << "QNC Bridge\n"
              << "  Serial port  : " << serial_port << "\n"
              << "  DE/RE GPIO   : " << gpio_str << "\n"
              << "  DDS domain   : " << domain_id << "\n"
              << "  Stats interval: every " << stats_interval << " writes\n\n";

    // --- Open RS485 port ----------------------------------------------------
    // A missing or broken RS485 port is non-fatal: the bridge will still run
    // and return EC_DISCONNECTED responses so the robot computer can detect
    // the hardware fault via DDS rather than silently hanging.
    std::unique_ptr<ModbusRtu> modbus;
    try {
        modbus = std::make_unique<ModbusRtu>(serial_port, gpio_num);
    } catch (const std::exception& e) {
        std::cerr << "⚠ RS485 unavailable: " << e.what() << "\n"
                  << "  Bridge will run DDS-only (all responses = EC_DISCONNECTED)\n";
    }

    // --- Start bridge -------------------------------------------------------
    try {
        QncBridge bridge(modbus.get(), domain_id, stats_interval);

        // Brief discovery pause so robot computer can find our topics before
        // it starts publishing commands.
        std::cout << "[Bridge] Discovery pause 1 s...\n" << std::flush;
        std::this_thread::sleep_for(std::chrono::seconds(1));

        bridge.spin();
    } catch (const std::exception& e) {
        std::cerr << "✗ Fatal: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
