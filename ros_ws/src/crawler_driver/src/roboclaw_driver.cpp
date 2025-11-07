#include "crawler_driver/roboclaw_driver.hpp"

#include <array>
#include <bit>
#include <rclcpp/rclcpp.hpp>

namespace crawler_driver::roboclaw {

// =====================
// コンストラクタ/デストラクタ
// =====================

RoboclawDriver::RoboclawDriver(std::string port, uint8_t address)
    : address_(address),
      port_(std::move(port)),
      logger_(rclcpp::get_logger("RoboclawDriver")),
      io_context_(),
      serial_port_(io_context_, port_),
      work_guard_(boost::asio::make_work_guard(io_context_)),
      io_thread_([this] { io_context_.run(); }) {
    try {
        initializeSerialPort();
        RCLCPP_INFO(logger_, "Roboclaw初期化完了: ポート=%s, アドレス=0x%02X",
                    port_.c_str(), address_);
    } catch (const boost::system::system_error& e) {
        RCLCPP_ERROR(logger_, "シリアルポート初期化失敗: %s", e.what());
        throw std::runtime_error("シリアルポート初期化失敗: " +
                                 std::string(e.what()));
    }
}

RoboclawDriver::~RoboclawDriver() {
    work_guard_.reset();
    io_context_.stop();
    if (io_thread_.joinable()) {
        io_thread_.join();
    }
    RCLCPP_DEBUG(logger_, "Roboclawドライバー終了");
}

// =====================
// シリアルポート設定
// =====================

void RoboclawDriver::initializeSerialPort() {
    serial_port_.set_option(
        boost::asio::serial_port_base::baud_rate(BAUD_RATE));
    serial_port_.set_option(boost::asio::serial_port_base::character_size(8));
    serial_port_.set_option(boost::asio::serial_port_base::parity(
        boost::asio::serial_port_base::parity::none));
    serial_port_.set_option(boost::asio::serial_port_base::stop_bits(
        boost::asio::serial_port_base::stop_bits::one));
    serial_port_.set_option(boost::asio::serial_port_base::flow_control(
        boost::asio::serial_port_base::flow_control::none));
}

// =====================
// コマンド送信
// =====================

void RoboclawDriver::sendCommand(ByteBuffer command, Callback callback) {
    auto buffer = std::make_shared<ByteBuffer>(std::move(command));

    boost::asio::async_write(
        serial_port_, boost::asio::buffer(*buffer),
        [this, buffer, callback = std::move(callback)](
            const boost::system::error_code& ec, std::size_t) {
            if (ec) {
                RCLCPP_ERROR(logger_, "書き込みエラー: %s",
                             ec.message().c_str());
                callback(false);
                return;
            }

            // 応答を読み取る（0xFF = ACK）
            auto response = std::make_shared<uint8_t>(0);
            auto timer = std::make_shared<boost::asio::steady_timer>(
                io_context_, TIMEOUT);

            // タイムアウト設定
            timer->async_wait(
                [this, response](const boost::system::error_code& ec) {
                    if (!ec) {
                        RCLCPP_WARN(logger_, "応答タイムアウト");
                        serial_port_.cancel();
                    }
                });

            boost::asio::async_read(
                serial_port_, boost::asio::buffer(response.get(), 1),
                [this, response, timer, callback](
                    const boost::system::error_code& ec, std::size_t) {
                    timer->cancel();

                    if (ec) {
                        RCLCPP_ERROR(logger_, "読み取りエラー: %s",
                                     ec.message().c_str());
                        callback(false);
                        return;
                    }

                    const bool success = (*response == ACK);
                    if (!success) {
                        RCLCPP_WARN(logger_,
                                    "無効な応答: 0x%02X (期待値: 0xFF)",
                                    *response);
                    }
                    RCLCPP_DEBUG(logger_, "応答受信: 0x%02X %s", *response,
                                 success ? "✓" : "✗");
                    callback(success);
                });
        });
}

void RoboclawDriver::sendCommandWithResponse(
    ByteBuffer command, size_t response_size,
    std::function<void(std::optional<ByteBuffer>)> callback) {
    auto buffer = std::make_shared<ByteBuffer>(std::move(command));

    boost::asio::async_write(
        serial_port_, boost::asio::buffer(*buffer),
        [this, buffer, response_size, callback = std::move(callback)](
            const boost::system::error_code& ec, std::size_t) {
            if (ec) {
                RCLCPP_ERROR(logger_, "書き込みエラー: %s",
                             ec.message().c_str());
                callback(std::nullopt);
                return;
            }

            auto response = std::make_shared<ByteBuffer>(response_size);
            auto timer = std::make_shared<boost::asio::steady_timer>(
                io_context_, TIMEOUT);

            timer->async_wait([this](const boost::system::error_code& ec) {
                if (!ec) {
                    RCLCPP_WARN(logger_, "応答タイムアウト");
                    serial_port_.cancel();
                }
            });

            boost::asio::async_read(
                serial_port_, boost::asio::buffer(*response),
                [this, response, timer, callback](
                    const boost::system::error_code& ec,
                    std::size_t bytes_read) {
                    timer->cancel();

                    if (ec || bytes_read != response->size()) {
                        RCLCPP_ERROR(logger_, "読み取りエラー: %s",
                                     ec.message().c_str());
                        callback(std::nullopt);
                        return;
                    }

                    callback(*response);
                });
        });
}

// =====================
// モーター制御API
// =====================

void RoboclawDriver::setVelocity(Motor motor, double counts_per_sec,
                                 Callback callback) {
    const auto cmd = getVelocityCommand(motor);
    auto command = buildCommand(cmd, static_cast<int32_t>(counts_per_sec));
    RCLCPP_DEBUG(logger_, "速度設定: M%d -> %.0f counts/sec",
                 static_cast<int>(motor), counts_per_sec);
    sendCommand(std::move(command), std::move(callback));
}

void RoboclawDriver::setPID(Motor motor, const PIDConstants& pid,
                            Callback callback) {
    const auto cmd = getPIDCommand(motor);
    auto command = buildCommand(cmd, pid);
    RCLCPP_DEBUG(logger_, "PID設定: M%d -> P=%.3f I=%.3f D=%.3f QPPS=%d",
                 static_cast<int>(motor), pid.p, pid.i, pid.d, pid.qpps);
    sendCommand(std::move(command), std::move(callback));
}

void RoboclawDriver::resetEncoders(Callback callback) {
    auto command = buildCommand(Command::RESET_ENCODERS);
    RCLCPP_DEBUG(logger_, "エンコーダリセット");
    sendCommand(std::move(command), std::move(callback));
}

void RoboclawDriver::readEncoder(Motor motor, EncoderCallback callback) {
    const auto cmd = getEncoderCommand(motor);
    ByteBuffer command = {address_, static_cast<uint8_t>(cmd)};
    // エンコーダ読み取りコマンドはCRCを含まない（送信側）

    // マニュアル仕様: 応答フォーマット
    // Encoder Value (4 bytes, Big Endian) + Status (1 byte) + CRC (2 bytes) = 7
    // bytes
    sendCommandWithResponse(
        std::move(command), 7,
        [this, motor,
         callback = std::move(callback)](std::optional<ByteBuffer> response) {
            if (!response || response->size() != 7) {
                RCLCPP_ERROR(logger_,
                             "M%d: エンコーダ読み取り失敗 (サイズ不正)",
                             static_cast<int>(motor));
                callback(std::nullopt);
                return;
            }

            // CRC検証: 最初の5バイト（エンコーダ値4 + ステータス1）
            const uint16_t received_crc =
                (static_cast<uint16_t>((*response)[5]) << 8) | (*response)[6];
            const uint16_t calculated_crc =
                calculateCRC(std::span(response->data(), 5));

            if (received_crc != calculated_crc) {
                RCLCPP_ERROR(
                    logger_, "M%d: CRCエラー (期待: 0x%04X, 受信: 0x%04X)",
                    static_cast<int>(motor), calculated_crc, received_crc);
                callback(std::nullopt);
                return;
            }

            EncoderValue value;
            value.counts = extractInt32BE(*response, 0);
            value.status = (*response)[4];

            RCLCPP_DEBUG(logger_, "M%d: エンコーダ=%d counts, status=0x%02X",
                         static_cast<int>(motor), value.counts, value.status);
            callback(value);
        });
}

// =====================
// コマンドヘルパー
// =====================

Command RoboclawDriver::getVelocityCommand(Motor motor) {
    return motor == Motor::M1 ? Command::M1_VELOCITY : Command::M2_VELOCITY;
}

Command RoboclawDriver::getPIDCommand(Motor motor) {
    return motor == Motor::M1 ? Command::M1_SET_PID : Command::M2_SET_PID;
}

Command RoboclawDriver::getEncoderCommand(Motor motor) {
    return motor == Motor::M1 ? Command::M1_ENCODER : Command::M2_ENCODER;
}

// =====================
// コマンドビルダー
// =====================

RoboclawDriver::ByteBuffer RoboclawDriver::buildCommand(Command cmd) const {
    ByteBuffer data = {address_, static_cast<uint8_t>(cmd)};
    appendCRC(data);
    return data;
}

RoboclawDriver::ByteBuffer RoboclawDriver::buildCommand(Command cmd,
                                                        int32_t value) const {
    ByteBuffer data = {address_, static_cast<uint8_t>(cmd)};
    appendInt32BE(data, value);
    appendCRC(data);
    return data;
}

RoboclawDriver::ByteBuffer RoboclawDriver::buildCommand(
    Command cmd, const PIDConstants& pid) const {
    ByteBuffer data = {address_, static_cast<uint8_t>(cmd)};
    appendFloat32BE(data, pid.d);
    appendFloat32BE(data, pid.p);
    appendFloat32BE(data, pid.i);
    appendInt32BE(data, pid.qpps);
    appendCRC(data);
    return data;
}

// =====================
// データシリアライゼーション
// =====================

constexpr uint16_t RoboclawDriver::calculateCRC(std::span<const uint8_t> data) {
    uint16_t crc = 0;
    for (uint8_t byte : data) {
        crc ^= static_cast<uint16_t>(byte) << 8;
        for (int i = 0; i < 8; ++i) {
            crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) : (crc << 1);
        }
    }
    return crc;
}

void RoboclawDriver::appendCRC(ByteBuffer& data) {
    const uint16_t crc = calculateCRC(data);
    data.push_back(static_cast<uint8_t>(crc >> 8));
    data.push_back(static_cast<uint8_t>(crc & 0xFF));
}

void RoboclawDriver::appendInt32BE(ByteBuffer& data, int32_t value) {
    data.push_back(static_cast<uint8_t>(value >> 24));
    data.push_back(static_cast<uint8_t>(value >> 16));
    data.push_back(static_cast<uint8_t>(value >> 8));
    data.push_back(static_cast<uint8_t>(value));
}

void RoboclawDriver::appendFloat32BE(ByteBuffer& data, float value) {
    // C++20のstd::bit_castを使用してより安全に変換
    const uint32_t bits = std::bit_cast<uint32_t>(value);

    data.push_back(static_cast<uint8_t>(bits >> 24));
    data.push_back(static_cast<uint8_t>(bits >> 16));
    data.push_back(static_cast<uint8_t>(bits >> 8));
    data.push_back(static_cast<uint8_t>(bits));
}

int32_t RoboclawDriver::extractInt32BE(std::span<const uint8_t> data,
                                       size_t offset) {
    return (static_cast<int32_t>(data[offset]) << 24) |
           (static_cast<int32_t>(data[offset + 1]) << 16) |
           (static_cast<int32_t>(data[offset + 2]) << 8) |
           static_cast<int32_t>(data[offset + 3]);
}

}  // namespace crawler_driver::roboclaw
