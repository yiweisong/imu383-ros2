#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "utils.h"

#include <serial_driver/serial_driver.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <filesystem>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

class IMU383Node : public rclcpp::Node {
public:
    IMU383Node() : Node("imu383_node") {
        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu383/data", 10);
        configured_port_ = this->declare_parameter<std::string>("serial_port", "");
        io_context_ = std::make_unique<drivers::common::IoContext>(1);
        serial_driver_ = std::make_unique<drivers::serial_driver::SerialDriver>(*io_context_);
        
        // Timer to handle reconnection and data reading continuously
        timer_ = this->create_wall_timer(
            10ms, std::bind(&IMU383Node::read_serial_data, this));

        RCLCPP_INFO(this->get_logger(), "IMU383 Node started. Searching for device...");
    }

    ~IMU383Node() {
        close_serial_port();
    }

private:
    static constexpr float kGToMps2 = 9.80665F;
    static constexpr float kDegToRad = 0.01745329251994329577F;

    void read_serial_data() {
        if (!is_serial_open()) {
            if (!find_and_open_device()) {
                return;
            }
        }

        try {
            std::vector<uint8_t> bytes(512, 0U);
            const size_t count = serial_port_->receive(bytes);
            for (size_t i = 0; i < count; ++i) {
                const uint8_t byte = bytes[i];
                process_byte(byte);
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Serial read error: %s", e.what());
            close_serial_port();
        }
    }

    bool find_and_open_device() {
        for (const auto& port_name : collect_candidate_ports()) {
            if (try_open_and_probe(port_name)) {
                active_port_name_ = port_name;
                RCLCPP_INFO(this->get_logger(), "IMU383 found on %s", active_port_name_.c_str());
                return true;
            }
        }

        return false;
    }

    std::vector<std::string> collect_candidate_ports() const {
        std::vector<std::string> candidates;
        if (!configured_port_.empty()) {
            candidates.push_back(configured_port_);
        }

#ifdef __linux__
        const std::filesystem::path by_id_dir{"/dev/serial/by-id"};
        if (std::filesystem::exists(by_id_dir) && std::filesystem::is_directory(by_id_dir)) {
            for (const auto& entry : std::filesystem::directory_iterator(by_id_dir)) {
                std::error_code ec;
                const auto resolved = std::filesystem::canonical(entry.path(), ec);
                if (!ec) {
                    candidates.push_back(resolved.string());
                }
            }
        }

        const std::array<std::string, 2> port_prefixes = {"/dev/ttyUSB", "/dev/ttyACM"};
        for (const auto& prefix : port_prefixes) {
            for (int i = 0; i < 20; ++i) {
                candidates.push_back(prefix + std::to_string(i));
            }
        }
#endif

        std::vector<std::string> deduped;
        std::set<std::string> seen;
        for (const auto& candidate : candidates) {
            if (!candidate.empty() && seen.insert(candidate).second) {
                deduped.push_back(candidate);
            }
        }
        return deduped;
    }

    bool try_open_and_probe(const std::string& port_name) {
        constexpr std::array<uint8_t, 7> kPkCommand = {0x55, 0x55, 0x50, 0x4B, 0x00, 0x9E, 0xF4};

        try {
            const auto config = drivers::serial_driver::SerialPortConfig(
                230400,
                drivers::serial_driver::FlowControl::NONE,
                drivers::serial_driver::Parity::NONE,
                drivers::serial_driver::StopBits::ONE);

            serial_driver_->init_port(port_name, config);
            auto candidate_port = serial_driver_->port();
            candidate_port->open();
            candidate_port->send(std::vector<uint8_t>(kPkCommand.begin(), kPkCommand.end()));

            std::this_thread::sleep_for(100ms);

            std::vector<uint8_t> response(128, 0U);
            const size_t bytes = candidate_port->receive(response);
            response.resize(bytes);

            if (has_pk_header(response)) {
                serial_port_ = std::move(candidate_port);
                return true;
            }

            candidate_port->close();
        } catch (const std::exception&) {
            close_serial_port();
        }

        return false;
    }

    bool has_pk_header(const std::vector<uint8_t>& response) const {
        constexpr std::array<uint8_t, 4> kPkHeader = {0x55, 0x55, 0x50, 0x4B};
        if (response.size() < kPkHeader.size()) {
            return false;
        }

        for (size_t i = 0; i + kPkHeader.size() <= response.size(); ++i) {
            if (std::equal(kPkHeader.begin(), kPkHeader.end(), response.begin() + i)) {
                return true;
            }
        }
        return false;
    }

    bool is_serial_open() const {
        return serial_port_ && serial_port_->is_open();
    }

    void close_serial_port() {
        if (serial_port_) {
            try {
                if (serial_port_->is_open()) {
                    serial_port_->close();
                }
            } catch (const std::exception&) {
            }
            serial_port_.reset();
        }
        active_port_name_.clear();
    }

    void process_byte(uint8_t b) {
        msg_buffer_.push_back(b);

        if (state_ == WAITING_SYNC1) {
            if (b == 0x55) {
                state_ = WAITING_SYNC2;
                msg_buffer_.clear();
                msg_buffer_.push_back(b);
            }
        } else if (state_ == WAITING_SYNC2) {
            if (b == 0x55) {
                state_ = READING_HEADER;
            } else {
                state_ = WAITING_SYNC1;
            }
        } else if (state_ == READING_HEADER) {
            if (msg_buffer_.size() == 5) {
                payload_len_ = b;
                state_ = READING_PAYLOAD;
            }
        } else if (state_ == READING_PAYLOAD) {
            if (msg_buffer_.size() == static_cast<size_t>(5 + payload_len_ + 2)) {
                // Buffer is full (Header(5) + Payload + CRC(2))
                if (verify_crc(msg_buffer_.data() + 2, msg_buffer_.size() - 2)) {
                    parse_packet();
                } else {
                    RCLCPP_WARN(this->get_logger(), "CRC verification failed");
                }
                state_ = WAITING_SYNC1;
            } else if (msg_buffer_.size() > 255) { // Security fallback
                state_ = WAITING_SYNC1;
            }
        }
    }

    void parse_packet() {
        if (msg_buffer_[2] == 0x53 && msg_buffer_[3] == 0x31) { // 'S', '1' -> 0x53, 0x31
            if (payload_len_ >= 24) {
                auto msg = sensor_msgs::msg::Imu();
                msg.header.stamp = this->get_clock()->now();
                msg.header.frame_id = "imu383_s1";

                const uint8_t* payload = msg_buffer_.data() + 5;

                const float x_accel_g = get_i2(payload + 0) * (20.0f / 65536.0f);
                const float y_accel_g = get_i2(payload + 2) * (20.0f / 65536.0f);
                const float z_accel_g = get_i2(payload + 4) * (20.0f / 65536.0f);

                const float x_rate_deg = get_i2(payload + 6) * (1260.0f / 65536.0f);
                const float y_rate_deg = get_i2(payload + 8) * (1260.0f / 65536.0f);
                const float z_rate_deg = get_i2(payload + 10) * (1260.0f / 65536.0f);

                msg.linear_acceleration.x = x_accel_g * kGToMps2;
                msg.linear_acceleration.y = y_accel_g * kGToMps2;
                msg.linear_acceleration.z = z_accel_g * kGToMps2;

                msg.angular_velocity.x = x_rate_deg * kDegToRad;
                msg.angular_velocity.y = y_rate_deg * kDegToRad;
                msg.angular_velocity.z = z_rate_deg * kDegToRad;

                // Orientation is unavailable from the current packet format.
                msg.orientation_covariance[0] = -1.0;

                msg.angular_velocity_covariance[0] = 0.0;
                msg.angular_velocity_covariance[4] = 0.0;
                msg.angular_velocity_covariance[8] = 0.0;

                msg.linear_acceleration_covariance[0] = 0.0;
                msg.linear_acceleration_covariance[4] = 0.0;
                msg.linear_acceleration_covariance[8] = 0.0;

                publisher_->publish(msg);
            }
        }
    }

    int16_t get_i2(const uint8_t* data) {
        return static_cast<int16_t>((data[0] << 8) | data[1]);
    }

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<drivers::common::IoContext> io_context_;
    std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
    std::shared_ptr<drivers::serial_driver::SerialPort> serial_port_;
    std::string configured_port_;
    std::string active_port_name_;

    enum State {
        WAITING_SYNC1,
        WAITING_SYNC2,
        READING_HEADER,
        READING_PAYLOAD
    };
    State state_ = WAITING_SYNC1;
    std::vector<uint8_t> msg_buffer_;
    uint8_t payload_len_ = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IMU383Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}