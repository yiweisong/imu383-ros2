#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "imu383/msg/imu383_data.hpp"
#include "utils.h"

#include <libserial/SerialPort.h>

#include <string>
#include <vector>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

class IMU383Node : public rclcpp::Node {
public:
    IMU383Node() : Node("imu383_node") {
        publisher_ = this->create_publisher<imu383::msg::IMU383Data>("/imu383/data", 10);
        
        // Timer to handle reconnection and data reading continuously
        timer_ = this->create_wall_timer(
            10ms, std::bind(&IMU383Node::read_serial_data, this));

        RCLCPP_INFO(this->get_logger(), "IMU383 Node started. Searching for device...");
    }

    ~IMU383Node() {
        if (serial_port_.IsOpen()) {
            serial_port_.Close();
        }
    }

private:
    void read_serial_data() {
        if (!serial_port_.IsOpen()) {
            if (!find_and_open_device()) {
                return;
            }
        }

        try {
            while (serial_port_.IsDataAvailable()) {
                uint8_t byte;
                serial_port_.ReadByte(byte, 0);
                process_byte(byte);
            }
        } catch (const LibSerial::ReadTimeout&) {
            // normal if no data available within 0 timeout, handled by IsDataAvailable loop gracefully
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Serial read error: %s", e.what());
            serial_port_.Close();
        }
    }

    bool find_and_open_device() {
        const std::vector<std::string> port_prefixes = {"/dev/ttyUSB", "/dev/ttyACM"};
        for (const auto& prefix : port_prefixes) {
            for (int i = 0; i < 10; ++i) {
                std::string port_name = prefix + std::to_string(i);
                try {
                    serial_port_.Open(port_name);
                    serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_230400);
                    serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
                    serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);
                    serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
                    serial_port_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);

                    // Send PK command
                    std::vector<uint8_t> pk_cmd = {0x55, 0x55, 0x50, 0x4B, 0x00, 0x9E, 0xF4};
                    serial_port_.Write(pk_cmd);
                    
                    // Wait 100ms for response
                    std::this_thread::sleep_for(100ms);
                    
                    size_t available_bytes = serial_port_.GetNumberOfBytesAvailable();
                    if (available_bytes >= 7) {
                        std::vector<uint8_t> resp;
                        for(size_t j = 0; j < available_bytes; ++j) {
                            uint8_t byte;
                            serial_port_.ReadByte(byte, 0);
                            resp.push_back(byte);
                        }

                        if (resp[0] == 0x55 && 
                            resp[1] == 0x55 && 
                            resp[2] == 0x50 && 
                            resp[3] == 0x4B) {
                            
                            RCLCPP_INFO(this->get_logger(), "IMU383 found on %s", port_name.c_str());
                            return true;
                        }
                    }
                    serial_port_.Close();
                } catch (const std::exception& e) {
                    if(serial_port_.IsOpen()) {
                        serial_port_.Close();
                    }
                }
            }
        }
        return false;
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
                auto msg = imu383::msg::IMU383Data();
                msg.header.stamp = this->get_clock()->now();
                msg.header.frame_id = "imu383_s1";

                const uint8_t* payload = msg_buffer_.data() + 5;

                msg.x_accel = get_i2(payload + 0) * (20.0f / 65536.0f);
                msg.y_accel = get_i2(payload + 2) * (20.0f / 65536.0f);
                msg.z_accel = get_i2(payload + 4) * (20.0f / 65536.0f);
                
                msg.x_rate = get_i2(payload + 6) * (1260.0f / 65536.0f);
                msg.y_rate = get_i2(payload + 8) * (1260.0f / 65536.0f);
                msg.z_rate = get_i2(payload + 10) * (1260.0f / 65536.0f);
                
                msg.x_rate_temp = get_i2(payload + 12) * (400.0f / 65536.0f);
                msg.y_rate_temp = get_i2(payload + 14) * (400.0f / 65536.0f);
                msg.z_rate_temp = get_i2(payload + 16) * (400.0f / 65536.0f);
                
                msg.board_temp = get_i2(payload + 18) * (400.0f / 65536.0f);
                
                msg.counter = get_u2(payload + 20);
                msg.master_bit_status = get_u2(payload + 22);

                publisher_->publish(msg);
            }
        }
    }

    int16_t get_i2(const uint8_t* data) {
        return static_cast<int16_t>((data[0] << 8) | data[1]);
    }

    uint16_t get_u2(const uint8_t* data) {
        return static_cast<uint16_t>((data[0] << 8) | data[1]);
    }

    rclcpp::Publisher<imu383::msg::IMU383Data>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    LibSerial::SerialPort serial_port_;

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