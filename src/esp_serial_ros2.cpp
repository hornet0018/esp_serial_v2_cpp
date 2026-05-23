#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <cstdint>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"
#include "nlohmann/json.hpp"

#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "diagnostic_updater/publisher.hpp"

using namespace std::chrono_literals;
using json = nlohmann::json;

class ESPSerialROS2 : public rclcpp::Node
{
public:
  ESPSerialROS2()
  : Node("esp_serial_ros2"),
    diagnostic_updater_(this),
    packet_count_(0),
    log_counter_(0)
  {
    this->declare_parameter("serial_port", "/dev/esp32_serial");
    this->declare_parameter("baud_rate", 115200);
    this->declare_parameter("expected_hz", 100.0);
    this->declare_parameter("wheel_radius", 0.0473);
    this->declare_parameter("wheel_separation", 0.1796);
    this->declare_parameter("max_rpm", 115);
    this->declare_parameter("update_rate", 50.0);
    this->declare_parameter("cmd_vel_timeout", 0.5);
    this->declare_parameter("invert_motor_l", false);
    this->declare_parameter("invert_motor_r", false);

    std::string port = this->get_parameter("serial_port").as_string();
    int baud = this->get_parameter("baud_rate").as_int();
    double expected_hz = this->get_parameter("expected_hz").as_double();
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    wheel_separation_ = this->get_parameter("wheel_separation").as_double();
    max_rpm_ = this->get_parameter("max_rpm").as_int();
    double update_rate = this->get_parameter("update_rate").as_double();
    cmd_vel_timeout_ = this->get_parameter("cmd_vel_timeout").as_double();
    invert_motor_l_ = this->get_parameter("invert_motor_l").as_bool();
    invert_motor_r_ = this->get_parameter("invert_motor_r").as_bool();

    last_cmd_vel_time_ = this->now();

    prev_pos_l_raw_ = 0;
    prev_pos_r_raw_ = 0;
    accumulated_pos_l_rad_ = 0.0;
    accumulated_pos_r_rad_ = 0.0;
    first_pos_l_ = true;
    first_pos_r_ = true;

    try {
      ser_.setPort(port);
      ser_.setBaudrate(baud);
      serial::Timeout to = serial::Timeout::simpleTimeout(1000);
      ser_.setTimeout(to);
      ser_.open();
    } catch (serial::IOException& e) {
      RCLCPP_ERROR(this->get_logger(), "Unable to open port %s", port.c_str());
      throw e;
    }

    if (ser_.isOpen()) {
      RCLCPP_INFO(this->get_logger(), "Serial Port initialized: %s at %d", port.c_str(), baud);

      // Flush any existing data in the buffer
      try {
        size_t avail = ser_.available();
        if (avail > 0) {
          std::vector<uint8_t> junk;
          ser_.read(junk, avail);
          RCLCPP_INFO(this->get_logger(), "Flushed %zu bytes from serial buffer", avail);
        }
      } catch (...) {
        // Ignore errors during flush
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Serial Port failed to open");
      throw std::runtime_error("Serial Port failed to open");
    }

    // Diagnostics
    diagnostic_updater_.setHardwareID("ESP32_Serial");
    
    // Frequency diagnostics
    double min_freq = expected_hz * 0.8;
    double max_freq = expected_hz * 1.2;
    diagnostic_updater::FrequencyStatusParam freq_param(&min_freq, &max_freq, 0.1, 10);
    
    freq_diag_ = std::make_unique<diagnostic_updater::HeaderlessTopicDiagnostic>(
      "esp_serial_packets", diagnostic_updater_, freq_param);

    // Dynamic diagnostic update timer
    diag_timer_ = this->create_wall_timer(
      1000ms, [this]() { diagnostic_updater_.force_update(); });

    // Service Server
    srv_trigger_ = this->create_service<std_srvs::srv::Trigger>(
      "esp_serial/trigger",
      std::bind(&ESPSerialROS2::handle_trigger, this, std::placeholders::_1, std::placeholders::_2));

    // Publishers
    pub_speed_r_ = this->create_publisher<std_msgs::msg::Int16>("esp/speed_r", 10);
    pub_speed_l_ = this->create_publisher<std_msgs::msg::Int16>("esp/speed_l", 10);
    pub_pos_r_ = this->create_publisher<std_msgs::msg::Int16>("esp/position_r", 10);
    pub_pos_l_ = this->create_publisher<std_msgs::msg::Int16>("esp/position_l", 10);
    pub_voltage_ = this->create_publisher<std_msgs::msg::Float32>("esp/battery_voltage", 10);
    pub_timestamp_ = this->create_publisher<std_msgs::msg::UInt32>("esp/timestamp", 10);
    pub_pos_l_rad_ = this->create_publisher<std_msgs::msg::Float64>("esp/position_l_rad", 10);
    pub_pos_r_rad_ = this->create_publisher<std_msgs::msg::Float64>("esp/position_r_rad", 10);

    // Subscriber
    sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&ESPSerialROS2::cmd_vel_callback, this, std::placeholders::_1));

    // Timer for reading serial data
    std::chrono::milliseconds timer_period(static_cast<int>(1000.0 / update_rate));
    timer_ = this->create_wall_timer(
      timer_period, std::bind(&ESPSerialROS2::timer_callback, this));
  }

private:
  void handle_trigger(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;
    RCLCPP_INFO(this->get_logger(), "Trigger service called");
    response->success = ser_.isOpen();
    response->message = "ESP Serial ROS 2 node is active. Port: " + this->get_parameter("serial_port").as_string() + 
                        ", Connected: " + std::string(ser_.isOpen() ? "Yes" : "No");
  }

  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_cmd_vel_time_ = this->now();
    double v_linear = msg->linear.x;
    double v_angular = msg->angular.z;

    // Kinematics: Differential drive
    double v_l = v_linear - (v_angular * wheel_separation_ / 2.0);
    double v_r = v_linear + (v_angular * wheel_separation_ / 2.0);

    // Linear velocity (m/s) to RPM
    // rpm = (v / (2 * PI * radius)) * 60
    double rpm_l = (v_l / (2.0 * M_PI * wheel_radius_)) * 60.0;
    double rpm_r = (v_r / (2.0 * M_PI * wheel_radius_)) * 60.0;

    // Apply inversion
    if (invert_motor_l_) rpm_l *= -1.0;
    if (invert_motor_r_) rpm_r *= -1.0;

    // Clip to max RPM
    int16_t cmd_speed_l = std::clamp(static_cast<int16_t>(std::round(rpm_l)), static_cast<int16_t>(-max_rpm_), static_cast<int16_t>(max_rpm_));
    int16_t cmd_speed_r = std::clamp(static_cast<int16_t>(std::round(rpm_r)), static_cast<int16_t>(-max_rpm_), static_cast<int16_t>(max_rpm_));

    send_speed_to_esp(cmd_speed_l, cmd_speed_r);
  }

  void send_speed_to_esp(int16_t speed_l, int16_t speed_r)
  {
    if (!ser_.isOpen()) return;

    try {
      json j;
      j["cmd_speed_L"] = speed_l;
      j["cmd_speed_R"] = speed_r;

      std::vector<uint8_t> cbor = json::to_cbor(j);
      uint16_t crc = calculate_crc16(cbor);
      
      std::vector<uint8_t> payload = cbor;
      payload.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
      payload.push_back(static_cast<uint8_t>(crc & 0xFF));

      std::vector<uint8_t> encoded = cobs_encode(payload);
      encoded.push_back(0x00); // Delimiter

      ser_.write(encoded);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error sending data to ESP: %s", e.what());
    }
  }

  void timer_callback()
  {
    try {
      // Watchdog: Send 0 speed if cmd_vel is not received
      if ((this->now() - last_cmd_vel_time_).seconds() > cmd_vel_timeout_) {
        send_speed_to_esp(0, 0);
      }

      if (ser_.available()) {
        std::vector<uint8_t> data;
        ser_.read(data, ser_.available());
        RCLCPP_DEBUG(this->get_logger(), "Read %zu bytes from serial, buffer size before: %zu", data.size(), buffer_.size());

        for (auto byte : data) {
          if (byte == 0x00) {
            if (!buffer_.empty()) {
              RCLCPP_DEBUG(this->get_logger(), "Packet delimiter found, processing %zu bytes", buffer_.size());
              std::vector<uint8_t> packet = buffer_.to_vector();
              if (!packet.empty()) {
                process_packet(packet);
              }
              buffer_.clear();
            }
          } else {
            buffer_.push_back(byte);
            if (buffer_.size() > 1024) {
              RCLCPP_WARN(this->get_logger(), "Buffer overflow, clearing");
              buffer_.clear();
            }
          }
        }
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Exception in timer_callback: %s", e.what());
      buffer_.clear();
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "Unknown exception in timer_callback");
      buffer_.clear();
    }
  }

  void process_packet(const std::vector<uint8_t>& packet)
  {
    // Skip first few packets to avoid initial junk data
    if (packet_count_ < 5) {
      packet_count_++;
      RCLCPP_DEBUG(this->get_logger(), "Skipping packet %zu (initial flush)", packet_count_);
      return;
    }

    packet_count_++;
    RCLCPP_DEBUG(this->get_logger(), "Processing packet %zu: size=%zu", packet_count_, packet.size());

    // Log hex data for debugging
    if (packet.size() > 0) {
      std::string hex_str;
      for (size_t i = 0; i < std::min(size_t(32), packet.size()); i++) {
        char buf[8];
        snprintf(buf, sizeof(buf), "%02X ", packet[i]);
        hex_str += buf;
      }
      RCLCPP_DEBUG(this->get_logger(), "Packet data: %s%s", hex_str.c_str(), packet.size() > 32 ? "..." : "");
    }

    auto decoded = cobs_decode(packet);
    RCLCPP_DEBUG(this->get_logger(), "COBS decoded: size=%zu", decoded.size());

    if (decoded.size() < 3) {
      RCLCPP_WARN(this->get_logger(), "Received data too short: size=%zu", decoded.size());
      return;
    }

    size_t cbor_len = decoded.size() - 2;
    std::vector<uint8_t> cbor_data(decoded.begin(), decoded.begin() + cbor_len);
    uint16_t recv_crc = (static_cast<uint16_t>(decoded[cbor_len]) << 8) | decoded[cbor_len + 1];
    uint16_t calc_crc = calculate_crc16(cbor_data);

    RCLCPP_DEBUG(this->get_logger(), "CRC check: recv=0x%04X, calc=0x%04X, match=%s", recv_crc, calc_crc, recv_crc == calc_crc ? "YES" : "NO");

    if (recv_crc != calc_crc) {
      // Only log CRC errors after initial flush period
      if (packet_count_ > 10) {
        RCLCPP_ERROR(this->get_logger(), "CRC mismatch: rec=0x%04X, calc=0x%04X, cbor_len=%zu", recv_crc, calc_crc, cbor_len);
      }
      return;
    }

    if (recv_crc == calc_crc) {
      freq_diag_->tick();
      try {
        // Parse CBOR using nlohmann::json
        json j = json::from_cbor(cbor_data);

        RCLCPP_DEBUG(this->get_logger(), "CBOR parsed successfully: %s", j.dump().c_str());

        // Publish data
        if (j.is_object()) {
          int16_t speed_r = 0, speed_l = 0;
          int16_t pos_r = 0, pos_l = 0;
          float voltage = 0.0f;
          uint32_t timestamp = 0;

          if (j.contains("speed_r")) {
            auto msg = std_msgs::msg::Int16();
            msg.data = j["speed_r"].get<int16_t>();
            speed_r = msg.data;
            pub_speed_r_->publish(msg);
          }
          if (j.contains("speed_l")) {
            auto msg = std_msgs::msg::Int16();
            msg.data = j["speed_l"].get<int16_t>();
            speed_l = msg.data;
            pub_speed_l_->publish(msg);
          }
          if (j.contains("position_r")) {
            auto msg = std_msgs::msg::Int16();
            msg.data = j["position_r"].get<int16_t>();
            pos_r = msg.data;
            pub_pos_r_->publish(msg);

            auto msg_rad = std_msgs::msg::Float64();
            msg_rad.data = unwrap_encoder(-pos_r, prev_pos_r_raw_, accumulated_pos_r_rad_, first_pos_r_);
            pub_pos_r_rad_->publish(msg_rad);
          }
          if (j.contains("position_l")) {
            auto msg = std_msgs::msg::Int16();
            msg.data = j["position_l"].get<int16_t>();
            pos_l = msg.data;
            pub_pos_l_->publish(msg);

            auto msg_rad = std_msgs::msg::Float64();
            msg_rad.data = unwrap_encoder(pos_l, prev_pos_l_raw_, accumulated_pos_l_rad_, first_pos_l_);
            pub_pos_l_rad_->publish(msg_rad);
          }
          if (j.contains("battery_voltage_mV")) {
            auto msg = std_msgs::msg::Float32();
            msg.data = j["battery_voltage_mV"].get<float>();
            voltage = msg.data;
            pub_voltage_->publish(msg);
          }
          if (j.contains("timestamp")) {
            auto msg = std_msgs::msg::UInt32();
            msg.data = j["timestamp"].get<uint32_t>();
            timestamp = msg.data;
            pub_timestamp_->publish(msg);
          }

          // Print real-time data at INFO level (throttled)
          log_counter_++;
          if (log_counter_ >= LOG_INTERVAL) {
            log_counter_ = 0;
            RCLCPP_INFO(this->get_logger(), "Speed R: %d, Speed L: %d | Pos R: %d, Pos L: %d | Voltage: %.2fV | Time: %u",
                       speed_r, speed_l, pos_r, pos_l, voltage / 1000.0f, timestamp);
          }
        }

        RCLCPP_DEBUG(this->get_logger(), "[JSON] Received data: %s", j.dump().c_str());
      } catch (json::parse_error& e) {
        RCLCPP_ERROR(this->get_logger(), "CBOR parse error: %s (cbor_len=%zu)", e.what(), cbor_data.size());
        // Log hex data for debugging
        std::string hex_str;
        for (size_t i = 0; i < std::min(size_t(32), cbor_data.size()); i++) {
          char buf[8];
          snprintf(buf, sizeof(buf), "%02X ", cbor_data[i]);
          hex_str += buf;
        }
        RCLCPP_ERROR(this->get_logger(), "CBOR data: %s%s", hex_str.c_str(), cbor_data.size() > 32 ? "..." : "");
      } catch (json::type_error& e) {
        RCLCPP_ERROR(this->get_logger(), "JSON type error: %s", e.what());
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "CRC mismatch: rec=0x%04X, calc=0x%04X, cbor_len=%zu", recv_crc, calc_crc, cbor_len);
      // Log first few bytes for debugging
      if (cbor_data.size() > 0) {
        std::string hex_str;
        for (size_t i = 0; i < std::min(size_t(16), cbor_data.size()); i++) {
          char buf[8];
          snprintf(buf, sizeof(buf), "%02X ", cbor_data[i]);
          hex_str += buf;
        }
        RCLCPP_ERROR(this->get_logger(), "CBOR data (first 16 bytes): %s", hex_str.c_str());
      }
    }
  }

  double unwrap_encoder(int16_t current, int16_t& prev, double& accumulated, bool& first)
  {
    const double SCALE = 2.0 * M_PI / 32767.0;
    if (first) {
      prev = current;
      accumulated = 0.0;
      first = false;
      return 0.0;
    }

    int32_t delta = static_cast<int32_t>(current) - static_cast<int32_t>(prev);
    const int32_t HALF_RANGE = 32767 / 2; // 16383
    const int32_t FULL_RANGE = 32767;

    if (delta > HALF_RANGE) {
      delta -= FULL_RANGE;
    } else if (delta < -HALF_RANGE) {
      delta += FULL_RANGE;
    }

    accumulated += static_cast<double>(delta) * SCALE;
    prev = current;
    return accumulated;
  }

  std::vector<uint8_t> cobs_encode(const std::vector<uint8_t>& input)
  {
    std::vector<uint8_t> output;
    output.push_back(0); // Reserve for first code
    size_t code_idx = 0;
    uint8_t code = 1;

    for (uint8_t byte : input) {
      if (byte == 0) {
        output[code_idx] = code;
        code_idx = output.size();
        output.push_back(0);
        code = 1;
      } else {
        output.push_back(byte);
        code++;
        if (code == 0xFF) {
          output[code_idx] = code;
          code_idx = output.size();
          output.push_back(0);
          code = 1;
        }
      }
    }
    output[code_idx] = code;
    return output;
  }

  std::vector<uint8_t> cobs_decode(const std::vector<uint8_t>& input)
  {
    std::vector<uint8_t> output;
    output.reserve(input.size());
    size_t idx = 0;
    while (idx < input.size()) {
      uint8_t code = input[idx++];
      for (uint8_t i = 1; i < code && idx < input.size(); i++) {
        output.push_back(input[idx++]);
      }
      // Only add zero if this wasn't a 0xFF code AND we haven't reached the end
      if (code < 0xFF && idx < input.size()) {
        output.push_back(0);
      }
    }
    return output;
  }

  uint16_t calculate_crc16(const std::vector<uint8_t>& data)
  {
    uint16_t crc = 0xFFFF;
    for (uint8_t byte : data) {
      crc ^= (static_cast<uint16_t>(byte) << 8);
      for (int i = 0; i < 8; i++) {
        if (crc & 0x8000) {
          crc = (crc << 1) ^ 0x1021;
        } else {
          crc = (crc << 1);
        }
      }
    }
    return crc & 0xFFFF;
  }

  serial::Serial ser_;
  rclcpp::TimerBase::SharedPtr timer_;
  // Ring Buffer implementation
  class RingBuffer
  {
  public:
    explicit RingBuffer(size_t capacity = 1024)
      : buffer_(capacity), head_(0), tail_(0), size_(0), capacity_(capacity) {}

    void push_back(uint8_t data)
    {
      buffer_[head_] = data;
      head_ = (head_ + 1) % capacity_;
      if (size_ < capacity_) {
        size_++;
      } else {
        // Buffer is full, move tail forward (overwrite oldest data)
        tail_ = (tail_ + 1) % capacity_;
      }
    }

    size_t size() const { return size_; }
    size_t capacity() const { return capacity_; }
    bool empty() const { return size_ == 0; }
    bool full() const { return size_ == capacity_; }

    void clear()
    {
      head_ = 0;
      tail_ = 0;
      size_ = 0;
    }

    // Convert to vector for packet processing
    std::vector<uint8_t> to_vector() const
    {
      try {
        std::vector<uint8_t> result;
        if (size_ > capacity_) {
          // Invalid state, return empty vector
          return result;
        }
        result.reserve(size_);
        for (size_t i = 0; i < size_; i++) {
          size_t idx = (tail_ + i) % capacity_;
          if (idx >= capacity_) {
            // Invalid index, return partial result
            return result;
          }
          result.push_back(buffer_[idx]);
        }
        return result;
      } catch (...) {
        // Return empty vector on any exception
        return std::vector<uint8_t>();
      }
    }

  private:
    std::vector<uint8_t> buffer_;
    size_t head_;
    size_t tail_;
    size_t size_;
    size_t capacity_;
  };

  RingBuffer buffer_;

  // Parameters
  double wheel_radius_;
  double wheel_separation_;
  int max_rpm_;
  double cmd_vel_timeout_;
  bool invert_motor_l_;
  bool invert_motor_r_;

  // Encoder unwrapping for continuous angle
  int16_t prev_pos_l_raw_;
  int16_t prev_pos_r_raw_;
  double accumulated_pos_l_rad_;
  double accumulated_pos_r_rad_;
  bool first_pos_l_;
  bool first_pos_r_;

  rclcpp::Time last_cmd_vel_time_;

  // Service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_trigger_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_speed_r_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_speed_l_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_pos_r_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_pos_l_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_voltage_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr pub_timestamp_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_pos_l_rad_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_pos_r_rad_;

  // Subscriber
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;

  // Diagnostics
  diagnostic_updater::Updater diagnostic_updater_;
  std::unique_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> freq_diag_;
  rclcpp::TimerBase::SharedPtr diag_timer_;

  // Packet counter for initial flush
  size_t packet_count_;

  // Throttle log output (print every N packets)
  size_t log_counter_;
  static constexpr size_t LOG_INTERVAL = 5;  // Print every 5 packets (50ms)
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ESPSerialROS2>());
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Node terminated: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}