// Adapted from the ROBOTIS SDK sync read/write example, Apache-2.0.
#include "five_dof_arm/config.hpp"
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <dynamixel_sdk_custom_interfaces/msg/sync_set_position.hpp>
#include <dynamixel_sdk_custom_interfaces/srv/sync_get_position.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <chrono>
#include <memory>

namespace five_dof_arm
{
using SetPosition = dynamixel_sdk_custom_interfaces::msg::SyncSetPosition;
using GetPosition = dynamixel_sdk_custom_interfaces::srv::SyncGetPosition;

class ArmDriver : public rclcpp::Node
{
public:
  ArmDriver() : Node("sync_read_write_node"), config_(*this), position_(config_.initial)
  {
    dry_run_ = declare_parameter("dry_run", false);
    disable_on_exit_ = declare_parameter("disable_torque_on_shutdown", true);
    const auto device = declare_parameter("device_name", std::string("/dev/ttyUSB0"));
    const auto baud = declare_parameter("baud_rate", 1000000);
    const auto velocity = declare_parameter("profile_velocity", 1000);
    const auto acceleration = declare_parameter("profile_acceleration", 500);
    const double rate = declare_parameter("publish_rate", 50.0);
    if (!std::isfinite(rate) || rate <= 0 || rate > 200 || baud <= 0 ||
      velocity <= 0 || acceleration <= 0)
    {
      throw std::invalid_argument("Invalid rate, baud or motion profile parameters");
    }
    if (!dry_run_) {
      port_.reset(dynamixel::PortHandler::getPortHandler(device.c_str()));
      packet_ = dynamixel::PacketHandler::getPacketHandler(2.0);
      try {
        if (!port_->openPort()) {throw std::runtime_error("Cannot open " + device);}
        opened_ = true;
        if (!port_->setBaudRate(baud)) {throw std::runtime_error("Cannot set baud rate");}
        reader_ = std::make_unique<dynamixel::GroupSyncRead>(port_.get(), packet_, 132, 4);
        writer_ = std::make_unique<dynamixel::GroupSyncWrite>(port_.get(), packet_, 116, 4);
        for (auto id : config_.ids) {
          if (!reader_->addParam(id)) {throw std::runtime_error("Cannot configure sync reader");}
          write1(id, 64, 0);
          write1(id, 11, 3);  // X/MX(2.0) single-turn position mode.
          write4(id, 108, static_cast<uint32_t>(acceleration));
          write4(id, 112, static_cast<uint32_t>(velocity));
        }
        if (!read_positions()) {throw std::runtime_error("Cannot read all five motors at startup");}
        // Seed the goal from the actual pose before enabling torque.
        if (!send_positions(position_)) {throw std::runtime_error("Cannot seed motor goals");}
        for (auto id : config_.ids) {write1(id, 64, 1);}
      } catch (...) {close(true); throw;}
    }
    state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    command_sub_ = create_subscription<SetPosition>("sync_set_position", 10,
      [this](SetPosition::ConstSharedPtr msg) {
        const std::array<uint8_t, 5> ids{msg->id1, msg->id2, msg->id3, msg->id4, msg->id5};
        const Positions incoming{msg->position1, msg->position2, msg->position3, msg->position4, msg->position5};
        Positions ordered;
        if (!order_positions(ids, incoming, ordered) || !config_.valid(ordered)) {
          RCLCPP_ERROR(get_logger(), "Rejected command: IDs must match motor_ids and positions must be within limits");
          return;
        }
        if (send_positions(ordered) && dry_run_) {position_ = ordered;}
      });
    service_ = create_service<GetPosition>("sync_get_position",
      [this](const GetPosition::Request::SharedPtr req, GetPosition::Response::SharedPtr res) {
        const std::array<uint8_t, 5> ids{req->id1, req->id2, req->id3, req->id4, req->id5};
        Positions unused;
        if (!order_positions(ids, position_, unused)) {
          res->success = false; res->message = "Request IDs must match motor_ids exactly"; return;
        }
        if (!dry_run_ && !read_positions()) {
          res->success = false; res->message = "Failed to read all five motors"; return;
        }
        Positions p;
        for (size_t i = 0; i < 5; ++i) {
          const auto it = std::find(config_.ids.begin(), config_.ids.end(), ids[i]);
          p[i] = position_[std::distance(config_.ids.begin(), it)];
        }
        res->position1 = p[0]; res->position2 = p[1]; res->position3 = p[2];
        res->position4 = p[3]; res->position5 = p[4];
        res->success = true; res->message = dry_run_ ? "dry_run" : "ok";
      });
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this]() {
      if (!dry_run_ && !read_positions()) {return;}
      sensor_msgs::msg::JointState msg;
      msg.header.stamp = now();
      for (size_t i = 0; i < 5; ++i) {
        msg.name.push_back("joint_" + std::to_string(i + 1));
        msg.position.push_back(ticks_to_radians(position_[i]));
      }
      state_pub_->publish(msg);
    });
    RCLCPP_INFO(get_logger(), "Five DOF driver ready (%s)", dry_run_ ? "dry_run: no serial I/O" : device.c_str());
  }
  ~ArmDriver() override {close(disable_on_exit_);}

private:
  bool order_positions(const std::array<uint8_t, 5> & ids, const Positions & p, Positions & out)
  {
    auto sorted = ids, expected = config_.ids;
    std::sort(sorted.begin(), sorted.end()); std::sort(expected.begin(), expected.end());
    if (sorted != expected) {return false;}
    for (size_t i = 0; i < 5; ++i) {
      const auto it = std::find(config_.ids.begin(), config_.ids.end(), ids[i]);
      out[std::distance(config_.ids.begin(), it)] = p[i];
    }
    return true;
  }
  void check(int result, uint8_t error, uint8_t id)
  {
    if (result != COMM_SUCCESS || error != 0) {
      throw std::runtime_error("Motor " + std::to_string(id) + ": " +
        (result != COMM_SUCCESS ? packet_->getTxRxResult(result) : packet_->getRxPacketError(error)));
    }
  }
  void write1(uint8_t id, uint16_t address, uint8_t value)
  {
    uint8_t error = 0;
    const int result = packet_->write1ByteTxRx(port_.get(), id, address, value, &error);
    check(result, error, id);
  }
  void write4(uint8_t id, uint16_t address, uint32_t value)
  {
    uint8_t error = 0;
    const int result = packet_->write4ByteTxRx(port_.get(), id, address, value, &error);
    check(result, error, id);
  }
  bool read_positions()
  {
    const int result = reader_->txRxPacket();
    if (result != COMM_SUCCESS) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Read failed: %s", packet_->getTxRxResult(result));
      return false;
    }
    Positions p;
    for (size_t i = 0; i < 5; ++i) {
      uint8_t error = 0;
      if (!reader_->isAvailable(config_.ids[i], 132, 4) ||
        !reader_->getError(config_.ids[i], &error) || error != 0)
      {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Missing/error response for motor %u", config_.ids[i]);
        return false;
      }
      p[i] = static_cast<int32_t>(reader_->getData(config_.ids[i], 132, 4));
    }
    position_ = p;
    return true;
  }
  bool send_positions(const Positions & p)
  {
    if (dry_run_) {return true;}
    writer_->clearParam();
    for (size_t i = 0; i < 5; ++i) {
      const uint32_t value = static_cast<uint32_t>(p[i]);
      uint8_t data[4]{DXL_LOBYTE(DXL_LOWORD(value)), DXL_HIBYTE(DXL_LOWORD(value)),
        DXL_LOBYTE(DXL_HIWORD(value)), DXL_HIBYTE(DXL_HIWORD(value))};
      if (!writer_->addParam(config_.ids[i], data)) {
        writer_->clearParam(); RCLCPP_ERROR(get_logger(), "Cannot prepare sync write"); return false;
      }
    }
    const int result = writer_->txPacket();
    writer_->clearParam();
    if (result != COMM_SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Write failed: %s", packet_->getTxRxResult(result)); return false;
    }
    return true;
  }
  void close(bool disable)
  {
    if (!opened_) {return;}
    if (disable) {
      for (auto id : config_.ids) {
        uint8_t error = 0;
        packet_->write1ByteTxRx(port_.get(), id, 64, 0, &error);
      }
    }
    port_->closePort(); opened_ = false;
  }
  Config config_;
  Positions position_;
  bool dry_run_ = false, disable_on_exit_ = true, opened_ = false;
  std::unique_ptr<dynamixel::PortHandler> port_;
  dynamixel::PacketHandler * packet_ = nullptr;  // SDK-owned singleton.
  std::unique_ptr<dynamixel::GroupSyncRead> reader_;
  std::unique_ptr<dynamixel::GroupSyncWrite> writer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;
  rclcpp::Subscription<SetPosition>::SharedPtr command_sub_;
  rclcpp::Service<GetPosition>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace five_dof_arm

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {rclcpp::spin(std::make_shared<five_dof_arm::ArmDriver>());}
  catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("sync_read_write_node"), "%s", e.what()); result = 1;
  }
  rclcpp::shutdown();
  return result;
}
