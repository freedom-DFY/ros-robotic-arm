#include "five_dof_arm/config.hpp"
#include <dynamixel_sdk_custom_interfaces/msg/sync_set_position.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <chrono>

namespace five_dof_arm
{
using SetPosition = dynamixel_sdk_custom_interfaces::msg::SyncSetPosition;

class ArmController : public rclcpp::Node
{
public:
  ArmController() : Node("logicool_controller"), config_(*this), position_(config_.initial)
  {
    kin_.a2 = declare_parameter("link_a2_mm", 246.06);
    kin_.a3 = declare_parameter("link_a3_mm", 242.25);
    kin_.tool = declare_parameter("tool_length_mm", 255.55);
    cartesian_step_ = declare_parameter("cartesian_step_mm", 10.0);
    layout_ = declare_parameter("joy_layout", std::string("sdl"));
    if (layout_ != "sdl" && layout_ != "legacy") {
      throw std::invalid_argument("joy_layout must be sdl or legacy");
    }
    if (!std::isfinite(kin_.a2) || !std::isfinite(kin_.a3) || !std::isfinite(kin_.tool) ||
      !std::isfinite(cartesian_step_) || kin_.a2 <= 0 || kin_.a3 <= 0 || kin_.tool <= 0 || cartesian_step_ <= 0)
    {
      throw std::invalid_argument("Link dimensions and Cartesian step must be finite and positive");
    }
    command_pub_ = create_publisher<SetPosition>("sync_set_position", 10);
    point_pub_ = create_publisher<geometry_msgs::msg::Point>("end_effector_position", 10);
    state_sub_ = create_subscription<sensor_msgs::msg::JointState>("joint_states", 10,
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
        Positions p;
        for (size_t i = 0; i < 5; ++i) {
          const auto it = std::find(msg->name.begin(), msg->name.end(), "joint_" + std::to_string(i + 1));
          if (it == msg->name.end()) {return;}
          const auto index = static_cast<size_t>(std::distance(msg->name.begin(), it));
          if (index >= msg->position.size() || !std::isfinite(msg->position[index]) ||
            std::abs(msg->position[index]) > 2 * pi) {return;}
          p[i] = radians_to_ticks(msg->position[index]);
        }
        position_ = p;
        have_state_ = true;
        last_state_ = std::chrono::steady_clock::now();
        const auto t = kin_.forward(position_);
        geometry_msgs::msg::Point point;
        point.x = t(0, 3); point.y = t(1, 3); point.z = t(2, 3);
        point_pub_->publish(point);
      });
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>("joy", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Joy::ConstSharedPtr msg) {on_joy(*msg);});
    target_sub_ = create_subscription<geometry_msgs::msg::Point>("auto_transform", 10,
      [this](geometry_msgs::msg::Point::ConstSharedPtr msg) {
        if (!ready()) {return;}
        Eigen::Vector3d target(msg->x, msg->y, msg->z);
        move_cartesian(target);
      });
    RCLCPP_INFO(get_logger(), "Controller ready, joystick layout=%s; waiting for joint_states", layout_.c_str());
  }

private:
  bool ready()
  {
    if (!have_state_ || std::chrono::steady_clock::now() - last_state_ > std::chrono::seconds(1)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for fresh joint_states from the driver");
      return false;
    }
    return true;
  }
  void publish(const Positions & p)
  {
    if (!config_.valid(p)) {
      RCLCPP_WARN(get_logger(), "Rejected position outside configured limits"); return;
    }
    SetPosition msg;
    msg.id1 = config_.ids[0]; msg.id2 = config_.ids[1]; msg.id3 = config_.ids[2];
    msg.id4 = config_.ids[3]; msg.id5 = config_.ids[4];
    msg.position1 = p[0]; msg.position2 = p[1]; msg.position3 = p[2];
    msg.position4 = p[3]; msg.position5 = p[4];
    command_pub_->publish(msg);
  }
  void move_cartesian(const Eigen::Vector3d & xyz)
  {
    const Eigen::Matrix4d target = kin_.position_target(position_, xyz);
    const auto solution = kin_.inverse(target, position_, config_.lower, config_.upper);
    if (!solution) {
      RCLCPP_WARN(get_logger(), "Rejected target: unreachable with current tool pitch/roll and joint limits");
      return;
    }
    publish(*solution);
  }
  void on_joy(const sensor_msgs::msg::Joy & msg)
  {
    const bool legacy = layout_ == "legacy";
    // SDL uses game_controller_node's standard mapping. Legacy is the original
    // Logitech DirectInput layout (6 axes/12 buttons) supplied with this arm.
    if (msg.axes.size() < 6 || msg.buttons.size() < (legacy ? 10u : 15u) ||
      !std::all_of(msg.axes.begin(), msg.axes.end(), [](float x) {return std::isfinite(x);}))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Unexpected Joy layout; check joy_layout and controller mode");
      return;
    }
    const auto button = [&msg](size_t i) {return msg.buttons[i] != 0;};
    const bool back = button(legacy ? 8 : 4);
    if (back && !back_pressed_) {
      task_mode_ = !task_mode_;
      RCLCPP_INFO(get_logger(), "Joystick mode: %s", task_mode_ ? "Cartesian" : "joint");
    }
    back_pressed_ = back;
    if (!ready()) {return;}
    // SDL controller indices follow SDL_GameControllerButton:
    // A/B/X/Y=0/1/2/3, back=4, start=6, shoulders=9/10, D-pad=11..14.
    const bool x = button(legacy ? 0 : 2), a = button(legacy ? 1 : 0);
    const bool b = button(legacy ? 2 : 1), y = button(3);
    const bool lb = button(legacy ? 4 : 9), rb = button(legacy ? 5 : 10);
    const bool lt = legacy ? button(6) : msg.axes[4] < 0.0f;
    const bool rt = legacy ? button(7) : msg.axes[5] < 0.0f;
    const bool start = button(legacy ? 9 : 6);
    const int ud = legacy ? (msg.axes[5] > 0.5f ? 1 : (msg.axes[5] < -0.5f ? -1 : 0)) :
      (button(11) ? 1 : (button(12) ? -1 : 0));
    const int lr = legacy ? (msg.axes[4] > 0.5f ? 1 : (msg.axes[4] < -0.5f ? -1 : 0)) :
      (button(13) ? 1 : (button(14) ? -1 : 0));
    Positions p = position_;
    bool changed = false;
    // Presets bypass IK in both modes, fixing the original task-mode overwrite.
    if ((!task_mode_ && y) || (task_mode_ && lb)) {p = config_.initial; changed = true;}
    else if ((!task_mode_ && a) || (task_mode_ && rb)) {p = {2048, 2048, 2048, 2048, 2048}; changed = true;}
    else if ((!task_mode_ && start) || (task_mode_ && rt)) {p = {2048, 2240, 1480, 2910, 2048}; changed = true;}
    else if (task_mode_) {
      const int dz = y ? 1 : (a ? -1 : 0);
      if (ud || lr || dz) {
        const Eigen::Matrix4d current = kin_.forward(position_);
        move_cartesian(current.block<3, 1>(0, 3) + cartesian_step_ * Eigen::Vector3d(ud, lr, dz));
      }
      return;
    } else {
      p[0] += lt ? -16 : (rt ? 16 : 0);
      p[1] += lb ? -8 : (rb ? 8 : 0);
      p[2] += ud * 8; p[3] += lr * 16;
      p[4] += x ? 16 : (b ? -16 : 0);
      changed = p != position_;
    }
    if (changed) {publish(p);}
  }
  Config config_;
  Kinematics kin_;
  Positions position_;
  bool have_state_ = false, task_mode_ = false, back_pressed_ = false;
  double cartesian_step_ = 10;
  std::string layout_;
  std::chrono::steady_clock::time_point last_state_;
  rclcpp::Publisher<SetPosition>::SharedPtr command_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr point_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_sub_;
};
}  // namespace five_dof_arm

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {rclcpp::spin(std::make_shared<five_dof_arm::ArmController>());}
  catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("logicool_controller"), "%s", e.what()); result = 1;
  }
  rclcpp::shutdown();
  return result;
}
