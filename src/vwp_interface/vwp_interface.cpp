#include <vwp_interface/vwp_interface.hpp>

#include <iostream>

VWPInterface::VWPInterface()
: Node("vwp_interface"), vehicle_info_(vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo())
{
  /* setup parameters */
  base_frame_id_ = declare_parameter("base_frame_id", "base_link");

  loop_rate_ = declare_parameter("loop_rate", 50.0);

  /* parameters for vehicle specifications */
  tire_radius_ = vehicle_info_.wheel_radius_m;
  wheel_base_ = vehicle_info_.wheel_base_m;

  std::cout << tire_radius_ << wheel_base_ << std::endl;

  steering_offset_ = declare_parameter("steering_offset", 0.0);
  // REVIEW
  enable_steering_rate_control_ = declare_parameter("enable_steering_rate_control", false);

  /* vehicle parameters */
  vgr_coef_a_ = declare_parameter("vgr_coef_a", 15.713);
  vgr_coef_b_ = declare_parameter("vgr_coef_b", 0.053);
  vgr_coef_c_ = declare_parameter("vgr_coef_c", 0.042);

  /* parameters for limitter */
  max_steering_wheel_ = declare_parameter("max_steering_wheel", 2.7 * M_PI);
  max_steering_wheel_rate_ = declare_parameter("max_steering_wheel_rate", 6.6);
  min_steering_wheel_rate_ = declare_parameter("min_steering_wheel_rate", 0.5);
  steering_wheel_rate_low_vel_ = declare_parameter("steering_wheel_rate_low_vel", 5.0);
  steering_wheel_rate_stopped_ = declare_parameter("steering_wheel_rate_stopped", 5.0);
  low_vel_thresh_ = declare_parameter("low_vel_thresh", 1.389);  // 5.0kmh

  /* initialize */
  prev_aw_cmd_msg_.header.stamp = this->now();
  prev_aw_cmd_msg_.aw_steer_wheel_cmd_rad = 0.0;

  /* subscribers */
  using std::placeholders::_1;
  using std::placeholders::_2;

  // From VehicleCAN
  vehicle_data_subscriber_ = create_subscription<vwp_msgs::msg::VehicleData>(
    "/vwp/vehicle_can/vehicle_data", 1, std::bind(&VWPInterface::callbackVehicleData, this, _1));

  // From autoware
  control_cmd_sub_ = create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
    "/control/command/control_cmd", 1, std::bind(&VWPInterface::callbackControlCmd, this, _1));

  // To CustomerCAN
  autoware_cmd_pub_ =
    create_publisher<vwp_msgs::msg::AutowareCommand>("/vwp/autoware_command", rclcpp::QoS{1});

  // Timer
  const auto period_ns = rclcpp::Rate(loop_rate_).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&VWPInterface::publishCommands, this));
}

void VWPInterface::publishCommands()
{
  /* guard */
  if (!vehicle_data_msg_ptr_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
      "Can't publish Commands to CustomerCAN. No VehicleData from VehicleCAN available.");
    return;
  }
  if (!control_cmd_ptr_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
      "Can't publish Commands to CustomerCAN. No Incoming AckermannControlCommand available.");
    return;
  }

  const rclcpp::Time current_time = get_clock()->now();
  const double current_velocity =
    calculateVehicleVelocity(*vehicle_data_msg_ptr_);  // current vehicle speed > 0 [m/s]

  // Steering
  const double current_steer_wheel = vehicle_data_msg_ptr_->steering_wheel_angle_rad;

  const double adaptive_gear_ratio =
    calculateVariableGearRatio(current_velocity, current_steer_wheel);
  double desired_steer_wheel =
    (control_cmd_ptr_->lateral.steering_tire_angle - steering_offset_) * adaptive_gear_ratio;

  desired_steer_wheel =
    std::min(std::max(desired_steer_wheel, -max_steering_wheel_), max_steering_wheel_);
  // Completed

  vwp_msgs::msg::AutowareCommand aw_cmd_msg;

  aw_cmd_msg.header.frame_id = base_frame_id_;
  aw_cmd_msg.header.stamp = current_time;
  aw_cmd_msg.aw_steer_rot_rate_rads = calcSteerWheelRateCmd(adaptive_gear_ratio);

  aw_cmd_msg.aw_steer_wheel_cmd_rad = steerWheelRateLimiter(
    desired_steer_wheel, prev_aw_cmd_msg_.aw_steer_wheel_cmd_rad, current_time,
    prev_aw_cmd_msg_.header.stamp, aw_cmd_msg.aw_steer_rot_rate_rads);
  // Important: Always at the end of this function
  autoware_cmd_pub_->publish(aw_cmd_msg);
  prev_aw_cmd_msg_ = aw_cmd_msg;
}

void VWPInterface::callbackControlCmd(
  const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg)
{
  control_command_received_time_ = this->now();
  control_cmd_ptr_ = msg;
}

// Done
void VWPInterface::callbackVehicleData(
  const vwp_msgs::msg::VehicleData::ConstSharedPtr vehicle_data_msg)
{
  vehicle_data_msg_ptr_ = vehicle_data_msg;
}

double VWPInterface::steerWheelRateLimiter(
  const double current_steer_cmd, const double prev_steer_cmd,
  const rclcpp::Time & current_steer_time, const rclcpp::Time & prev_steer_time,
  const double steer_rate)
{
  const double dsteer = current_steer_cmd - prev_steer_cmd;
  const double dt = std::max(0.0, (current_steer_time - prev_steer_time).seconds());
  const double max_dsteer = std::fabs(steer_rate) * dt;
  const double limited_steer_cmd =
    prev_steer_cmd + std::min(std::max(-max_dsteer, dsteer), max_dsteer);
  return limited_steer_cmd;
}

double VWPInterface::calculateVariableGearRatio(const double vel, const double steer_wheel)
{
  return std::max(
    1e-5, vgr_coef_a_ + vgr_coef_b_ * vel * vel - vgr_coef_c_ * std::fabs(steer_wheel));
}

// Completed
double VWPInterface::calcSteerWheelRateCmd(const double gear_ratio)
{
  const auto current_vel = std::fabs(calculateVehicleVelocity(*vehicle_data_msg_ptr_));

  // send low steer rate at low speed
  if (current_vel < std::numeric_limits<double>::epsilon()) {
    return steering_wheel_rate_stopped_;
  } else if (current_vel < low_vel_thresh_) {
    return steering_wheel_rate_low_vel_;
  }

  if (!enable_steering_rate_control_) {
    return max_steering_wheel_rate_;
  }

  constexpr double margin = 1.5;
  const double rate = margin * control_cmd_ptr_->lateral.steering_tire_rotation_rate * gear_ratio;
  return std::min(std::max(std::fabs(rate), min_steering_wheel_rate_), max_steering_wheel_rate_);
}

// Completed
double VWPInterface::calculateVehicleVelocity(const vwp_msgs::msg::VehicleData & vehicle_data_msg)
{
  const double sign =
    (vehicle_data_msg.gear_lever_pos == vwp_msgs::msg::VehicleData::SHIFT_REVERSE) ? -1 : 1;
  const double vel = (vehicle_data_msg.wheel_speed_rl_ms + vehicle_data_msg.wheel_speed_rr_ms) *
                     0.5 * tire_radius_ * speed_scale_factor_;

  return sign * vel;
}