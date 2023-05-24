#ifndef VWP_INTERFACE__VWP_INTERFACE_HPP_
#define VWP_INTERFACE__VWP_INTERFACE_HPP_

#include "tier4_autoware_utils/math/unit_conversion.hpp"

#include <rclcpp/rclcpp.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <memory>

// Messages
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <tier4_vehicle_msgs/msg/steering_wheel_status_stamped.hpp>
#include <vwp_msgs/msg/autoware_command.hpp>
#include <vwp_msgs/msg/vehicle_data.hpp>

class VWPInterface : public rclcpp::Node
{
public:
  using SteeringWheelStatusStamped = tier4_vehicle_msgs::msg::SteeringWheelStatusStamped;

  VWPInterface();

private:
  // ros params
  std::string base_frame_id_;

  double loop_rate_;                    // [Hz]
  double vgr_coef_a_;                   // variable gear ratio coeffs
  double vgr_coef_b_;                   // variable gear ratio coeffs
  double vgr_coef_c_;                   // variable gear ratio coeffs
  double tire_radius_;                  // [m]
  double wheel_base_;                   // [m]
  double max_steering_wheel_;           // max steering wheel angle [rad]
  double max_steering_wheel_rate_;      // [rad/s]
  double min_steering_wheel_rate_;      // [rad/s]
  double steering_offset_;              // [rad] def: measured = truth + offset
  double speed_scale_factor_;           // scale factor of speed
  double steering_wheel_rate_stopped_;  // [rad/s]
  double steering_wheel_rate_low_vel_;  // [rad/s]
  double low_vel_thresh_;               // [m/s]

  bool enable_steering_rate_control_;  // use steering angle speed for command [rad/s]

  vehicle_info_util::VehicleInfo vehicle_info_;

  // Subscriber: VehicleData
  rclcpp::Subscription<vwp_msgs::msg::VehicleData>::SharedPtr vehicle_data_subscriber_;
  vwp_msgs::msg::VehicleData::ConstSharedPtr vehicle_data_msg_ptr_;
  void callbackVehicleData(const vwp_msgs::msg::VehicleData::ConstSharedPtr vehicle_data_msg);

  // From Autoware
  rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr
    control_cmd_sub_;
  autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr control_cmd_ptr_;

  void callbackControlCmd(
    const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg);

  // To CustomerCAN
  rclcpp::Publisher<vwp_msgs::msg::AutowareCommand>::SharedPtr autoware_cmd_pub_;
  vwp_msgs::msg::AutowareCommand prev_aw_cmd_msg_;

  // Functions
  void publishCommands();
  double calculateVehicleVelocity(const vwp_msgs::msg::VehicleData & vehicle_data_msg);
  double calculateVariableGearRatio(const double vel, const double steer_wheel);
  double calcSteerWheelRateCmd(const double gear_ratio);
  double steerWheelRateLimiter(
    const double current_steer_cmd, const double prev_steer_cmd,
    const rclcpp::Time & current_steer_time, const rclcpp::Time & prev_steer_time,
    const double steer_rate);

  // input values
  rclcpp::Time control_command_received_time_;

  // Misc.
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif