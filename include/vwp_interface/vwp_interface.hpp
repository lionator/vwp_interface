#ifndef VWP_INTERFACE__VWP_INTERFACE_HPP_
#define VWP_INTERFACE__VWP_INTERFACE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>


// Messages
#include <tier4_vehicle_msgs/msg/steering_wheel_status_stamped.hpp>
#include <vwp_msgs/msg/to_autoware.hpp>


class VWPInterface : public rclcpp::Node
{
public:
  using SteeringWheelStatusStamped = tier4_vehicle_msgs::msg::SteeringWheelStatusStamped;

  VWPInterface();
private:
  // ros params
  double loop_rate_;           // [Hz]
  
  vehicle_info_util::VehicleInfo vehicle_info_;


  // Subscriptions
  rclcpp::Subscription<vwp_msgs::msg::ToAutoware>::SharedPtr vwp_autoware_sub_;

  // Callbacks
  void callbackVWPRpt(const vwp_msgs::msg::ToAutoware::ConstSharedPtr msg);

  // Functions
  void publishCommands();


  // Misc.
  rclcpp::TimerBase::SharedPtr timer_;

};

#endif 