// TODO: 1. Move all from vehicle_can_node to here:
// TODO: 2. Create Composition: 
//                  - Publish Autoware Msgs
//                  - Publish Autoware Cmds

#include "vehicle_status_publisher/vehicle_status_publisher.hpp"

using namespace std::chrono_literals;

namespace vwp_interface
{
VehicleStatusPublisher::VehicleStatusPublisher() : Node("vehicle_status_publisher"), vehicle_info_(vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo())
{
  /* setup parameters */
  base_frame_id_ = declare_parameter("base_frame_id", "base_link");

  /* parameters for vehicle specifications */
  tire_radius_ = vehicle_info_.wheel_radius_m;
  wheel_base_ = vehicle_info_.wheel_base_m;

  steering_offset_ = declare_parameter("steering_offset", 0.0);

  /* vehicle parameters */
  vgr_coef_a_ = declare_parameter("vgr_coef_a", 15.713);
  vgr_coef_b_ = declare_parameter("vgr_coef_b", 0.053);
  vgr_coef_c_ = declare_parameter("vgr_coef_c", 0.042);

  velocity_report_publisher_ = create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>("/vehicle/status/velocity_status", 1);
  timer_ = create_wall_timer(20ms, std::bind(&VehicleStatusPublisher::publish, this));

  vehicle_data_subscriber_ = create_subscription<vwp_msgs::msg::VehicleData>(
    "/vwp/vehicle_can/vehicle_data", rclcpp::QoS{1}, std::bind(&VehicleStatusPublisher::callbackVehicleData, this, std::placeholders::_1)); 

  position_data_subscriber_ = create_subscription<vwp_msgs::msg::PositionDataVehicleCan>(
    "/vwp/vehicle_can/position_data", rclcpp::QoS{1}, std::bind(&VehicleStatusPublisher::callbackPositionData, this, std::placeholders::_1));   
}

void VehicleStatusPublisher::callbackVehicleData(const vwp_msgs::msg::VehicleData::ConstSharedPtr vehicle_data_msg_ptr) {

  currentData.vehicleVelocity = vehicle_data_msg_ptr->vehicle_velocity;
  currentData.steeringWheelAngle = vehicle_data_msg_ptr->steering_wheel_angle;

}

void VehicleStatusPublisher::callbackPositionData(const vwp_msgs::msg::PositionDataVehicleCan::ConstSharedPtr position_data_msg_ptr) {
  auto x = position_data_msg_ptr->heading_direction;
  std::cout << x << std::endl;
}

void VehicleStatusPublisher::publish() 
{

  const double current_velocity = calculateVehicleVelocity(
    *wheel_speed_rpt_ptr_, *gear_cmd_rpt_ptr_);  // current vehicle speed > 0 [m/s]
  const double current_steer_wheel =
    steer_wheel_rpt_ptr_->output;  // current vehicle steering wheel angle [rad]
  const double adaptive_gear_ratio =
    calculateVariableGearRatio(current_velocity, current_steer_wheel);
  const double current_steer = current_steer_wheel / adaptive_gear_ratio + steering_offset_;


  std_msgs::msg::Header header;
  header.frame_id = base_frame_id_;
  header.stamp = get_clock()->now();

  /* publish steering wheel status */
  {
    steering_report_msg_.stamp = header.stamp;
    steering_report_msg_.steering_tire_angle = currentData.steeringWheelAngle
    steering_report_publisher_->publish(steering_report_msg_);
  }

  /* publish velocity status */
  {
    velocity_report_msg_.header = header;
    velocity_report_msg_.longitudinal_velocity = currentData.vehicleVelocity / 3.6f; // [m/s]
    velocity_report_msg_.heading_rate = currentData.vehicleVelocity * std::tan(currentData.steeringWheelAngle) / wheel_base_;  // [rad/s]
    velocity_report_publisher_->publish(velocity_report_msg_);
  }


}

} // namespace compostion

//#include "rclcpp_components/register_node_macro.hpp"

//RCLCPP_COMPONENTS_REGISTER_NODE(vwp_interface::VehicleStatusPublisher)