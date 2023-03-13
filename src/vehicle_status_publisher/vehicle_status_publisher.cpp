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
  std::cout << "1" << std::endl;
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
  steering_report_publisher_ = create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>("/vehicle/status/steering_status", 1);

  timer_ = create_wall_timer(20ms, std::bind(&VehicleStatusPublisher::publish, this));


  vehicle_data_subscriber_ = create_subscription<vwp_msgs::msg::VehicleData>(
    "/vwp/vehicle_can/vehicle_data", rclcpp::QoS{1}, std::bind(&VehicleStatusPublisher::callbackVehicleData, this, std::placeholders::_1)); 

  position_data_subscriber_ = create_subscription<vwp_msgs::msg::PositionDataVehicleCan>(
    "/vwp/vehicle_can/position_data", rclcpp::QoS{1}, std::bind(&VehicleStatusPublisher::callbackPositionData, this, std::placeholders::_1));   
    
}

void VehicleStatusPublisher::callbackVehicleData(const vwp_msgs::msg::VehicleData::ConstSharedPtr vehicle_data_msg_ptr) {

  currentData.vehicleVelocity = vehicle_data_msg_ptr->vehicle_velocity;
  
  using tier4_autoware_utils::deg2rad;
  currentData.steeringWheelAngleRad = deg2rad(vehicle_data_msg_ptr->steering_wheel_angle);

  currentData.wheel_speed_rl_ms = vehicle_data_msg_ptr->wheel_speed_rl / 3.6f;
  currentData.wheel_speed_rr_ms = vehicle_data_msg_ptr->wheel_speed_rr / 3.6f;
}

void VehicleStatusPublisher::callbackPositionData(const vwp_msgs::msg::PositionDataVehicleCan::ConstSharedPtr position_data_msg_ptr) {
  auto x = position_data_msg_ptr->heading_direction;
}

void VehicleStatusPublisher::publish() 
{
  const double current_velocity = currentData.vehicleVelocity / 3.6f; //calculateVehicleVelocity();  // current vehicle speed > 0 [m/s]
  const double current_steer_wheel = currentData.steeringWheelAngleRad; // current vehicle steering wheel angle [rad]
  //TODO: Check, whether this is correct for our VW Passat?! 
  const double adaptive_gear_ratio =
    calculateVariableGearRatio(current_velocity, current_steer_wheel);
  const double current_steer = current_steer_wheel / adaptive_gear_ratio + steering_offset_;
  

  std_msgs::msg::Header header;
  header.frame_id = base_frame_id_;
  header.stamp = get_clock()->now();

  /* publish steering report */
  {
    steering_report_msg_.stamp = header.stamp;
    steering_report_msg_.steering_tire_angle = current_steer;
    steering_report_publisher_->publish(steering_report_msg_);
  }

  /* publish velocity report */
  {
    velocity_report_msg_.header = header;
    velocity_report_msg_.longitudinal_velocity = current_velocity; // m/s
    velocity_report_msg_.heading_rate = current_velocity * std::tan(current_steer) / wheel_base_;  // [rad/s]
    velocity_report_publisher_->publish(velocity_report_msg_);
  }
}

double VehicleStatusPublisher::calculateVehicleVelocity() 
{
  const double vel = (currentData.wheel_speed_rr_ms + currentData.wheel_speed_rl_ms) * 0.5 *
    tire_radius_ * speed_scale_factor_;
  return vel;
}

double VehicleStatusPublisher::calculateVariableGearRatio(const double vel, const double steer_wheel)
{
  return std::max(
    1e-5, vgr_coef_a_ + vgr_coef_b_ * vel * vel - vgr_coef_c_ * std::fabs(steer_wheel));
}

} // namespace compostion

//#include "rclcpp_components/register_node_macro.hpp"

//RCLCPP_COMPONENTS_REGISTER_NODE(vwp_interface::VehicleStatusPublisher)