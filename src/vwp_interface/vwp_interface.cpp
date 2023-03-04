#include <vwp_interface/vwp_interface.hpp>
#include <iostream>

VWPInterface::VWPInterface(): Node("vwp_interface"),
  vehicle_info_(vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo())
{
  std::cout << "TEST" << std::endl;
  // From VWP (VehicleCANNode)
  vwp_autoware_sub_ = create_subscription<vwp_msgs::msg::ToAutoware>(
    "/vwp/vehicle_can/to_autoware", rclcpp::QoS{1}, std::bind(&VWPInterface::callbackVWPRpt, this, std::placeholders::_1));  
  
 
  std::cout << vehicle_info_.wheel_radius_m << std::endl;
}

void VWPInterface::callbackVWPRpt(const vwp_msgs::msg::ToAutoware::ConstSharedPtr msg) {


}

void VWPInterface::publishCommands() {

}
