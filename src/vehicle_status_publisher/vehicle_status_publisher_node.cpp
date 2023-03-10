#include <vehicle_status_publisher/vehicle_status_publisher.hpp>
#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vwp_interface::VehicleStatusPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  
  return 0;
}
