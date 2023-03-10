#ifndef COMPOSITION__VEHICLE_STATUS_PUBLISHER_HPP_
#define COMPOSITION__VEHICLE_STATUS_PUBLISHER_HPP_

#include <chrono>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <vehicle_info_util/vehicle_info_util.hpp>


// VWP Messages
#include "vwp_msgs/msg/vehicle_data.hpp"
#include "vwp_msgs/msg/position_data_vehicle_can.hpp"


// AutowareAuto Messages
#include <autoware_auto_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <tier4_vehicle_msgs/msg/actuation_status_stamped.hpp>

namespace vwp_interface
{
class VehicleStatusPublisher : public rclcpp::Node
{
public:
  explicit VehicleStatusPublisher();

protected:
  void publish();

private:


  

  rclcpp::TimerBase::SharedPtr timer_;

  // Subscriber: VehicleData
  rclcpp::Subscription<vwp_msgs::msg::VehicleData>::SharedPtr vehicle_data_subscriber_;
  vwp_msgs::msg::VehicleData vehicle_data_msg_;
  void callbackVehicleData(const vwp_msgs::msg::VehicleData::ConstSharedPtr vehicle_data_msg_ptr);

  // Subscriber: PositionDataFromCan
  rclcpp::Subscription<vwp_msgs::msg::PositionDataVehicleCan>::SharedPtr position_data_subscriber_;
  vwp_msgs::msg::PositionDataVehicleCan position_data_msg_;
  void callbackPositionData(const vwp_msgs::msg::PositionDataVehicleCan::ConstSharedPtr position_data_msg_ptr);


  // To Autoware Messages
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>::SharedPtr control_mode_report_publisher_;
  autoware_auto_vehicle_msgs::msg::ControlModeReport control_mode_report_msg_;

  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_report_publisher_;
  autoware_auto_vehicle_msgs::msg::VelocityReport velocity_report_msg_; 

  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_report_publisher_;
  autoware_auto_vehicle_msgs::msg::SteeringReport steering_report_msg_; 

  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::GearReport>::SharedPtr gear_report_publisher_;
  autoware_auto_vehicle_msgs::msg::GearReport gear_report_msg_; 
  
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr turn_indicators_report_publisher_;
  autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport turn_indicators_report_msg_; 

  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::HazardLightsReport>::SharedPtr hazard_lights_report_publisher_;
  autoware_auto_vehicle_msgs::msg::HazardLightsReport hazard_lights_report_msg_; 

  rclcpp::Publisher<tier4_vehicle_msgs::msg::ActuationStatusStamped>::SharedPtr actuation_status_report_publisher_;
  tier4_vehicle_msgs::msg::ActuationStatusStamped actuation_status_report_msg_;

  struct subscribedData
    {
        uint8_t   timestamp      = 0;               // Software timestamp in us                                   
        uint8_t   gearLeverPos    = 0;              // 0 Intermediate, 1 = Init, 5 = Park, 6 = Reverse, 7 = Neutral, 8 = Drive, 
                                                    // 9 = Sport, 10 = Efficient, 13 = Tip in S, 14 = Tip in D, 15 = Error             
        float   vehicleVelocity  = 0.0f;          // Speed in km/h                                         
        float   steeringWheelAngle = 0.0f;        // [deg]                                 
    }; 

  subscribedData currentData;

  /* ros param */
  std::string base_frame_id_;
  double loop_rate_;           // [Hz]
  double wheel_base_;          // [m]
  double tire_radius_;         // [m]
  vehicle_info_util::VehicleInfo vehicle_info_;

  

};
}

#endif // COMPOSITION__VEHICLE_STATUS_PUBLISHER_HPP_