// TODO: 1. Move all from vehicle_can_node to here:
// TODO: 2. Create Composition: 
//                  - Publish Autoware Msgs
//                  - Publish Autoware Cmds


void VehicleCanNode::publish_autoware_msgs() {
    velocity_report_msg_.header.frame_id = "base_link"
    velocity_report_msg_.header.stamp = rclcpp::Clock().now();
    velocity_report_msg_.longitudinal_velocity = vehicleCan.egoData.vehicleVelocity / 3.6f;
    velocity_report_msg_.lateral_velocity = 0.0f;
    velocity_report_msg_heading_rate = 0.0f;
    velocity_report_publisher_->publish(velocity_report_msg_heading_rate);
}