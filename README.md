# vwp_interface

`vwp_interface`is the package to connect Autoware with Pacmod.

## Input / Output

### Input topics

- From Autoware

  | Name                                   | Type                                                     | Description                                           |
  | -------------------------------------- | -------------------------------------------------------- | ----------------------------------------------------- |
  | `/control/command/control_cmd`         | autoware_auto_control_msgs::msg::AckermannControlCommand | lateral and longitudinal control command              |
  | `/control/command/gear_cmd`            | autoware_auto_vehicle_msgs::msg::GearCommand             | gear command                                          |
  | `/control/command/turn_indicators_cmd` | autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand   | turn indicators command                               |
  | `/control/command/hazard_lights_cmd`   | autoware_auto_vehicle_msgs::msg::HazardLightsCommand     | hazard lights command                                 |
  | `/vehicle/engage`                      | autoware_auto_vehicle_msgs::msg::Engage                  | engage command                                        |
  | `/vehicle/command/actuation_cmd`       | tier4_vehicle_msgs::msg::ActuationCommandStamped         | actuation (accel/brake pedal, steering wheel) command |
  | `/control/command/emergency_cmd`       | tier4_vehicle_msgs::msg::VehicleEmergencyStamped         | emergency command                                     |

- From VWP (... TODO ...)

  | Name                      | Type                              | Description                                                             |
  | ------------------------- | --------------------------------- | ----------------------------------------------------------------------- |
  | `/pacmod/steering_rpt`    | pacmod3_msgs::msg::SystemRptFloat | current steering wheel angle                                            |
  | `/pacmod/wheel_speed_rpt` | pacmod3_msgs::msg::WheelSpeedRpt  | current wheel speed                                                     |
  | `/pacmod/accel_rpt`       | pacmod3_msgs::msg::SystemRptFloat | current accel pedal                                                     |
  | `/pacmod/brake_rpt`       | pacmod3_msgs::msg::SystemRptFloat | current brake pedal                                                     |
  | `/pacmod/shift_rpt`       | pacmod3_msgs::msg::SystemRptInt   | current gear status                                                     |
  | `/pacmod/turn_rpt`        | pacmod3_msgs::msg::SystemRptInt   | current turn indicators status                                          |
  | `/pacmod/global_rpt`      | pacmod3_msgs::msg::GlobalRpt      | current status of other parameters (e.g. override_active, can_time_out) |

### Output topics

- To VWP

  | Name                   | Type                              | Description                                           |
  | ---------------------- | --------------------------------- | ----------------------------------------------------- |
  | `pacmod/accel_cmd`     | pacmod3_msgs::msg::SystemCmdFloat | accel pedal command                                   |
  | `pacmod/brake_cmd`     | pacmod3_msgs::msg::SystemCmdFloat | brake pedal command                                   |
  | `pacmod/steering_cmd`  | pacmod3_msgs::msg::SystemCmdFloat | steering wheel angle and angular velocity command     |
  | `pacmod/shift_cmd`     | pacmod3_msgs::msg::SystemCmdInt   | gear command                                          |
  | `pacmod/turn_cmd`      | pacmod3_msgs::msg::SystemCmdInt   | turn indicators command                               |
  | `pacmod/raw_steer_cmd` | pacmod3_msgs::msg::SteerSystemCmd | raw steering wheel angle and angular velocity command |

- To Autoware

  | Name                                     | Type                                                    | Description                                          |
  | ---------------------------------------- | ------------------------------------------------------- | ---------------------------------------------------- |
  | `/vehicle/status/control_mode`           | autoware_auto_vehicle_msgs::msg::ControlModeReport      | control mode                                         |
  | `/vehicle/status/velocity_status`        | autoware_auto_vehicle_msgs::msg::VelocityReport         | velocity                                             |
  | `/vehicle/status/steering_status`        | autoware_auto_vehicle_msgs::msg::SteeringReport         | steering wheel angle                                 |
  | `/vehicle/status/gear_status`            | autoware_auto_vehicle_msgs::msg::GearReport             | gear status                                          |
  | `/vehicle/status/turn_indicators_status` | autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport   | turn indicators status                               |
  | `/vehicle/status/hazard_lights_status`   | autoware_auto_vehicle_msgs::msg::HazardLightsReport     | hazard lights status                                 |
  | `/vehicle/status/actuation_status`       | autoware_auto_vehicle_msgs::msg::ActuationStatusStamped | actuation (accel/brake pedal, steering wheel) status |
