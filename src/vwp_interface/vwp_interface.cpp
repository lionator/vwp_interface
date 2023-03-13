#include <vwp_interface/vwp_interface.hpp>
#include <iostream>

VWPInterface::VWPInterface(): Node("vwp_interface"),
  vehicle_info_(vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo())
{
    std::cout << vehicle_info_.wheel_radius_m << std::endl;
}


void VWPInterface::publishCommands() {

}
