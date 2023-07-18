#include "ros/ros.h"
#include "cylindrical_scan_msg/CylinderScanService.h"

bool add(cylindrical_scan_msg::CylinderScanService::Request  &req,
         cylindrical_scan_msg::CylinderScanService::Response &res)
{
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cylindrical_scan_service");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("cylindrical_scan_service", add);
  ROS_INFO("Ready to receive cylindrical workpiece parameters.");
  ros::spin();

  return 0;
}