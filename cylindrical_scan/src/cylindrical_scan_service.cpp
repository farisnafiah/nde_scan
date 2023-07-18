#include "ros/ros.h"
#include "cylindrical_scan_msg/CylinderScanService.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/OrientationConstraint.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

moveit_msgs::RobotTrajectory plan_trajectory(moveit::planning_interface::MoveGroupInterface move_group, const geometry_msgs::Pose& destination_pose,
const std::vector<double>& start_joint_angles)
{
  
  moveit::core::RobotStatePtr custom_state(new moveit::core::RobotState(move_group.getRobotModel()));
  custom_state->setJointGroupPositions(move_group.getName(), start_joint_angles);
  move_group.setStartState(*custom_state);
  move_group.setPoseTarget(destination_pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (!success)
  {
      ROS_ERROR("Trajectory planning failed!");
      throw std::runtime_error("Trajectory planning failed");
  }

  return plan.trajectory_;
}





bool add(cylindrical_scan_msg::CylinderScanService::Request  &req,
         cylindrical_scan_msg::CylinderScanService::Response &res)
{
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cylindrical_scan_service");
  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService("cylindrical_scan_service", add);
  ROS_INFO("Ready to receive cylindrical workpiece parameters.");
  ros::spin();

  return 0;
}