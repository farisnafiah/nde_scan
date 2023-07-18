#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/OrientationConstraint.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "toolpath_tangential_movement");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Set up MoveIt
  static const std::string PLANNING_GROUP = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
  ROS_INFO_NAMED("tutorial", "Pose reference frame: %s", move_group.getPoseReferenceFrame().c_str());

  // We can get a list of all the groups in the robot:
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(),
          move_group.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));




  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link", "visualization_marker_array");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();




  // Define the toolpath waypoints
  std::vector<geometry_msgs::Pose> waypoints;

  // Define the circle parameters
  double radius = 0.2;  // Radius of the circle
  int num_points = 20;  // Number of points along the circle

  // Define the center of the semi-circle
  double center_x = -0.75;
  double center_y = 0.0;

  // Generate waypoints for the circular trajectory
  waypoints.reserve(num_points);
  for (int i = 0; i < num_points; ++i)
  {
      double angle = M_PI * static_cast<double>(i) / static_cast<double>(num_points);
      double x = center_x + radius * std::cos(angle - M_PI/2);
      double y = center_y + radius * std::sin(angle - M_PI/2);
      double z = 0.5;  // Height above the ground

      geometry_msgs::Pose waypoint;
      waypoint.position.x = x;
      waypoint.position.y = y;
      waypoint.position.z = z;

      // Calculate the orientation to face towards the center of the semi-circle
      double yaw = std::atan2(center_y - y, center_x - x);
      tf2::Quaternion orientation;
      orientation.setRPY(0, M_PI/2, yaw);
      waypoint.orientation.x = orientation.x();
      waypoint.orientation.y = orientation.y();
      waypoint.orientation.z = orientation.z();
      waypoint.orientation.w = orientation.w();

      waypoints.push_back(waypoint);

      visual_tools.publishAxisLabeled(waypoint, "s");
  }

  moveit_msgs::RobotTrajectory trajectory;
  const double eef_step = 0.05;  // Distance between waypoints
  const double jump_threshold = 0.0;  // Disable jump threshold check
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  if (fraction == 1.0)
  {
      ROS_INFO_STREAM("Successfully computed Cartesian path");
  }
  else
  {
      ROS_WARN_STREAM("Unable to compute Cartesian path. Fraction: " << fraction);
  }
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move the robot");
  visual_tools.publishTrajectoryLine(trajectory, joint_model_group->getLinkModel("tool0"), joint_model_group, rvt::LIME_GREEN); // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();

  
  visual_tools.prompt("Press 'next' to move");
  move_group.execute(trajectory);










  ros::shutdown();
  return 0;
}