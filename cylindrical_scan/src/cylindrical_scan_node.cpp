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
  moveit_visual_tools::MoveItVisualTools visual_tools("world");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();


  // Define the toolpath waypoints
  std::vector<geometry_msgs::Pose> waypoints;
  
  geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();

  waypoints.push_back(current_pose.pose);
  
  double x = current_pose.pose.position.x;
  double y = current_pose.pose.position.y;
  double z = current_pose.pose.position.z;
  double qx = current_pose.pose.orientation.x;
  double qy = current_pose.pose.orientation.y;
  double qz = current_pose.pose.orientation.z;
  double qw = current_pose.pose.orientation.w;
  // print out the current Cartesian position (1.184300, 0.256141, 0.011600, -0.000000, -0.707107, -0.707107, 0.000000) 
  //                                         (-0.000288, 0.256141, 1.427300, -0.707107, -0.000144, -0.000144, 0.707107)   
  ROS_INFO_NAMED("current_position", "Current position: (%f, %f, %f, %f, %f, %f, %f)", x, y, z, qx, qy, qz, qw);

  // Add the end waypoint
  geometry_msgs::Pose end_pose;
  end_pose.position.x = 0.02;
  end_pose.position.y = 0.35;
  end_pose.position.z = 1.35;
  end_pose.orientation.x = qx;
  end_pose.orientation.y = qy;
  end_pose.orientation.z = qz;
  end_pose.orientation.w = qw;
  waypoints.push_back(end_pose);

  // Add the end waypoint
  end_pose.position.x = 0.075;
  end_pose.position.y = 0.5;
  end_pose.position.z = 1.2;
  waypoints.push_back(end_pose);

  // Add the end waypoint
  end_pose.position.x = 0.1;
  end_pose.position.y = 0.75;
  end_pose.position.z = 1;
  waypoints.push_back(end_pose);

  // Add the end waypoint
  end_pose.position.x = 0.45;
  end_pose.position.y = 0.6;
  end_pose.position.z = 0.85;
  waypoints.push_back(end_pose);

  // Add the end waypoint
  end_pose.position.x = 0.1;
  end_pose.position.y = y;
  end_pose.position.z = 1.05;
  waypoints.push_back(end_pose);

  x = end_pose.position.x;
  y = end_pose.position.y;
  z = end_pose.position.z;
  qx = end_pose.orientation.x;
  qy = end_pose.orientation.y;
  qz = end_pose.orientation.z;
  qw = end_pose.orientation.w;
  
  ROS_INFO_NAMED("current_position", "End position: (%f, %f, %f, %f, %f, %f, %f)", x, y, z, qx, qy, qz, qw);


// Set the waypoints for the Cartesian path
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

  // moveit::planning_interface::MoveGroupInterface::Plan plan;
  // plan.trajectory_ = trajectory;
  // move_group.plan(plan);

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move the robot");
  visual_tools.publishAxisLabeled(end_pose, "end goal"); 
  visual_tools.publishTrajectoryLine(trajectory, joint_model_group->getLinkModel("tool0"), joint_model_group, rvt::LIME_GREEN); // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  

  
  
  
  visual_tools.prompt("Press 'next' to move");
  move_group.execute(trajectory);

  ros::shutdown();
  return 0;
}