#!/usr/bin/env python

from __future__ import print_function

import rospy

import sys
import copy
import math
import moveit_commander

import moveit_msgs.msg
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
import geometry_msgs.msg
from geometry_msgs.msg import Quaternion, Pose
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from tf.transformations import quaternion_from_euler

from cylindrical_scan_msg.srv import CylinderScanService, CylinderScanServiceRequest, CylinderScanServiceResponse

joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
"""
    Given the start angles of the robot, plan a trajectory that ends at the destination pose.
"""
def plan_trajectory(move_group, destination_pose, start_joint_angles): 

    current_joint_state = JointState()
    current_joint_state.name = joint_names
    current_joint_state.position = start_joint_angles

    moveit_robot_state = RobotState()
    moveit_robot_state.joint_state = current_joint_state
    move_group.set_start_state(moveit_robot_state)

    move_group.set_pose_target(destination_pose)
    plan = move_group.plan()

    if not plan:
        exception_str = """
            Trajectory could not be planned for a destination of {} with starting joint angles {}.
            Please make sure target and destination are reachable by the robot.
        """.format(destination_pose, destination_pose)
        raise Exception(exception_str)

    return plan


"""
    Creates a pick and place plan using the three states below.
    
    1. Pre scan - enganging position
    2. Scan - Scanning trajectory
    3. Post scan - disengage position

    https://github.com/ros-planning/moveit/blob/master/moveit_commander/src/moveit_commander/move_group.py
"""
def plan_scan(req):
    response = CylinderScanServiceResponse()

    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    pick_pose = geometry_msgs.msg.Pose()
    pick_pose.position.x = req.params.base_x
    pick_pose.position.y = req.params.base_y
    pick_pose.position.z = req.params.base_z
    [pick_pose.orientation.w, pick_pose.orientation.x, pick_pose.orientation.y, pick_pose.orientation.z] = quaternion_from_euler(math.pi, 0.0, 0.0)

    print("Requesting %s+%s"%(pick_pose.position.x, pick_pose.orientation.x))
    
    # Pre scan - position TCP slightly away from the scan start point
    pre_grasp_pose = plan_trajectory(move_group, pick_pose, move_group.get_current_joint_values())

    # If trajectory planning worked for all pick and place stages, add plan to response
    response.trajectories.append(pre_grasp_pose)

    move_group.clear_pose_targets()

    return response


def cylindrical_scan_server():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('cylindrical_scan_server')

    s = rospy.Service('cylindrical_scan', CylinderScanService, plan_scan)
    print("Ready to plan")
    rospy.spin()


if __name__ == "__main__":
    cylindrical_scan_server()