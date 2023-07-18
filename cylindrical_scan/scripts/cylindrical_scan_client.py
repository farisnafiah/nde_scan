#!/usr/bin/env python



from __future__ import print_function

import sys
import rospy
import geometry_msgs.msg
from cylindrical_scan_msg.srv import CylinderScanService, CylinderScanServiceRequest

from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest

def cylindrical_scan_client():
    rospy.wait_for_service('cylindrical_scan')
    
    try:
        cylindrical_scan = rospy.ServiceProxy('cylindrical_scan', CylinderScanService)
        request = CylinderScanServiceRequest()
        # Set your desired pose here
        request.params.base_x = 0.95
        request.params.base_y = 0.2
        request.params.base_z = 0.5
        request.params.base_rot_x = 0.0
        request.params.base_rot_y = 0.0
        request.params.base_rot_z = 0.0
        request.params.base_rot_w = 1.0
        request.params.radius = 0.2
        request.params.height = 0.5
        request.params.num_points = 25

        response = cylindrical_scan(request)
        if response:
            print("Trajectory successfully planned!")
        else:
            print("odo")

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    cylindrical_scan_client()