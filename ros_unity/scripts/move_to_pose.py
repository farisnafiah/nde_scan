#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

seq = 0

def talker():
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    global seq
    seq += 1
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(0.01)
    while not rospy.is_shutdown():
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose.position.x = 27.0
        goal.pose.position.y = 17.1760223325
        goal.pose.position.z = 0.0

        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = -0.856234963969
        goal.pose.orientation.w = 0.516586572102
        pub.publish(goal)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass