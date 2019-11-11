#!/usr/bin/env python

# roscore
# rosrun rosserial_python serial_node.py _baud:=9600 _port:=/dev/ttyUSB0
# In case of any problem do following: sudo chmod 666 /dev/ttyUSB0
# rosrun <package_name> serial_communication.py

import rospy
from geometry_msgs.msg import Twist

if __name__ == "__main__":

    # Initialize node
    rospy.init_node('serial_communication', anonymous=False)
    pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(2)
    # Info
    rospy.loginfo("READY!")

    # Body
    linear_cnt = 0
    angular_cnt = 0
    while not rospy.is_shutdown():
        msg = Twist()
        msg.linear.x = linear_cnt % 100
        msg.angular.z = angular_cnt % 45
        pub_cmd_vel.publish(msg)

        linear_cnt += 1
        angular_cnt += 1
        rate.sleep()

