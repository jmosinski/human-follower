#!/usr/bin/env python
import rospy
from std_msgs.msg import String

from luma.core.virtual import terminal

class Display():
    """
    Control I2C Display

    Ports:
    GND | VCC(3V3) | GPIO3(SCL) | GPIO2(SDA)
    """
    def __init__(self):
        pass
    def print(self, text):
        term.clean()
        term.println(text)

class PI():
    def __init__(self):
        rospy.init_node('raspberry')
        rate = rospy.Rate(10)  # 10hz
        self.modeSub = rospy.Subscriber("/mode", string, self.modeCallback)

        self.mode = 'System Check'

        while not rospy.is_shutdown():

            rata.sleep()
        rospy.spin()


    def modeCallback(self, data):
        self.mode = data.data

    def display(self):
        pass


if __name__ == '__main__':
    # try:
    #     robot = HumanFollower()
    # except rospy.ROSInterruptException:
    #     pass

    display = Display()
    display.print("test")
