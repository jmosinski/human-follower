#!/usr/bin/env python
import rospy
from std_msgs.msg import String

import os
import time
from luma.core.interface.serial import i2c
from luma.oled.device import sh1106
from luma.core.render import canvas
from luma.core.virtual import terminal
from PIL import ImageFont

class Display():
    """
    Control I2C Display

    Ports:
    GND | VCC(3V3) | GPIO3(SCL) | GPIO2(SDA)
    """
    def __init__(self):
	self.font = ImageFont.truetype('Ubuntu-B.ttf', 18)
	serial = i2c(port=1, address=0x3c)
	self.device = sh1106(serial)
	
    def display(self, text):
	with canvas(self.device) as draw:
	    draw.text((0, 20), text, fill='white', font=self.font)

class Raspberry():
    def __init__(self):
        rospy.init_node('raspberry')
        rate = rospy.Rate(10)  # 10hz
        self.modeSub = rospy.Subscriber("/mode", String, self.modeCallback)

        self.mode = 'System Check'
	self.display = Display()

        while not rospy.is_shutdown():
            rate.sleep()
        rospy.spin()


    def modeCallback(self, data):
        self.mode = data.data
	self.display.display(self.mode)


if __name__ == '__main__':
    try:
        pi = Raspberry()
    except rospy.ROSInterruptException:
         pass
