#!/usr/bin/env python

import rospy
import geometry_msgs.msg
from std_msgs.msg import Float64
from sensor_msgs.msg import Range
import numpy as np
import time
import serial

from message_filters import ApproximateTimeSynchronizer, Subscriber

class MainClass(object):
    def __init__(self):
        self.scale_factor_ = 1.0
        self.offset_ = 0.0
        self.serial_port_ = "/dev/ttyS1"
        self.baud_rate_ = 115200
        self.topic_name_ = "uwb_distance"
    def read_parameters(self):
        # Read in the parameters from ros
        self.scale_factor_ = rospy.get_param('~scale_factor', self.scale_factor_)
        self.offset_ = rospy.get_param('~offset', self.offset_)
        self.serial_port_ = rospy.get_param('~serial_port', self.serial_port_)
        self.baud_rate_ = rospy.get_param('~baud_rate', self.baud_rate_)
        self.topic_name_ = rospy.get_param('~topic_name', self.topic_name_)

def main():
    rospy.init_node('dwm1001_driver_node')
    
    rospy.loginfo('Starting the dwm1001 driver node')
    main_class = MainClass()
    main_class.read_parameters()
    ser = serial.Serial('/dev/serial/by-id/usb-SEGGER_J-Link_000760088177-if00', 115200, timeout=1)
    pub = rospy.Publisher(main_class.topic_name_, Range, queue_size=10)
    rate = rospy.Rate(1000)  # 10hz

    while not rospy.is_shutdown():
        range_msg = Range()
        line = ser.readline()
        range_msg.header.stamp = rospy.get_rostime()
        range_msg.min_range = 0.0
        range_msg.max_range = 200.0
        try:
            val = float(line)
            val = val / 1000.0
            val = val * main_class.scale_factor_ + main_class.offset_
            range_msg.range = val
            pub.publish(range_msg)
            rate.sleep()
        except:
            print('failed to read\n')

if __name__ == '__main__':
    main()
