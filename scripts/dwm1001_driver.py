#!/usr/bin/env python

import rospy
import geometry_msgs.msg
from std_msgs.msg import Float64
from sensor_msgs.msg import Range
import numpy as np
import time
import serial
import json

from message_filters import ApproximateTimeSynchronizer, Subscriber

class MainClass(object):
    def __init__(self):
        self.scale_factor_ = 1.0
        self.offset_ = 0.0
        self.serial_port_ = "/dev/ttyS1"
        self.baud_rate_ = 460800
        self.topic_name_ = "uwb_distance"
       # self.publisher_ = rospy.Publisher();
    def read_parameters(self):
        # Read in the parameters from ros
        scale_factor = rospy.get_param("~scale_factor")
        offset = rospy.get_param("~offset")
        self.serial_port_ = rospy.get_param('~serial_port', self.serial_port_)
        self.baud_rate_ = rospy.get_param('~baud_rate', self.baud_rate_)
        topic_name = "uwb_distance"
        topic_name = rospy.get_param('~topic_name', topic_name)
        print(scale_factor)
        self.publisher_ = rospy.Publisher(topic_name, Range, queue_size=10);

def main():
    rospy.init_node('dwm1001_driver_node')
    
    rospy.loginfo('Starting the dwm1001 driver node')
    main_class = MainClass()
    main_class.read_parameters()
    ser = serial.Serial(main_class.serial_port_, main_class.baud_rate_, timeout=1)
    rate = rospy.Rate(400)


    range_msg = Range()
    range_msg.min_range = 0.0
    range_msg.max_range = 200.0
    while not rospy.is_shutdown():
        line = ser.readline()
        data = {}
        try:
          data = json.loads(line)
        except ValueError as e:
          continue
        range_msg.header.stamp = rospy.get_rostime()
        if 'raz' not in data : 
          print("no data");
          continue
        range_msg.range = float(data['raz'][0])
        main_class.publisher_.publish(range_msg)
        rate.sleep()

if __name__ == '__main__':
    main()
