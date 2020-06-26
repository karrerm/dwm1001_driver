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
        self.scale_factor_ = {}
        self.offset_ = {}
        self.serial_port_ = "/dev/ttyS1"
        self.baud_rate_ = 115200
        self.this_id_ = 0
        #self.topic_name_ = "uwb_distance"
        self.uid_mapping_ = {}
        self.publishers_ = {}
    def read_parameters(self):
        # Read in the parameters from ros
        scale_factor = rospy.get_param("~scale_factor")
        offset = rospy.get_param("~offset")
        self.serial_port_ = rospy.get_param('~serial_port', self.serial_port_)
        self.baud_rate_ = rospy.get_param('~baud_rate', self.baud_rate_)
        topic_name = "uwb_distance"
        topic_name = rospy.get_param('~topic_name', topic_name)
        self.this_id_ = rospy.get_param('~this_id', self.this_id_)
        uids = rospy.get_param("~uids")
        uids_mapping = rospy.get_param("~uids_mapping")
        if (len(uids) != len(uids_mapping)) : print('Mappings are not correct')
        print(scale_factor)
        for i in range(len(uids)) :
            self.scale_factor_[uids[i]] = scale_factor[i]
            self.offset_[uids[i]] = offset[i]
            self.uid_mapping_[uids[i]] = uids_mapping[i]
            tmp_topic_name = '/%s_%d_%d' %(topic_name, self.this_id_, uids_mapping[i])
            self.publishers_[uids[i]] = rospy.Publisher(tmp_topic_name, Range, queue_size=10)
            print(tmp_topic_name)

def main():
    rospy.init_node('dwm1001_driver_node')
    
    rospy.loginfo('Starting the dwm1001 driver node')
    main_class = MainClass()
    main_class.read_parameters()
    ser = serial.Serial(main_class.serial_port_, 115200, timeout=1)
    rate = rospy.Rate(200) # 1000Hz

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
        if 'nrng' not in data : 
          continue
        if 'rng' not in data['nrng'] :
          continue
        if 'uid' not in data['nrng'] :
          continue
        for i in range(len(data['nrng']['uid'])) :
          range_msg.range = float(data['nrng']['rng'][i])
          main_class.publishers_[int(data['nrng']['uid'][i])].publish(range_msg)
        rate.sleep()

if __name__ == '__main__':
    main()
