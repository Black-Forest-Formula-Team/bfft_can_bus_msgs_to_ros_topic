#!/usr/bin/env python3
# coding: utf-8

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
from can_msgs.msg import Frame

import time

class GPSClass():
    """gps class for holding lat and long as well as status, altitude and corresponding time_stamp
    in needed format for standard ros nav message
    - according to ADMA documentation, INS_Lat_Rel and INS_Long_Rel "best position available in sense of precision"
    """
    
    def __init__(self):
        super(GPSClass, self).__init__()
        
        self.__gps_msg = NavSatFix()
        self.__status = NavSatStatus()

    def set_status(self, status):
        self.__status = status
    
    def set_lat_long(self, latitude, longitude):
        self.__gps_msg.latitude = latitude
        self.__gps_msg.longitude = longitude

    def set_altitude(self, altitude):
        self.__gps_msg.altitude = altitude

    def set_time(self, secs, nsecs):
        self.__gps_msg.header.stamp.secs = secs
        self.__gps_msg.header.stamp.nsecs = nsecs
        
    def set_position_covariance(self, pos_covariance):
        '''covariance_type  = 0 unknown
                            = 1 approximated
                            = 2 diagonal known
                            = 3 known'''
        self.__gps_msg.position_covariance=3
        
    def set_position_covariance_type(self, pos_covariance_type):
        self.__gps_msg.position_covariance_type=0
        
    def __get_status(self):
        return self.__status 
        
    def __get_latitude(self):
        return self.__gps_msg.latitude

    def __get_longitude(self):
        return self.__gps_msg.longitude

    def __get_altitude(self):
        return self.__gps_msg.altitude

    def __get_secs(self):
        return self.__gps_msg.header.stamp.secs
    
    def __get_nsecs(self):
        return self.__gps_msg.header.stamp.nsecs
    
    def publish_gps_data(self):
        ''' Extract GPS data from CAN frame and publish it as ros message on topic
            https://python.hotexamples.com/site/file?hash=0xdc5e344f55e2dff36b129f42f91321be202db7a5f5a933199cd91423ae3e7b10&fullName=Terminus-master/groovy/GPS.py&project=bnitkin/Terminus
        '''
        return self.__gps_msg