#!/usr/bin/env python3
# coding: utf-8

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
from can_msgs.msg import Frame

import time

class GPSClass():
    """gps class for holding lat and long as well as status, altitude and corresponding time_stamp
    in needed format for standard ros nav message"""

    def __init__(self, lat=0.0, lon=0.0, alt=0.0, time_stamp=1.0):
        super(GPSClass, self).__init__()
        # self = NavSatFix()
        self.__status = NavSatStatus()
        self.__latitude = round(float(lat), 6)
        self.__longitude = round(float(lon), 6)
        self.__altitude = round(float(alt), 6)
        self.__time_secs = round(time.time())
        self.__time_nsecs = round(time.time())
        self.__position_covariance = (0, 0, 0, 0, 0, 0, 0, 0, 0)
        self.__position_covariance_type = 0

    def set_status(self, status):
        self.__status = status
    
    def set_lat_long(self, latitude, longitude):
        self.__latitude = latitude
        self.__longitude = longitude

    def set_altitude(self, altitude):
        self.__altitude = altitude

    def set_time(self, secs, nsecs):
        self.__time_secs = secs
        self.__time_nsecs = nsecs
        
    def __get_status(self):
        return self.__status 
        
    def __get_latitude(self):
        return self.__latitude

    def __get_longitude(self):
        return self.__longitude

    def __get_altitude(self):
        return self.__altitude

    def __get_secs(self):
        return self.__time_secs
    
    def __get_nsecs(self):
        return self.__time_nsecs
    
    def publish_gps_data(self):
        ''' Extract GPS data from CAN frame and publish it as ros message on topic
            https://python.hotexamples.com/site/file?hash=0xdc5e344f55e2dff36b129f42f91321be202db7a5f5a933199cd91423ae3e7b10&fullName=Terminus-master/groovy/GPS.py&project=bnitkin/Terminus
        '''
        gps_msg = NavSatFix()
        Fix = NavSatStatus()

        # Fix.status = self.mode
        # Fix.service = self.numSat
        # gps_msg.status = Fix
        
        # according to ADMA documentation, INS_Lat_Rel and INS_Long_Rel "best position available in sense of precision"
        gps_msg.altitude = self.__get_altitude()
        gps_msg.header.stamp.secs = self.__get_secs()
        gps_msg.header.stamp.nsecs = self.__get_nsecs()                  # TBD
        gps_msg.latitude = self.__get_latitude()
        gps_msg.longitude = self.__get_longitude()
                # covariance_type = 0 unknown
                #                = 1 approximated
                #                = 2 diagonal known
                #                = 3 known
        return gps_msg