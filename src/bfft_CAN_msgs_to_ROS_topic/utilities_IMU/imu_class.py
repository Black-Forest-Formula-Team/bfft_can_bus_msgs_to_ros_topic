#!/usr/bin/env python3
# coding: utf-8

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3

class IMUClass():
    """imu class for holding quaternions, angular velocity and linear acceleartion 
    in needed format for standard ros nav message"""

    def __init__(self):
        super(IMUClass, self).__init__()
        self.__quat_x=0.0
        self.__quat_y=0.0
        self.__quat_z=0.0
        self.__quat_w=1.0
        
        # Angular Velocity - Body Rates of ADMA
        self.__ang_x=0.0
        self.__ang_y=0.0
        self.__ang_z=0.0
        
        # Linear Acceleration - Accel Body of Adma
        self.__lin_x=0.0
        self.__lin_y=0.0
        self.__lin_z=0.0      

    def set_quaternions(self, x,y,z,w):
        self.__quat_x=x
        self.__quat_y=y
        self.__quat_z=z
        self.__quat_w=w
        
    def set_angular_velocity(self, x,y,z):
        self.__ang_x=x
        self.__ang_y=y
        self.__ang_z=z

    def set_linear_acceleration(self, x,y,z):
        self.__lin_x=x
        self.__lin_y=y
        self.__lin_z=z

    def __get_quaternions(self):
        return Quaternion(self.__quat_x, self.__quat_y, self.__quat_z, self.__quat_w)
        
    def __get_angular_velocity(self):
        return Vector3(self.__ang_x, self.__ang_y, self.__ang_z)

    def __get_linear_acceleration(self):
        return Vector3(self.__lin_x, self.__lin_y, self.__lin_z)

    def publish_imu_data(self):
        imu_msg=Imu()
        imu_msg.orientation=self.__get_quaternions()
        #imu_msg.orientation_covariance= 
        imu_msg.angular_velocity=self.__get_angular_velocity()
        #imu_msg.angular_velocity_covariance=
        imu_msg.linear_acceleration=self.__get_linear_acceleration()
        #imu_msg.linear_acceleration_covariance=
        return imu_msg