#!/usr/bin/env python3
# coding: utf-8

"""
#  * @file imu_class.py
#  * @brief This class provides the structure for the
#  * orientation in quaternions, angular velocity as well as
#  * data of ADMA Slim IMU
#  * @authors Alex Sperka
#  * @date 17.03.2021
"""

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3


class IMUClass(object):
    """imu class for holding quaternions, angular velocity and linear acceleartion
    in needed format for standard ros nav message"""

    def __init__(self):
        super(IMUClass, self).__init__()
        self.__quat_x = 0.0
        self.__quat_y = 0.0
        self.__quat_z = 0.0
        self.__quat_w = 1.0

        # Angular Velocity - Body Rates of ADMA
        self.__ang_x = 0.0
        self.__ang_y = 0.0
        self.__ang_z = 0.0

        # Linear Acceleration - Accel Body of Adma
        self.__lin_x = 0.0
        self.__lin_y = 0.0
        self.__lin_z = 0.0

    def set_quaternions(self, quat_x, quat_y, quat_z, quat_w):
        self.__quat_x = quat_x
        self.__quat_y = quat_y
        self.__quat_z = quat_z
        self.__quat_w = quat_w

    def set_angular_velocity(self, ang_vel_x, ang_vel_y, ang_vel_z):
        self.__ang_x = ang_vel_x
        self.__ang_y = ang_vel_y
        self.__ang_z = ang_vel_z

    def set_linear_acceleration(self, lin_acc_x, lin_acc_y, lin_acc_z):
        self.__lin_x = lin_acc_x
        self.__lin_y = lin_acc_y
        self.__lin_z = lin_acc_z

    def __get_quaternions(self):
        return Quaternion(self.__quat_x, self.__quat_y, self.__quat_z, self.__quat_w)

    def __get_angular_velocity(self):
        return Vector3(self.__ang_x, self.__ang_y, self.__ang_z)

    def __get_linear_acceleration(self):
        return Vector3(self.__lin_x, self.__lin_y, self.__lin_z)

    def publish_imu_data(self):
        imu_msg = Imu()
        imu_msg.orientation = self.__get_quaternions()
        # imu_msg.orientation_covariance=
        imu_msg.angular_velocity = self.__get_angular_velocity()
        # imu_msg.angular_velocity_covariance=
        imu_msg.linear_acceleration = self.__get_linear_acceleration()
        # imu_msg.linear_acceleration_covariance=
        return imu_msg
