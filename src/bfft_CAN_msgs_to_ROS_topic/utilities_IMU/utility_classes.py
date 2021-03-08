#!/usr/bin/env python3
# coding: utf-8

import rospy
from std_msgs.msg import String

from bfft_CAN_msgs_to_ROS_topic.msg import Can_id_msg

class StatusErrorClass():
    """Status, Error and Warning class for holding information until puplished to topic"""

    def __init__(self):
        super(StatusErrorClass, self).__init__()
        self.__status_gps_mode=0
        self.__status_standstill=0
        self.__status_skidding=0
        self.__status_trig_gps=0
        self.__status_signal_IN3=0
        self.__status_signal_IN2=0
        self.__status_signal_IN1=0
        self.__status_alignment=0
        self.__status_ahrs_ins=0
        self.__status_deadreckoning=0
        self.__status_synclock=0
        self.__status_evk_activ=0
        self.__status_evk_estimates=0
        self.__status_tilt=0
        self.__status_pos=0
        self.__error_HW=0
        self.__error_byte0_nibble1=0
        self.__error_byte0_nibble0=0
        self.__error_byte1_nibble1=0
        self.__warning_byte2_gps=0

class UnknownCANIdsClass():
    """Unknown CAN Ids for data visualization"""

    def __init__(self):
        super(UnknownCANIdsClass, self).__init__()
        self.__can_id=0
        self.__seq=0
        self.__secs=0
        self.__nsecs=0
        self.__frame_id=0
        self.__type_col=['','','','']
        self.__values=[0.0,0.0,0.0,0.0]

    def set_types_values(self, can_id, can_msgs, header):
        self.__type_col=['','','','']
        self.__values=[0.0,0.0,0.0,0.0]
        
        columns_names=can_msgs.columns.values
        length = len(columns_names)-1
        values=[None]*length
            
        self.__can_id=can_id
        
        #Save header from CAN file
        self.__seq=header.seq
        self.__secs=header.stamp.secs
        self.__nsecs=header.stamp.nsecs
        self.__frame_id=header.frame_id
        
        counter = 0
        
        #Save values and type of CAN frame (0-3) in class array, currently limited to 4 values
        for counter in range(length): 
            if can_msgs.loc[0, str(columns_names[0])] is not None:
                self.__type_col[counter]=columns_names[counter]
                self.__values[counter]=can_msgs.loc[0, str(columns_names[0])]
        
    def publish_types_values(self):
        '''Returning the custom message format as a template for all incoming unknown CAN IDs 
        to be published (for later data visualization purpose)'''
        can_msg=Can_id_msg()
        
        can_msg.can_id=self.__can_id
        
        #Take header from CAN file
        can_msg.seq=self.__seq
        can_msg.stamp.secs=self.__secs
        can_msg.stamp.nsecs=self.__nsecs
        can_msg.frame_id=self.__frame_id
        
        can_msg.value_0=self.__values[0]
        can_msg.value_1=self.__values[1]
        can_msg.value_2=self.__values[2]
        can_msg.value_3=self.__values[3]

        can_msg.type_col_0=self.__type_col[0]
        can_msg.type_col_1=self.__type_col[1]
        can_msg.type_col_2=self.__type_col[2]
        can_msg.type_col_3=self.__type_col[3]
        
        return can_msg