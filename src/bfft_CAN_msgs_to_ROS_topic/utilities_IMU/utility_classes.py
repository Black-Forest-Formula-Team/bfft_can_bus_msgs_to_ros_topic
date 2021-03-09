#!/usr/bin/env python3
# coding: utf-8

import rospy
from std_msgs.msg import String

from bfft_CAN_msgs_to_ROS_topic.msg import Adma_status_errors_msg
from bfft_CAN_msgs_to_ROS_topic.msg import Can_id_msg

class StatusErrorClass():
    """Status, Error and Warning class for holding information until puplished to topic
        Stat_Byte0_GPS_Mode
        Stat_Byte0_Standstill
        Stat_Byte0_Skidding
        Stat_Byte0_External_Vel_Out
        Stat_Byte1_Trig_GPS
        Stat_Byte1_Signal_IN3
        Stat_Byte1_Signal_IN2
        Stat_Byte1_Signal_IN1
        Stat_Byte1_Alignment
        Stat_Byte1_AHRS_INS
        Stat_Byte1_Deadreckoning
        Stat_Byte1_SyncLock
        Stat_Byte2_EVK_activ
        Stat_Byte2_EVK_Estimates
        Stat_Byte2_Tilt
        Stat_Byte2_Pos
        Errors_Byte0_HW
        Errors_Byte0_Nibble1
        Errors_Byte1_Nibble0
        Errors_Byte1_Nibble1
        Warn_Byte2_GPS
    """

    def __init__(self):
        super(StatusErrorClass, self).__init__()
        
        self.__adma_status_errors_msg = Adma_status_errors_msg()
    
    def set_status(self, status_msgs):
        self.__adma_status_errors_msg.Stat_Byte0_GPS_Mode=status_msgs['Stat_Byte0_GPS_Mode']
        self.__adma_status_errors_msg.Stat_Byte0_Standstill=status_msgs['Stat_Byte0_Standstill']
        self.__adma_status_errors_msg.Stat_Byte0_Skidding=status_msgs['Stat_Byte0_Skidding']
        self.__adma_status_errors_msg.Stat_Byte0_External_Vel_Out=status_msgs['Stat_Byte0_External_Vel_Out']
        self.__adma_status_errors_msg.Stat_Byte1_Trig_GPS=status_msgs['Stat_Byte1_Trig_GPS']
        self.__adma_status_errors_msg.Stat_Byte1_Signal_IN3=status_msgs['Stat_Byte1_Signal_IN3']
        self.__adma_status_errors_msg.Stat_Byte1_Signal_IN2=status_msgs['Stat_Byte1_Signal_IN2']
        self.__adma_status_errors_msg.Stat_Byte1_Signal_IN1=status_msgs['Stat_Byte1_Signal_IN1']
        self.__adma_status_errors_msg.Stat_Byte1_Alignment=status_msgs['Stat_Byte1_Alignment']
        self.__adma_status_errors_msg.Stat_Byte1_AHRS_INS=status_msgs['Stat_Byte1_AHRS_INS']
        self.__adma_status_errors_msg.Stat_Byte1_Deadreckoning=status_msgs['Stat_Byte1_Deadreckoning']
        self.__adma_status_errors_msg.Stat_Byte1_SyncLock=status_msgs['Stat_Byte1_SyncLock']
        self.__adma_status_errors_msg.Stat_Byte2_EVK_activ=status_msgs['Stat_Byte2_EVK_activ']
        self.__adma_status_errors_msg.Stat_Byte2_EVK_Estimates=status_msgs['Stat_Byte2_EVK_Estimates']
        self.__adma_status_errors_msg.Stat_Byte2_Tilt=status_msgs['Stat_Byte2_Tilt']
        self.__adma_status_errors_msg.Stat_Byte2_Pos=status_msgs['Stat_Byte2_Pos']
        
        
    def set_errors(self, error_msgs):
        self.__adma_status_errors_msg.Errors_Byte0_HW=error_msgs['Errors_Byte0_HW']
        self.__adma_status_errors_msg.Errors_Byte0_Nibble1=error_msgs['Errors_Byte0_Nibble1']
        self.__adma_status_errors_msg.Errors_Byte1_Nibble0=error_msgs['Errors_Byte1_Nibble0']
        self.__adma_status_errors_msg.Errors_Byte1_Nibble1=error_msgs['Errors_Byte1_Nibble1']
        self.__adma_status_errors_msg.Warn_Byte2_GPS=error_msgs['Warn_Byte2_GPS']

    def publish_status_errors(self):
        ''' Bring errors and status data into msg format to be published on topic'''
        return self.__adma_status_errors_msg

class UnknownCANIdsClass():
    """Unknown CAN Ids for data visualization"""

    def __init__(self):
        super(UnknownCANIdsClass, self).__init__()
        self.__can_msg=Can_id_msg()

    def set_types_values(self, can_id, can_msgs, header):
        '''        #Save values and type of CAN frame (0-3) in class array, currently limited to 4 values
        for counter in range(length): '''
            
        keys=list(can_msgs.keys())
        keys_length=len(keys)
        try:
            self.__can_msg.can_id=can_id
        
            #Save header from CAN frame
            self.__can_msg.seq=header.seq
            self.__can_msg.stamp.secs=header.stamp.secs
            self.__can_msg.stamp.nsecs=header.stamp.nsecs
            self.__can_msg.frame_id=header.frame_id
            
            if keys_length>1:
                self.__can_msg.value_0=can_msgs[keys[0]]
                self.__can_msg.type_col_0=keys[0]
            
            if keys_length>2:
                self.__can_msg.value_1=can_msgs[keys[1]]
                self.__can_msg.type_col_1=keys[1]
            
            if keys_length>3:
                self.__can_msg.value_2=can_msgs[keys[2]]
                self.__can_msg.type_col_2=keys[2]
            
            if keys_length>4:
                self.__can_msg.value_3=can_msgs[keys[3]]
                self.__can_msg.type_col_3=keys[3]          
            
        except rospy.ROSInterruptException:
            pass
        
    def publish_types_values(self):
        '''Returning the custom message format as a template for all incoming unknown CAN IDs 
        to be published (for later data visualization purpose)'''
        
        return self.__can_msg