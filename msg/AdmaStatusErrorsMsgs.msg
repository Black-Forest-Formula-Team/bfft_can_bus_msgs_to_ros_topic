# sequence ID: consecutively increasing ID 
uint32 seq

# Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is calledbool secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is calledbool nsecs')
# time-handling sugar is provided by the client library
time stamp

# Message fields are not in lowercase as usually since they refer to the Adma Slim Message format exactly for easier identification and handling
bool Stat_Byte0_GPS_Mode
bool Stat_Byte0_Standstill
bool Stat_Byte0_Skidding
bool Stat_Byte0_External_Vel_Out
bool Stat_Byte1_Trig_GPS
bool Stat_Byte1_Signal_IN3
bool Stat_Byte1_Signal_IN2
bool Stat_Byte1_Signal_IN1
bool Stat_Byte1_Alignment
bool Stat_Byte1_AHRS_INS
bool Stat_Byte1_Deadreckoning
bool Stat_Byte1_SyncLock
bool Stat_Byte2_EVK_activ
bool Stat_Byte2_EVK_Estimates
bool Stat_Byte2_Tilt
bool Stat_Byte2_Pos

bool Errors_Byte0_HW
bool Errors_Byte0_Nibble1
bool Errors_Byte1_Nibble0
bool Errors_Byte1_Nibble1
bool Warn_Byte2_GPS