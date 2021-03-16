# sequence ID: consecutively increasing ID 
uint32 seq

# Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp

# Frame this data is associated with
string frame_id

# CAN Bus ID of Sensor
uint8 can_id

# Up to 4 Types of the values as strings needed for later matching in data visualization
string type_col_0
string type_col_1
string type_col_2
string type_col_3

# Up to four values of unknown or new CAN sensor
float32 value_0
float32 value_1
float32 value_2
float32 value_3