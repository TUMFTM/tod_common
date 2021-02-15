std_msgs/Header operator_header
int8 operator_control_mode              # Details at include/tod_msgs/controlMode.h
string operator_operator_ip_address
string operator_broker_ip_address
int8 operator_video_rate_mode           # Details at include/tod_msgs/controlMode.h
int8 operator_input_device                 # Details at include/tod_msgs/inputDevice.h

int8 tod_status          		 # Details at include/tod_msgs/connectionStatus.h

std_msgs/Header vehicle_header
string vehicle_vehicle_ip_address
bool vehicle_emergency_stop_released
bool vehicle_long_approved
bool vehicle_lat_approved
int8 vehicle_control_mode               # Details at include/tod_msgs/controlMode.h
string vehicle_nav_status
string vehicle_gps_pos_type
