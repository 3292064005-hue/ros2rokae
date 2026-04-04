builtin_interfaces/Time send_time
uint64 seq
int32 rt_mode
uint8 command_kind
float64[6] values
bool finished
string dispatch_mode
uint8 transport_hint

uint8 COMMAND_JOINT_POSITION=0
uint8 COMMAND_CARTESIAN_POSITION=1
uint8 COMMAND_TORQUE=2
uint8 COMMAND_STOP=3

uint8 TRANSPORT_HINT_UNKNOWN=0
uint8 TRANSPORT_HINT_SHM=1
uint8 TRANSPORT_HINT_TOPIC=2
uint8 TRANSPORT_HINT_LEGACY=3
