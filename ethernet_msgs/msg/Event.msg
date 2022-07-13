# A network event.
std_msgs/Header header

uint8 type    # enum defined in EventType.msg
int32 value   # depends on type, see EventType.msg
