# Message for SET_OMNI_ATTITUDE_TARGET
#
# Some complex system requires all feautures that mavlink
# message provide. See issue #402, #418.

std_msgs/Header header

uint8 type_mask
uint8 IGNORE_ROLL_RATE = 1	# body_rate.x
uint8 IGNORE_PITCH_RATE = 2	# body_rate.y
uint8 IGNORE_YAW_RATE = 4	# body_rate.z
uint8 IGNORE_THRUST = 64
uint8 IGNORE_ATTITUDE = 128	# orientation field

geometry_msgs/Quaternion orientation
geometry_msgs/Vector3 body_rate
geometry_msgs/Vector3 thrust
