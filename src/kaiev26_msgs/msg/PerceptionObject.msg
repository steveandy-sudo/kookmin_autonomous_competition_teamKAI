uint8 CLASS_VEHICLE=1
uint8 CLASS_BIKE=2
uint8 CLASS_PEDESTRIAN=3
uint8 CLASS_TRAFFIC_CONE=4
uint8 CLASS_OBSTACLE=5
uint8 CLASS_TRAFFIC_LIGHT=6
uint8 CLASS_SIGN=7

uint32 detection_id
uint32 track_id
uint8 class_id
geometry_msgs/Pose pose
geometry_msgs/Vector3 size
geometry_msgs/Twist twist
float32 confidence
string source
