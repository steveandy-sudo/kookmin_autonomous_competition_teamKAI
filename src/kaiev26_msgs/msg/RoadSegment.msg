uint8 TYPE_WHSOL=1
uint8 TYPE_WHDOT=2
uint8 TYPE_YESOL=3
uint8 TYPE_YEDOT=4
uint8 TYPE_STOP_LINE=5
uint8 TYPE_CROSSWALK=6

uint32 detection_id
uint32 track_id
uint8 type
geometry_msgs/Point[] points
float32 confidence
string source
