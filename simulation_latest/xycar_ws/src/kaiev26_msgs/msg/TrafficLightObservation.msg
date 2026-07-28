uint8 STATE_UNKNOWN=0
uint8 STATE_RED=1
uint8 STATE_YELLOW=2
uint8 STATE_GREEN=3
uint8 STATE_ARROW=4

uint32 detection_id
uint32 track_id
uint8 state
geometry_msgs/Point position
float32 red_score
float32 yellow_score
float32 green_score
float32 arrow_score
float32 confidence
string source
