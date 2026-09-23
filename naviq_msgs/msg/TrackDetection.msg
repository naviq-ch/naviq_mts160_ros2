# Decoded TPDO1 ("Sense") frame of the MTS160.
std_msgs/Header header   # stamp = CAN receive time, frame_id = sensor link

Track left
Track right

uint8 STRENGTH_NONE=0
uint8 STRENGTH_WEAK=1
uint8 STRENGTH_MEDIUM=2
uint8 STRENGTH_STRONG=3
uint8 strength           # 0 none (no tape), 1 weak, 2 medium, 3 strong

bool tape_detected       # strength != 0
bool single_track        # left == right (position and angle)
bool left_marker         # marker detected on the left side
bool right_marker        # marker detected on the right side
bool intersection        # broad crossing under the sensor (angles forced to 0, markers cleared)
bool fork                # advisory: left/right angle difference +10..45 deg
bool merge               # advisory: left/right angle difference -10..-45 deg
