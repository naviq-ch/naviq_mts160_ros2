# Decoded TPDO2 ("Marker") frame of the MTS160: point-source / marker positions
# in the sensor frame, 0.1 mm resolution. Detected flags are copied from the
# latest TPDO1. Positions read 0 when the corresponding marker is not detected.
std_msgs/Header header

bool left_detected
bool right_detected

float32 left_x_mm        # X axis: lateral (across the 160 mm sensing width), see README for sign
float32 left_y_mm        # Y axis: longitudinal (front/back), see README for sign
float32 right_x_mm
float32 right_y_mm
