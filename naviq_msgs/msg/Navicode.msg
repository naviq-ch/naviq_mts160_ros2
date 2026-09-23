# Decoded TPDO3 ("Navicode") frame of the MTS160.
std_msgs/Header header

uint16 code              # last decoded Navicode value (1..16 bits)
uint8 counter            # increments each time a valid code is captured (wraps at 255)
bool is_new              # true on the first frame in which counter differs from the previous frame
