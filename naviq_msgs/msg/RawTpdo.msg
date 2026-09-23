# Raw CAN frame from the sensor's node ID (TPDO1..3, heartbeat, SDO response),
# published when the driver parameter publish_raw is true.
std_msgs/Header header   # stamp = CAN receive time

uint16 can_id            # 11-bit CAN identifier (COB-ID)
uint8[] data             # payload bytes as received
