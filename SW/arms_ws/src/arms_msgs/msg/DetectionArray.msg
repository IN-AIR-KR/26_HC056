std_msgs/Header header
# Wall-clock time when the detection node received the source image. Unlike
# header.stamp, this shares the FC/IMU ROS time base and is used for delayed
# state-estimation correction. A zero value means unavailable.
builtin_interfaces/Time source_receipt_stamp
arms_msgs/BoundingBox[] detections
