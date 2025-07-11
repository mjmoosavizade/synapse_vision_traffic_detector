# synapse_vision_traffic_detector/msg/TrafficSign.msg
#
# This message defines the structure for a single detected traffic sign,
# including its type, confidence, bounding box coordinates, and the model used.

std_msgs/Header header         # Standard ROS 2 header (timestamp and frame_id)
string sign_type               # Classification of the detected sign (e.g., "STOP", "SPEED_LIMIT_60")
float32 confidence             # Confidence score of the detection (0.0 to 1.0)
int32 bbox_x_min               # X-coordinate of the top-left corner of the bounding box (pixels)
int32 bbox_y_min               # Y-coordinate of the top-left corner of the bounding box (pixels)
int32 bbox_width               # Width of the bounding box (pixels)
int32 bbox_height              # Height of the bounding box (pixels)
string model_name              # Name of the ONNX model that performed the detection
