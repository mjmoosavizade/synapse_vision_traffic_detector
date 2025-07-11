# YOLOv8 Traffic Sign Detector for ROS 2

This ROS 2 C++ package provides a real-time traffic sign detection node. It uses a YOLOv8 object detection model, executed with OpenCV's DNN module, to identify traffic signs from a camera feed and publish the results as custom ROS 2 messages.

This project is designed to be a clear and robust starting point for integrating modern deep learning models into ROS 2 robotics projects.

## Key Features

- **Real-Time Detection**: Subscribes to a ROS 2 image topic and performs inference on each frame.
- **YOLOv8 & ONNX**: Utilizes a pre-trained YOLOv8 model in the highly portable ONNX format.
- **OpenCV DNN**: Leverages OpenCV's Deep Neural Network (DNN) module for efficient model inference.
- **Custom ROS Messages**: Publishes detection results (class name, confidence, bounding box) using a custom `TrafficSign.msg` message.
- **Configurable**: All key parameters (model path, confidence thresholds, etc.) are exposed in a `params.yaml` file for easy tuning.
- **Launch File Included**: Comes with a ROS 2 launch file for easy startup.

---

## System Dependencies

Before building, ensure you have the following installed on your system:

- **Ubuntu 22.04 (Jammy Jellyfish)**
- **ROS 2 Humble Hawksbill**:
  - `ros-humble-desktop`
  - `ros-dev-tools`
- **OpenCV**: The version included with ROS 2 Humble is sufficient.

---

## Project Structure


.
├── CMakeLists.txt
├── config
│   └── params.yaml        # Node parameters
├── launch
│   └── detector.launch.py # Launch file
├── models
│   └── best.onnx          # The ONNX model
├── msg
│   └── TrafficSign.msg    # Custom message definition
├── package.xml
├── README.md              # This file
└── src
└── traffic_sign_detector_node.cpp # The C++ source code
---

## Setup and Build Instructions

Follow these steps to set up a ROS 2 workspace and build the package.

1.  **Create a ROS 2 Workspace:**
    ```bash
    mkdir -p ~/ros2_ws/src
    ```

2.  **Clone or Move the Package:**
    Move the `synapse_vision_traffic_detector` package folder into the `~/ros2_ws/src` directory.

3.  **Build the Workspace:**
    Navigate to the root of your workspace and run `colcon build`.
    ```bash
    # Open a new terminal
    source /opt/ros/humble/setup.bash
    cd ~/ros2_ws
    colcon build
    ```

---

## How to Run the Node

1.  **Source the Workspace:**
    In every new terminal you use, you must first source your workspace's setup file.
    ```bash
    cd ~/ros2_ws
    source install/setup.bash
    ```

2.  **Launch the Detector:**
    Use the provided launch file to start the node.
    ```bash
    ros2 launch synapse_vision_traffic_detector detector.launch.py
    ```
    The node will now be running and waiting for images on the `/camera/image_raw` topic.

3.  **Provide an Image Stream:**
    You need a camera node or a ROS 2 bag file publishing images. For testing, you can use the `image_publisher` from the `image_tools` package:
    ```bash
    # In a new, sourced terminal
    ros2 run image_tools image_publisher --ros-args -p publish_rate:=10.0 /path/to/your/test_image.jpg
    ```

4.  **Verify the Output:**
    To see the detection results, echo the output topic in another sourced terminal:
    ```bash
    ros2 topic echo /traffic_sign_detection/output
    ```

---

## Known Issues

### ONNX Model Incompatibility

The provided `best.onnx` model was created with a newer version of PyTorch and contains operations (specifically in its attention mechanism) that are **not compatible with the version of OpenCV (4.5.4) included with ROS 2 Humble**.

This results in a runtime error when OpenCV's DNN module attempts to load the model:
`error: (-215:Assertion failed) splits > 0 && inpShape[axis_rw] % splits == 0`

**Solution:** To resolve this, the C++ node must be modified to use a different inference engine, such as the official **ONNX Runtime**, which can correctly parse and execute this model.
