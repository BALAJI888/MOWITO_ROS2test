# ROS2 Image Conversion Package

A ROS2 package that provides real-time image conversion between color and grayscale modes with dynamic mode switching via service calls.

## 📋 Overview

This package implements a ROS2 node that subscribes to a camera image topic, converts the images based on the current mode (color or grayscale), and publishes the processed images. It includes both the main conversion node and a dummy USB camera node for testing without hardware dependencies.

## ✨ Features

- **Real-time Image Processing**: Continuously processes camera images at 30Hz
- **Dynamic Mode Switching**: Switch between color and grayscale modes via ROS2 service calls
- **Flexible Input Sources**: Works with real USB cameras or video files
- **Configurable Topics**: Customizable input and output topic names
- **Production Ready**: Includes proper launch files and parameter configuration

## 🏗️ System Architecture

```
[USB Camera/Dummy Cam] → [/image_raw] → [Image Conversion Node] → [/converted_image]
                              ↓
                      [Mode Service (/set_mode)]
```

## 📁 Package Structure

```
image_conversion_pkg/
├── src/
│   ├── image_conversion_node.cpp    # Main image conversion node
│   └── dummy_usb_cam_node.cpp       # Mock camera node for testing
├── launch/
│   └── image_conversion.launch.py   # Launch file for both nodes
├── CMakeLists.txt                   # Build configuration
├── package.xml                      # Package dependencies
└── README.md                        # This file
```

## 🚀 Quick Start

### Prerequisites

- ROS2 Humble/Humble Hawksbill or newer
- OpenCV 4.2+
- USB camera (optional, dummy camera included)

### Installation

1. **Create a ROS2 workspace** (if you don't have one):
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. **Clone or create the package**:
```bash
git clone <repository-url> image_conversion_pkg
# OR copy the package to your src directory
```

3. **Build the package**:
```bash
cd ~/ros2_ws
colcon build --packages-select image_conversion_pkg
```

4. **Source the workspace**:
```bash
source install/setup.bash
```

## 🎯 Usage

### Method 1: Using Launch File (Recommended)

**Launch with dummy camera (default)**:
```bash
ros2 launch image_conversion_pkg image_conversion.launch.py
```

**Launch with real USB camera**:
```bash
ros2 launch image_conversion_pkg image_conversion.launch.py use_dummy_cam:=false
```

**Launch with specific video file**:
```bash
ros2 launch image_conversion_pkg image_conversion.launch.py video_path:=/path/to/your/video.mp4
```

### Method 2: Running Nodes Individually

**Start dummy camera node**:
```bash
ros2 run image_conversion_pkg dummy_usb_cam_node
```

**Start image conversion node**:
```bash
ros2 run image_conversion_pkg image_conversion_node
```

## 🔧 Service Interface

### Changing Conversion Mode

**Switch to grayscale mode**:
```bash
ros2 service call /set_mode std_srvs/srv/SetBool "{data: true}"
```

**Switch to color mode**:
```bash
ros2 service call /set_mode std_srvs/srv/SetBool "{data: false}"
```

### Service Response
- `success: true` - Mode change successful
- `message: "Mode set to Grayscale"` or `"Mode set to Color"`

## ⚙️ Configuration Parameters

### Image Conversion Node
- `input_topic` (default: `/image_raw`) - Input image topic
- `output_topic` (default: `/converted_image`) - Output image topic

### Dummy USB Camera Node
- `video_path` (default: "") - Path to video file (empty for webcam)
- `output_topic` (default: `/image_raw`) - Camera output topic
- `frame_rate` (default: 30) - Publishing frame rate

### Launch File Parameters
- `use_dummy_cam` (default: true) - Use dummy camera instead of real USB cam
- `video_path` (default: "") - Video file path for dummy camera
- `input_topic` (default: `/image_raw`) - Input topic name
- `output_topic` (default: `/converted_image`) - Output topic name

## 🔍 Monitoring

**View available topics**:
```bash
ros2 topic list
```

**View camera images**:
```bash
ros2 run rqt_image_view rqt_image_view
```

**Monitor node status**:
```bash
ros2 node list
ros2 node info /image_conversion
```

**Check service availability**:
```bash
ros2 service list
ros2 service type /set_mode
```

## 🐛 Troubleshooting

### Common Issues

1. **"No image received" error**
   - Check if camera is properly connected
   - Verify topic names match between nodes
   - Use `ros2 topic echo /image_raw --once` to check if images are published

2. **Dummy camera not working**
   - Ensure OpenCV is installed: `apt install python3-opencv`
   - Check video file path is correct and accessible
   - Verify webcam permissions: `ls -l /dev/video*`

3. **Build errors**
   - Ensure all dependencies are installed
   - Clean build: `colcon build --packages-select image_conversion_pkg --cmake-clean-first`

4. **Service call fails**
   - Check if image conversion node is running
   - Verify service name: `ros2 service list`
   - Check node logs for errors

### Debug Mode

Enable verbose logging:
```bash
ros2 launch image_conversion_pkg image_conversion.launch.py --log-level debug
```

## 🧪 Testing

### Basic Functionality Test

1. Launch the system:
```bash
ros2 launch image_conversion_pkg image_conversion.launch.py
```

2. Check if nodes are running:
```bash
ros2 node list
# Should show: /dummy_usb_cam, /image_conversion
```

3. Verify topics:
```bash
ros2 topic list
# Should include: /image_raw, /converted_image, /set_mode
```

4. Test mode switching:
```bash
ros2 service call /set_mode std_srvs/srv/SetBool "{data: true}"
ros2 service call /set_mode std_srvs/srv/SetBool "{data: false}"
```

## 🔄 API Reference

### Topics

- **Subscribed**: `/image_raw` (sensor_msgs/Image)
- **Published**: `/converted_image` (sensor_msgs/Image)

### Services

- **Service**: `/set_mode` (std_srvs/SetBool)
  - Request: `bool data` (true=grayscale, false=color)
  - Response: `bool success`, `string message`

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch: `git checkout -b feature/new-feature`
3. Commit your changes: `git commit -am 'Add new feature'`
4. Push to the branch: `git push origin feature/new-feature`
5. Submit a pull request

## 📄 License

This project is licensed under the Apache 2.0 License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- ROS2 community for excellent documentation
- OpenCV team for computer vision libraries
- Contributors and testers

---
