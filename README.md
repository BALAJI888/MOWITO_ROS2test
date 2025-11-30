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
# Behavior Tree Robot Task

A C++ implementation of a behavior tree for a robot task where the robot navigates to a room, opens a fridge, picks an apple, and exits the room.

## Overview

This project demonstrates the use of BehaviorTree.CPP to create a modular behavior tree for robotic task execution. The robot follows a predefined sequence of actions to complete its mission, with conditional checks and fallback behaviors.

## Task Flow

```
Start → Move to Room Door → Check Door → [If Closed: Open Door] → Enter Room → 
Move to Fridge Door → Check Door → [If Closed: Open Door] → Find Apple → 
Pick Apple → Close Fridge Door → Move to Room Door → Exit Room → End
```

## Features

- Modular behavior tree design using BehaviorTree.CPP
- Clean separation of actions and conditions
- Fallback behaviors for door handling
- Comprehensive logging of execution flow
- Easy to extend with new behaviors

## Prerequisites

- Ubuntu 18.04 or higher (tested on Ubuntu 20.04/22.04)
- CMake (version 3.5 or higher)
- C++14 compatible compiler
- BehaviorTree.CPP library

## Installation

### Install BehaviorTree.CPP

```bash
sudo apt-get update
sudo apt-get install libbehaviortree-cpp-dev
```

### Clone and Build

```bash
# Clone the repository
git clone https://github.com/your-username/behavior-tree-robot-task.git
cd behavior-tree-robot-task

# Create build directory and compile
mkdir build
cd build
cmake ..
make
```

## Project Structure

```
behavior_tree_task/
├── CMakeLists.txt          # Build configuration
├── src/
│   ├── main.cpp           # Main application and tree definition
│   ├── nodes.h            # Node class declarations
│   └── nodes.cpp          # Node implementations
└── README.md
```

## Usage

Run the behavior tree executable:

```bash
./build/bt_task
```

### Expected Output

```
=== Starting Behavior Tree Execution ===
[MoveTowardsDoorOfRoom] Moving towards the door of the room...
[IsDoorClosed] Checking if room door is closed: YES
[OpenDoor] Opening the door...
[EnterRoom] Entering the room...
[MoveTowardsDoorOfFridge] Moving towards the door of the fridge...
[IsDoorClosed] Checking if fridge door is closed: YES
[OpenDoor] Opening the door...
[FindApple] Finding the apple in the fridge...
[PickApple] Picking the apple...
[CloseDoorOfFridge] Closing the door of the fridge...
[MoveTowardsDoorOfRoom] Moving towards the door of the room...
[ExitRoom] Exiting the room...
=== Behavior Tree Execution Completed ===
```

## Behavior Tree Nodes

### Action Nodes
- `MoveTowardsDoorOfRoom` - Navigate to room entrance
- `OpenDoor` - Open closed doors
- `EnterRoom` - Enter through doorway
- `MoveTowardsDoorOfFridge` - Navigate to fridge
- `FindApple` - Locate apple in fridge
- `PickApple` - Pick up the apple
- `CloseDoorOfFridge` - Close fridge door
- `ExitRoom` - Leave the room

### Condition Nodes
- `IsDoorClosed` - Check door status (room or fridge)

## Customization

### Adding New Nodes

1. Declare the node in `src/nodes.h`:
```cpp
class NewAction : public BT::SyncActionNode
{
public:
    NewAction(const std::string& name) : BT::SyncActionNode(name, {}) {}
    BT::NodeStatus tick() override;
};
```

2. Implement in `src/nodes.cpp`:
```cpp
BT::NodeStatus NewAction::tick()
{
    std::cout << "[NewAction] Performing action..." << std::endl;
    return BT::NodeStatus::SUCCESS;
}
```

3. Register in `RegisterNodes` function:
```cpp
factory.registerNodeType<NewAction>("NewAction");
```

### Modifying the Behavior Tree

Edit the XML tree in `src/main.cpp` to change the execution flow or add new behaviors.

## Dependencies

- [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) - Behavior Tree library
- CMake - Build system
- C++14 Standard Library

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- BehaviorTree.CPP library for providing the behavior tree framework
- ROS community for inspiration in robotic task planning

## Related Projects

- [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) - The underlying behavior tree library
- [ROS2 Behavior Trees](https://github.com/ros-planning/navigation2) - ROS2 navigation with behavior trees

---

For questions or support, please open an issue on GitHub.
