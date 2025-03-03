# Object Detection in Gazebo using YOLOv5 and ROS1

## Overview
This repository provides a ROS1 package for real-time object detection in a Gazebo simulation using YOLOv5. The package processes images from a simulated camera in Gazebo and performs object detection using a pre-trained YOLOv5 model.

## Features
- Real-time object detection in Gazebo
- Integration with ROS1 image topics
- Joystick control for navigation (alternative keyboard control can be implemented)
- Customizable YOLOv5 model support

## Installation

### Prerequisites
Ensure you have the following installed:
- **Ubuntu 18.04/20.04** (Recommended for ROS1 compatibility)
- **ROS1 (Melodic/Noetic)**
- **Gazebo** (Installed with ROS1 or standalone)
- **Python 3**
- **PyTorch**
- **YOLOv5**
- **OpenCV**
- **cv_bridge** (for ROS to OpenCV image conversion)
- **Joystick drivers** (if using joystick control)

### Clone the Repository
```bash
git clone https://github.com/yourusername/yourrepository.git
cd yourrepository
```

### Install Dependencies
```bash
pip install -r requirements.txt
```

### Set Up a Catkin Workspace
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
ln -s $(pwd)/yourrepository yolov5_ros
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Usage

### 1. Launch Gazebo
```bash
roslaunch gazebo_ros empty_world.launch
```

### 2. Run the Object Detection Node
```bash
rosrun yolov5_ros detect.py
```

### 3. Control the Robot
- **Joystick Control**: Ensure your joystick is connected and configured in ROS.
- **Keyboard Control** (Optional): You can modify the code to use keyboard commands instead of a joystick.

### 4. View the Detections
```bash
rqt_image_view /yolov5/detections
```

## Customization
- **Use a different YOLO model**: Replace `'yolov5s.pt'` with `'yolov5m.pt'` or a custom-trained model.
- **Change camera topic**: Modify `rospy.Subscriber("/camera/image_raw", Image, self.callback)` in `detect.py` to match your camera topic.
- **Switch Control Method**: Modify the control script to use keyboard input instead of a joystick if desired.

## Contributing
Contributions are welcome! Feel free to open issues and pull requests.

## License
This project is licensed under the MIT License.

## Credits
- YOLOv5: [https://github.com/ultralytics/yolov5](https://github.com/ultralytics/yolov5)
- ROS: [http://wiki.ros.org/](http://wiki.ros.org/)