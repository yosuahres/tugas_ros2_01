| Name           | NRP        | Class     |
| ---            | ---        | ----------|
| Yosua Hares | 5025221270 | Robotika (T)|
| Putu Indra Mahendra | 5025221215 | Robotika (T) |
|Fernando|5025231091|Robotika (T)|
|Muhammad Ihsan Al Khwaritsmi|5025221211|Robotika (T)|
|Aryaka Leorgi Eprideka|5025231117|Robotika (T)|
|Rahmad Bisma Zulfi Pahlevi|5025231290|Robotika (T)|


# Tugas 1 Robotika - ROS2

## Link Demo:
[Video Demo](https://drive.google.com/file/d/1ii1GfZ3U7n-s3BWm-agACutXsV3ozUOK/view?usp=drive_link)

## Quick Setup 

### Using HTTP:

```sh
mkdir -p robot_ws/src && cd robot_ws/src
git clone https://github.com/yosuahres/tugas_ros2_01.git
cd ..
source /opt/ros/humble/setup.bash
colcon build --packages-select tugas1_msgs tugas1_mediapipe tugas1_turtlesim
source install/setup.bash 
ros2 run tugas1_mediapipe camera_publisher 

source install/setup.bash ## beda terminal
ros2 run tugas1_mediapipe mediapipe

source install/setup.bash ## beda terminal
ros2 run tugas1_turtlesim turtle
```
