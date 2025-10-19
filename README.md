| Name           | NRP        | Class     |
| ---            | ---        | ----------|
| Yosua Hares | 5025221270 | Robotika (T)|
| Putu Indra Mahendra | 5025221215 | Robotika (T) |
Fernando|5025231091|Robotika (T)|
|Muhammad Ihsan Al Khwaritsmi|5025221211|Robotika (T)|
|Aryaka Leorgi Eprideka|5025231117|Robotika (T)|
|Rahmad Bisma Zulfi Pahlevi|5025231290|Robotika (T)|


# Tugas 1 Robotika - ROS2

## Link Demo:


## Quick Setup 

### Using HTTP:

```
mkdir -p robot_ws/src && cd robot_ws/src
git clone https://github.com/yosuahres/tugas_ros2_01.git
cd ../..
colcon build --packages-select robot2025_msgs
source install/setup.bash ## or .zsh if using zsh terminal
colcon build --packages-select robot2025_mediapipe
source install/setup.bash ## or .zsh if using zsh terminal
```
