# golf_cart_sim

**English** | [简体中文](README.zh-CN.md)

An Ackermann-steered golf cart simulation for ROS 2 Humble and Gazebo Classic 11, including a vehicle model, control interface, and a separate manual control UI.

[![Golf simulation](docs/media/golf-demo.gif)](docs/media/golf-demo.mp4)

## Getting Started

Requirements: Ubuntu 22.04, ROS 2 Humble, and Gazebo Classic 11.

Clone the package with Git LFS and build it in a ROS workspace:

```bash
sudo apt install git-lfs
git lfs install
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/max-niuniu/golf_cart_sim.git
cd golf_cart_sim
git lfs pull
cd ../..
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select golf_cart_sim --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch golf_cart_sim sim.launch.py
```

Gazebo and the manual control window open by default.

The mesh files use Git LFS. Use the clone commands above to download the complete models.

## Manual Control

Click **启用手动控制** (Enable manual control), then use:

- **W / S or ↑ / ↓**: hold to drive forward or reverse; release to decelerate to a stop.
- **A / D or ← / →**: hold to steer left or right.
- **Mouse**: adjust speed and steering with the sliders, then hold a forward or reverse button to drive.
- **Space / Esc / Stop button**: stop and disable manual control. Losing window focus or closing the window also disables it.

The UI and vehicle controller run as separate nodes. Disable manual control before using an external controller.

## ROS Interfaces

| Topic | Message Type | Description |
|---|---|---|
| `/golf/drive` | `ackermann_msgs/msg/AckermannDriveStamped` | Vehicle command input |
| `/golf/odom` | `nav_msgs/msg/Odometry` | Simulation ground-truth odometry |
| `/golf/joint_states` | `sensor_msgs/msg/JointState` | Wheel and steering joint states |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | Coordinate transforms |

Commands use `drive.speed` (m/s, positive forward) and `drive.steering_angle` (rad, equivalent front-wheel angle, positive left). Other fields are ignored. Publish at a recommended rate of 20 Hz; the vehicle decelerates to a stop if no command arrives for 0.5 seconds.

## Configuration

Vehicle dimensions and control parameters are defined in `config/vehicle.yaml`. Rebuild after editing.

Append launch arguments to `ros2 launch golf_cart_sim sim.launch.py`:

| Argument | Description |
|---|---|
| `manual_ui:=false` | Disable the manual control UI |
| `gui:=false manual_ui:=false` | Run without graphical windows |
| `detailed_visuals:=false` | Use simplified visuals |
| `world:=/path/to/world.world` | Load a custom world |
| `config:=/path/to/vehicle.yaml` | Load a custom vehicle configuration |
| `x:=0 y:=0 z:=0.30 yaw:=0` | Initial position (m) and heading (rad); z is the rear axle center height |

## Contributors

| Contributor | Contribution | Contact |
|---|---|---|
| [Tianwei Niu (@max-niuniu)](https://github.com/max-niuniu) | ROS simulation package development and maintenance | [tianwei.niu@cau.edu.cn](mailto:tianwei.niu@cau.edu.cn) |
| Zhongbiao He | SolidWorks vehicle modeling | [b20263081029@cau.edu.cn](mailto:b20263081029@cau.edu.cn) |
