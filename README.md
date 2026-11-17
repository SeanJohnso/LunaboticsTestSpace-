# LunaboticsTestSpace-
Test space for lunabotics 

This repo is a ROS 2 (Jazzy) colcon workspace: packages live in `src/`.

## Build (on the Pi)

```bash
cd ~/LunaboticsTestSpace-
colcon build --packages-select t265_wrapper
source install/setup.bash
```

## t265_wrapper

Intel RealSense T265 visual odometry as ROS 2 topics. Uses librealsense **v2.50** (the last version with T265 support) from `~/t265_project/install`; the launch file sets the paths, so nothing needs adding to `.bashrc`.

```bash
ros2 launch t265_wrapper t265.launch.py
# other librealsense install:  t265_install:=/path/to/install
# other settings:              params_file:=/path/to/t265.yaml
```

Frames follow REP-103/105: `odom` → `t265_link` (x forward, y left, z up). Settings in `src/t265_wrapper/config/t265.yaml` (rebuild after editing).

| Topic | Type | Rate | Enabled by |
|---|---|---|---|
| `/t265/odom` | `nav_msgs/Odometry` (twist in `t265_link`) | 200 Hz | always |
| `/t265/confidence` | `std_msgs/UInt8` (0 failed … 3 high) | 200 Hz | always |
| `/tf` `odom` → `t265_link` | | 200 Hz | `publish_tf: true` |
| `/t265/imu/gyro`, `/t265/imu/accel` | `sensor_msgs/Imu` in `t265_link` | 200 / 62.5 Hz | `enable_imu: true` |
| `/t265/fisheye{1,2}/image_raw`, `.../camera_info` | `Image` mono8 848×800, `CameraInfo` (`equidistant`) | 30 Hz | `enable_fisheye: true` |
| `/tf_static` `t265_link` → `t265_fisheye{1,2}_optical_frame` | | once | `enable_fisheye: true` |

Odometry covariance follows tracker confidence: pose `pose_covariance × 10^(3−conf)`, twist `twist_covariance × 10^(1−conf)`.
