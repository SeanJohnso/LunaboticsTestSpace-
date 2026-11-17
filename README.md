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

Frames follow REP-103/105: `odom` → `base_link` → `t265_link` (x forward, y left, z up). The node turns the camera's pose into the robot's pose using the camera mount (`mount_xyz`, `mount_rpy` in `t265.yaml`; currently 20 cm right of centre, facing right), so `odom` starts at the robot's start position and spinning in place doesn't show up as movement. Settings in `src/t265_wrapper/config/t265.yaml` (rebuild after editing).

| Topic | Type | Rate | Enabled by |
|---|---|---|---|
| `/t265/odom` | `nav_msgs/Odometry`, `odom` → `base_link` (twist in `base_link`) | 200 Hz | always |
| `/t265/confidence` | `std_msgs/UInt8` (0 failed … 3 high) | 200 Hz | always |
| `/tf` `odom` → `base_link` | | 200 Hz | `publish_tf: true` |
| `/tf_static` `base_link` → `t265_link` (the mount) | | once | `publish_mount_tf: true` |
| `/t265/imu/gyro`, `/t265/imu/accel` | `sensor_msgs/Imu` in `t265_link` | 200 / 62.5 Hz | `enable_imu: true` |
| `/t265/fisheye{1,2}/image_raw`, `.../camera_info` | `Image` mono8 848×800, `CameraInfo` (`equidistant`) | 30 Hz | `enable_fisheye: true` |
| `/tf_static` `t265_link` → `t265_fisheye{1,2}_optical_frame` | | once | `enable_fisheye: true` |

### RViz (dev machine)

Build the package on the dev machine too (pure Python, works on Humble), then with `t265.launch.py` running on the Pi:

```bash
ros2 launch t265_wrapper t265_rviz.launch.py
```

Shows `/t265/odom` (arrow trail, fixed frame `odom`) and TF. From a VS Code terminal (snap) RViz crashes with `libpthread.so.0: undefined symbol: __libc_pthread_init`; use a normal terminal, or `unset GTK_PATH GIO_MODULE_DIR LOCPATH` first.

Odometry covariance follows tracker confidence: pose `pose_covariance × 10^(3−conf)`, twist `twist_covariance × 10^(1−conf)`.
