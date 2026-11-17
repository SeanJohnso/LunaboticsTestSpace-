"""ROS 2 node for the Intel RealSense T265 (pyrealsense2 v2.50).

Frames (REP-103 / REP-105):
  odom -> base_frame (t265_link): T265 pose, x forward, y left, z up.
  base_frame -> t265_fisheye{1,2}_optical_frame: static, from the device's
  factory extrinsics (optical convention: x right, y down, z forward).

The T265 itself reports in its own frame: X right, Y up, Z backward.
"""
import array
import threading
import time

import numpy as np
import pyrealsense2 as rs
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, Imu
from std_msgs.msg import UInt8
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

# Maps a vector in T265 axes (X right, Y up, Z back) to ROS body axes
# (x forward, y left, z up): x = -Z, y = -X, z = Y.
C = np.array([[0.0, 0.0, -1.0],
              [-1.0, 0.0, 0.0],
              [0.0, 1.0, 0.0]])

NO_FRAME_TIMEOUT = 3.0  # s without any frame -> restart the pipeline
RETRY_PERIOD = 2.0      # s between start attempts / watchdog checks


def quat_to_mat(w, x, y, z):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def mat_to_quat(m):
    """Rotation matrix -> (w, x, y, z)."""
    t = np.trace(m)
    if t > 0:
        s = 2.0 * np.sqrt(t + 1.0)
        return (0.25 * s, (m[2, 1] - m[1, 2]) / s,
                (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s)
    i = int(np.argmax(np.diag(m)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2.0 * np.sqrt(1.0 + m[i, i] - m[j, j] - m[k, k])
    q = [0.0, 0.0, 0.0]
    q[i] = 0.25 * s
    q[j] = (m[j, i] + m[i, j]) / s
    q[k] = (m[k, i] + m[i, k]) / s
    return ((m[k, j] - m[j, k]) / s, q[0], q[1], q[2])


def extrinsics_to_pose(profile, pose_profile):
    """Rotation and translation taking points in `profile`'s frame to the T265 pose frame."""
    e = profile.get_extrinsics_to(pose_profile)
    r = np.array(e.rotation).reshape(3, 3).T  # librealsense stores it column-major
    return r, np.array(e.translation)


class T265Node(Node):
    def __init__(self):
        super().__init__('t265')
        self.serial = self.declare_parameter('serial', '').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 't265_link').value
        self.publish_tf = self.declare_parameter('publish_tf', True).value
        self.enable_imu = self.declare_parameter('enable_imu', False).value
        self.enable_fisheye = self.declare_parameter('enable_fisheye', False).value
        self.pose_cov = self.declare_parameter('pose_covariance', 0.01).value
        self.twist_cov = self.declare_parameter('twist_covariance', 0.01).value

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.conf_pub = self.create_publisher(UInt8, 'confidence', 10)
        self.tf_pub = TransformBroadcaster(self) if self.publish_tf else None
        if self.enable_imu:
            self.gyro_pub = self.create_publisher(Imu, 'imu/gyro', 10)
            self.accel_pub = self.create_publisher(Imu, 'imu/accel', 10)
        if self.enable_fisheye:
            self.static_tf_pub = StaticTransformBroadcaster(self)
            self.img_pubs = {i: self.create_publisher(Image, f'fisheye{i}/image_raw', 10)
                             for i in (1, 2)}
            self.info_pubs = {i: self.create_publisher(CameraInfo, f'fisheye{i}/camera_info', 10)
                              for i in (1, 2)}

        self.pipe = None
        self.last_frame = 0.0
        self.imu_rot = {}       # stream -> 3x3, IMU axes -> ROS body axes
        self.cam_info = {}      # fisheye index -> CameraInfo template
        self.lock = threading.Lock()

        self.start_pipeline()
        self.create_timer(RETRY_PERIOD, self.watchdog)

    # ---------------------------------------------------------------- device

    def start_pipeline(self):
        cfg = rs.config()
        if self.serial:
            cfg.enable_device(self.serial)
        cfg.enable_stream(rs.stream.pose)
        if self.enable_imu:
            cfg.enable_stream(rs.stream.gyro)
            cfg.enable_stream(rs.stream.accel)
        if self.enable_fisheye:
            cfg.enable_stream(rs.stream.fisheye, 1)
            cfg.enable_stream(rs.stream.fisheye, 2)

        pipe = rs.pipeline()
        try:
            self.last_frame = time.monotonic()
            profile = pipe.start(cfg, self.on_frame)
        except RuntimeError as e:
            self.get_logger().warn(f'T265 not started: {e}; retrying', throttle_duration_sec=30.0)
            return
        self.pipe = pipe
        dev = profile.get_device()
        self.get_logger().info(
            f'T265 started: serial {dev.get_info(rs.camera_info.serial_number)}, '
            f'firmware {dev.get_info(rs.camera_info.firmware_version)}')

        pose_profile = profile.get_stream(rs.stream.pose)
        if self.enable_imu:
            for st in (rs.stream.gyro, rs.stream.accel):
                r, _ = extrinsics_to_pose(profile.get_stream(st), pose_profile)
                self.imu_rot[st] = C @ r
        if self.enable_fisheye:
            tfs = []
            for i in (1, 2):
                sp = profile.get_stream(rs.stream.fisheye, i)
                self.cam_info[i] = self.make_camera_info(sp, i)
                r, t = extrinsics_to_pose(sp, pose_profile)
                tfs.append(self.make_tf(self.base_frame, self.optical_frame(i),
                                        C @ t, mat_to_quat(C @ r), self.get_clock().now().to_msg()))
            self.static_tf_pub.sendTransform(tfs)

    def stop_pipeline(self):
        if self.pipe is not None:
            try:
                self.pipe.stop()
            except RuntimeError:
                pass
            self.pipe = None

    def watchdog(self):
        with self.lock:
            if self.pipe is None:
                self.start_pipeline()
            elif time.monotonic() - self.last_frame > NO_FRAME_TIMEOUT:
                self.get_logger().error(f'No T265 frames for {NO_FRAME_TIMEOUT} s; restarting')
                self.stop_pipeline()

    # ---------------------------------------------------------------- frames

    def on_frame(self, frame):
        """librealsense callback thread."""
        self.last_frame = time.monotonic()
        # T265 frame timestamps are on the device clock; stamp with ROS time on arrival.
        stamp = self.get_clock().now().to_msg()
        try:
            if frame.is_frameset():
                fs = frame.as_frameset()
                for k in range(fs.size()):
                    self.handle_frame(fs[k], stamp)
            else:
                self.handle_frame(frame, stamp)
        except Exception as e:  # never let one bad frame kill the callback thread
            self.get_logger().error(f'frame handling failed: {e}', throttle_duration_sec=5.0)

    def handle_frame(self, f, stamp):  # not `handle`: that is rclpy Node's own attribute
        sp = f.get_profile()
        st = sp.stream_type()
        if st == rs.stream.pose:
            self.publish_pose(f.as_pose_frame().get_pose_data(), stamp)
        # IMU / fisheye frames can arrive before start_pipeline() has read the extrinsics.
        elif st in self.imu_rot:
            self.publish_imu(st, f.as_motion_frame().get_motion_data(), stamp)
        elif st == rs.stream.fisheye and sp.stream_index() in self.cam_info:
            self.publish_image(sp.stream_index(), f.as_video_frame(), stamp)

    def publish_pose(self, p, stamp):
        # Rotation T265 -> ROS: R_ros = C R C^T, i.e. quaternion vector part rotated by C.
        q = (p.rotation.w, *(C @ [p.rotation.x, p.rotation.y, p.rotation.z]))
        pos = C @ [p.translation.x, p.translation.y, p.translation.z]
        r_body = quat_to_mat(*q)
        # T265 velocities are in the odom frame; Odometry twist is in the child frame.
        vel = r_body.T @ (C @ [p.velocity.x, p.velocity.y, p.velocity.z])
        ang = r_body.T @ (C @ [p.angular_velocity.x, p.angular_velocity.y, p.angular_velocity.z])

        conf = int(p.tracker_confidence)  # 0 failed, 1 low, 2 medium, 3 high
        cov_pose = self.pose_cov * 10.0 ** (3 - conf)
        cov_twist = self.twist_cov * 10.0 ** (1 - conf)

        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        pp, po = msg.pose.pose.position, msg.pose.pose.orientation
        pp.x, pp.y, pp.z = (float(v) for v in pos)
        po.w, po.x, po.y, po.z = (float(v) for v in q)
        tl, ta = msg.twist.twist.linear, msg.twist.twist.angular
        tl.x, tl.y, tl.z = (float(v) for v in vel)
        ta.x, ta.y, ta.z = (float(v) for v in ang)
        for k in range(6):
            msg.pose.covariance[k * 7] = cov_pose
            msg.twist.covariance[k * 7] = cov_twist
        self.odom_pub.publish(msg)
        self.conf_pub.publish(UInt8(data=conf))

        if self.tf_pub is not None:
            self.tf_pub.sendTransform(
                self.make_tf(self.odom_frame, self.base_frame, pos, q, stamp))

    def publish_imu(self, st, d, stamp):
        v = self.imu_rot[st] @ [d.x, d.y, d.z]
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = self.base_frame
        msg.orientation_covariance[0] = -1.0  # no orientation in raw IMU data
        if st == rs.stream.gyro:
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = (
                float(x) for x in v)
            msg.linear_acceleration_covariance[0] = -1.0
            self.gyro_pub.publish(msg)
        else:
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = (
                float(x) for x in v)
            msg.angular_velocity_covariance[0] = -1.0
            self.accel_pub.publish(msg)

    def publish_image(self, i, vf, stamp):
        data = np.asanyarray(vf.get_data())
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self.optical_frame(i)
        msg.height, msg.width = data.shape
        msg.encoding = 'mono8'
        msg.step = msg.width
        msg.data = array.array('B', data.tobytes())  # array.array skips rclpy's per-element check
        self.img_pubs[i].publish(msg)

        info = self.cam_info[i]
        info.header = msg.header
        self.info_pubs[i].publish(info)

    # ---------------------------------------------------------------- helpers

    def optical_frame(self, i):
        return f't265_fisheye{i}_optical_frame'

    def make_camera_info(self, sp, i):
        k = sp.as_video_stream_profile().get_intrinsics()
        info = CameraInfo()
        info.width, info.height = k.width, k.height
        info.distortion_model = 'equidistant'  # T265 fisheye = Kannala-Brandt 4
        info.d = [float(c) for c in k.coeffs[:4]]
        info.k = [k.fx, 0.0, k.ppx, 0.0, k.fy, k.ppy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [k.fx, 0.0, k.ppx, 0.0, 0.0, k.fy, k.ppy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    @staticmethod
    def make_tf(parent, child, t, q, stamp):
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = parent
        tf.child_frame_id = child
        tr, ro = tf.transform.translation, tf.transform.rotation
        tr.x, tr.y, tr.z = (float(v) for v in t)
        ro.w, ro.x, ro.y, ro.z = (float(v) for v in q)
        return tf


def main():
    rclpy.init()
    node = T265Node()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        with node.lock:
            node.stop_pipeline()
        node.destroy_node()
        rclpy.try_shutdown()
