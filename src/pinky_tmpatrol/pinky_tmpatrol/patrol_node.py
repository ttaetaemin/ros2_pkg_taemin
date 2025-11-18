#!/usr/bin/env python3
import math
import os
import time
import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from ament_index_python.packages import get_package_share_directory


def yaw_to_quat(yaw_rad: float):
    """Convert yaw (rad) to quaternion (x,y,z,w) without external deps."""
    half = yaw_rad * 0.5
    cz = math.cos(half)
    sz = math.sin(half)
    # roll=pitch=0 -> quaternion = (0,0,sin(z/2),cos(z/2))
    return (0.0, 0.0, sz, cz)


class PatrolNode(Node):
    def __init__(self):
        super().__init__('pinky_wall_patrol')

        # Parameters
        # - waypoints_file: 'waypoints.yaml' (pkg share/config) 또는 절대경로 지원
        # - initpose: [x, y, yaw_deg] (선택, 없으면 RViz에서 2D Pose Estimate 사용)
        self.declare_parameter('waypoints_file', 'waypoints.yaml')
        self.declare_parameter('initpose', [])

        # Load waypoints
        wfile = self.get_parameter('waypoints_file').get_parameter_value().string_value

        # ✅ 절대경로면 그대로 사용, 아니면 패키지 share/config 안에서 찾기
        if os.path.isabs(wfile):
            wpath = wfile
        else:
            pkg_share = get_package_share_directory('pinky_tmpatrol')  # ← 패키지명 교정
            wpath = os.path.join(pkg_share, 'config', wfile)

        if not os.path.exists(wpath):
            raise FileNotFoundError(f'Waypoints file not found: {wpath}')

        with open(wpath, 'r') as f:
            data = yaml.safe_load(f)

        self.frame_id = data.get('frame_id', 'map')
        self.waypoints = []

        for i, p in enumerate(data.get('poses', []), start=1):
            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.pose.position.x = float(p['x'])
            pose.pose.position.y = float(p['y'])
            yaw_rad = math.radians(float(p.get('yaw_deg', 0.0)))
            qx, qy, qz, qw = yaw_to_quat(yaw_rad)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            self.waypoints.append(pose)

        if not self.waypoints:
            raise RuntimeError('No waypoints found in file.')

        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints from {wpath}')

        # Navigator
        self.navigator = BasicNavigator()

        # Optional initial pose
        init = self.get_parameter('initpose').value
        if isinstance(init, (list, tuple)) and len(init) == 3:
            ix, iy, iyaw_deg = float(init[0]), float(init[1]), float(init[2])
            ipose = PoseStamped()
            ipose.header.frame_id = self.frame_id
            ipose.pose.position.x = ix
            ipose.pose.position.y = iy
            qx, qy, qz, qw = yaw_to_quat(math.radians(iyaw_deg))
            ipose.pose.orientation.x = qx
            ipose.pose.orientation.y = qy
            ipose.pose.orientation.z = qz
            ipose.pose.orientation.w = qw
            self.navigator.setInitialPose(ipose)
            self.get_logger().info(
                f'InitialPose set: ({ix:.2f}, {iy:.2f}, {iyaw_deg:.1f} deg)'
            )
        else:
            self.get_logger().info(
                'No initpose param provided. Use RViz "2D Pose Estimate" to initialize AMCL.'
            )

        # Wait until Nav2 is active
        self.get_logger().info('Waiting for Nav2 to become active...')
        self.navigator.waitUntilNav2Active()
        time.sleep(0.2)

        # Start patrol
        self.run_patrol()

    def run_patrol(self):
        n = len(self.waypoints)
        self.get_logger().info(f'Waypoint patrol start: {n} goals')
        self.navigator.followWaypoints(self.waypoints)

        while not self.navigator.isTaskComplete():
            fb = self.navigator.getFeedback()
            if fb and (fb.current_waypoint is not None):
                idx = int(fb.current_waypoint) + 1
                self.get_logger().info_throttle(
                    1.0, f'Proceeding: {idx}/{len(self.waypoints)}'
                )
            rclpy.spin_once(self, timeout_sec=0.1)

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('All waypoints visited ✔')
        elif result == TaskResult.CANCELED:
            self.get_logger().warn('Task canceled')
        else:
            self.get_logger().error(f'Task failed: {result}')

        rclpy.shutdown()


def main():
    rclpy.init()
    PatrolNode()


if __name__ == '__main__':
    main()
