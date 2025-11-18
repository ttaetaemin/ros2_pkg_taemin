#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
from typing import List, Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.duration import Duration
from rclpy.time import Time

from std_msgs.msg import Int32, Bool, String
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from ament_index_python.packages import get_package_share_directory

# 🔧 TF2 추가
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

import yaml
import os


def yaw_to_quat(yaw: float):
    """roll=pitch=0 가정에서 yaw(rad) -> quaternion(x,y,z,w)"""
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class TmCenterlineNav(Node):
    """
    토픽 인터페이스:
      - 구독:
        /pinky_tmproject/target (std_msgs/Int32) : 1..5 목표 포인트 번호
        /pinky_tmproject/stop   (std_msgs/Bool)  : True -> 현재 주행 취소
      - 발행:
        /pinky_tmproject/status (std_msgs/String): 진행 상태 한글 메시지
        /pinky_tmproject/points_markers (MarkerArray): RViz 번호 표기
    """

    def __init__(self):
        super().__init__('tm_centerline_nav')

        # --- 파라미터/설정 로드 ---
        self.declare_parameter('points_yaml', '')
        self.declare_parameter('frame_id', 'map')        # 지도 프레임
        self.declare_parameter('base_frame', 'base_link')# 로봇 바디 프레임

        yaml_path = self.get_parameter('points_yaml').get_parameter_value().string_value
        if not yaml_path:
            pkg_share = get_package_share_directory('pinky_tmproject')
            yaml_path = os.path.join(pkg_share, 'config', 'points.yaml')

        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)

        # ✅ YAML 키 정수화 (e.g. "1","2" -> 1,2)
        raw_points = data['points']
        self.points: Dict[int, Dict[str, float]] = {int(k): v for k, v in raw_points.items()}

        # 프레임/파라미터
        self.frame_id: str = data.get('frame_id', self.get_parameter('frame_id').get_parameter_value().string_value)
        self.base_frame: str = data.get('base_frame', self.get_parameter('base_frame').get_parameter_value().string_value)

        self.center_x: float = float(data.get('center_x', 0.0))
        self.goal_tol_xy: float = float(data.get('goal_tolerance_xy', 0.25))
        self.goal_tol_yaw: float = float(data.get('goal_tolerance_yaw', 0.5))
        self.marker_scale: float = float(data.get('marker_scale', 0.25))

        # --- TF 버퍼/리스너 초기화 ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        # --- Nav2 Commander ---
        self.nav = BasicNavigator()
        self.nav.waitUntilNav2Active()

        # --- QoS & 토픽 ---
        qos_cmd = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.sub_target = self.create_subscription(
            Int32, '/pinky_tmproject/target', self.on_target, qos_cmd)

        self.sub_stop = self.create_subscription(
            Bool, '/pinky_tmproject/stop', self.on_stop, qos_cmd)

        self.pub_status = self.create_publisher(
            String, '/pinky_tmproject/status', 10)

        qos_markers = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # 래칭
        )
        self.pub_markers = self.create_publisher(
            MarkerArray, '/pinky_tmproject/points_markers', qos_markers
        )

        # 내부 상태
        self.current_goal_id: int = -1
        self.navigating: bool = False
        self.timer = self.create_timer(0.25, self.spin_once)

        # RViz 마커 1회 발행
        self.publish_point_markers()

        self.info("초기화 완료. /pinky_tmproject/target 에 1~5를 발행하면 이동합니다.")

    # -------------------- 유틸 --------------------

    def info(self, msg: str):
        self.get_logger().info(msg)
        self.pub_status.publish(String(data=msg))

    def warn(self, msg: str):
        self.get_logger().warn(msg)
        self.pub_status.publish(String(data=msg))

    def make_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        ps = PoseStamped()
        ps.header.frame_id = self.frame_id
        ps.header.stamp = self.get_clock().now().to_msg()
        qx, qy, qz, qw = yaw_to_quat(yaw)
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = 0.0
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        return ps

    # 🔧 TF를 사용해 현재 포즈 조회 (map -> base_link)
    def current_pose(self, timeout_sec: float = 0.5) -> PoseStamped:
        try:
            t = self.tf_buffer.lookup_transform(
                self.frame_id,              # target frame (e.g., "map")
                self.base_frame,            # source frame (e.g., "base_link")
                Time(),                     # "latest"
                timeout=Duration(seconds=timeout_sec)
            )
            ps = PoseStamped()
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.header.frame_id = self.frame_id
            ps.pose.position.x = t.transform.translation.x
            ps.pose.position.y = t.transform.translation.y
            ps.pose.position.z = t.transform.translation.z
            ps.pose.orientation = t.transform.rotation
            return ps
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.warn(f"TF 조회 실패({self.frame_id} <- {self.base_frame}): {e}. (임시 0,0,0 사용)")
            # 안전 폴백: (0,0,0)
            ps = PoseStamped()
            ps.header.frame_id = self.frame_id
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose.position.x = 0.0
            ps.pose.position.y = 0.0
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            return ps

    def heading_between(self, a: PoseStamped, b: PoseStamped) -> float:
        dx = b.pose.position.x - a.pose.position.x
        dy = b.pose.position.y - a.pose.position.y
        return math.atan2(dy, dx)

    # -------------------- 센터라인 경로 생성 --------------------

    def build_centerline_path(self, goal_xy) -> List[PoseStamped]:
        """
        항상 x = center_x 세로축을 경유하는 경로:
          [ (center_x, start_y) -> (center_x, goal_y) -> (goal_x, goal_y) ]
        각 세그먼트의 진행 방향으로 yaw를 맞춤.
        """
        start = self.current_pose()
        goal_x, goal_y = goal_xy
        waypoints: List[PoseStamped] = []

        # 1) 수평 이동 → (center_x, start_y)
        a = PoseStamped()
        a.header.frame_id = self.frame_id
        a.pose.position.x = self.center_x
        a.pose.position.y = start.pose.position.y
        yaw_a = self.heading_between(start, a)
        waypoints.append(self.make_pose(a.pose.position.x, a.pose.position.y, yaw_a))

        # 2) 세로축 따라 목표 y까지 → (center_x, goal_y)
        #    위/아래 방향을 기준으로 yaw 설정
        yaw_b = math.pi / 2 if goal_y >= a.pose.position.y else -math.pi / 2
        b = self.make_pose(self.center_x, goal_y, yaw_b)
        waypoints.append(b)

        # 3) 목표 x까지 수평 이동 → (goal_x, goal_y)
        yaw_c = 0.0 if goal_x >= self.center_x else math.pi
        c = self.make_pose(goal_x, goal_y, yaw_c)
        waypoints.append(c)

        return waypoints

    # -------------------- RViz 마커 --------------------

    def publish_point_markers(self):
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()
        for idx, p in self.points.items():
            # 숫자 텍스트
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = now
            m.ns = "pinky_tm_points"
            m.id = int(idx) * 10 + 1
            m.type = Marker.TEXT_VIEW_FACING
            m.action = Marker.ADD
            m.pose.position.x = float(p['x'])
            m.pose.position.y = float(p['y'])
            m.pose.position.z = 0.2
            m.scale.z = self.marker_scale
            m.color.r, m.color.g, m.color.b, m.color.a = (1.0, 0.0, 0.0, 1.0)
            m.text = str(idx)
            ma.markers.append(m)

            # 위치 점(작은 구)
            s = Marker()
            s.header.frame_id = self.frame_id
            s.header.stamp = now
            s.ns = "pinky_tm_points"
            s.id = int(idx) * 10 + 2
            s.type = Marker.SPHERE
            s.action = Marker.ADD
            s.pose.position.x = float(p['x'])
            s.pose.position.y = float(p['y'])
            s.pose.position.z = 0.05
            s.scale.x = s.scale.y = s.scale.z = 0.06
            s.color.r, s.color.g, s.color.b, s.color.a = (0.1, 0.6, 1.0, 0.9)
            ma.markers.append(s)

        self.pub_markers.publish(ma)

    # -------------------- 콜백/메인 루프 --------------------

    def on_target(self, msg: Int32):
        idx = int(msg.data)
        if idx not in self.points:
            self.warn(f"알 수 없는 포인트 번호 {idx}. (1~5)")
            return
        if self.navigating:
            self.warn("이동 중 명령을 갱신합니다(기존 목표 취소).")
            self.nav.cancelTask()

        self.current_goal_id = idx
        goal = self.points[idx]
        goal_xy = (float(goal['x']), float(goal['y']))
        path = self.build_centerline_path(goal_xy)

        # 안내
        self.info(f"{idx}번 포인트로 가는 중입니다.")

        # 내비게이션 시작 (Waypoints: [PoseStamped, ...])
        self.nav.followWaypoints(path)
        self.navigating = True

    def on_stop(self, msg: Bool):
        if msg.data:
            try:
                self.nav.cancelTask()
            except Exception as e:
                self.warn(f"cancelTask 예외: {e}")
            self.navigating = False
            self.info("멈춤 명령을 받았습니다. 현재 주행을 중단했습니다.")
            
    def spin_once(self):
        if not self.navigating:
            return

        status = self.nav.getResult()
        if status is None:
            # 진행률/피드백은 필요 시 여기에 처리
            return

        # 완료/실패/취소 처리
        if status == TaskResult.SUCCEEDED:
            self.info(f"{self.current_goal_id}번 포인트에 도착했습니다.")
        elif status == TaskResult.CANCELED:
            self.info("멈췄습니다.")
        else:
            self.warn("목표에 도달하지 못했습니다(실패).")

        self.navigating = False
        self.current_goal_id = -1


def main():
    rclpy.init()
    node = TmCenterlineNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
