import os
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from ament_index_python.packages import get_package_share_directory


class Click2Yaml(Node):
    def __init__(self):
        super().__init__('click2yaml')
        self.declare_parameter('count', 5)
        self.declare_parameter('outfile', 'waypoints.yaml')
        self.declare_parameter('yaw_deg', 0.0)  # 일괄 yaw 기본값
        self.declare_parameter('frame_id', 'map')

        self.need = int(self.get_parameter('count').value)
        self.outfile = str(self.get_parameter('outfile').value)
        self.yaw = float(self.get_parameter('yaw_deg').value)
        self.frame_id = str(self.get_parameter('frame_id').value)

        self.points = []
        self.sub = self.create_subscription(PointStamped, '/clicked_point', self.cb, 10)
        self.get_logger().info(f'Click {self.need} points in RViz on /clicked_point (frame: {self.frame_id})')

    def cb(self, msg: PointStamped):
        if msg.header.frame_id != self.frame_id:
            self.get_logger().warn(f'Ignored: frame_id {msg.header.frame_id} != {self.frame_id}')
            return
        self.points.append({'x': msg.point.x, 'y': msg.point.y, 'yaw_deg': self.yaw})
        self.get_logger().info(f'#{len(self.points)}: x={msg.point.x:.3f}, y={msg.point.y:.3f}')
        if len(self.points) >= self.need:
            data = {'frame_id': self.frame_id, 'poses': self.points}
            pkg = get_package_share_directory('pinky_wallpatrol')
            path = os.path.join(pkg, 'config', self.outfile)
            with open(path, 'w') as f:
                yaml.safe_dump(data, f)
            self.get_logger().info(f'Saved: {path}')
            rclpy.shutdown()


def main():
    rclpy.init()
    rclpy.spin(Click2Yaml())
