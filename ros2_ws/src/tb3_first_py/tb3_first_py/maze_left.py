import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class SimpleAvoidNode(Node):
    """
    最简单可靠的避障：
    - 前方距离 >= safe_dist：一直向前走
    - 前方距离 <  safe_dist：原地右转，直到前方有空位
    """

    def __init__(self):
        super().__init__('simple_avoid_node')

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)

        self.safe_dist = 2      # 安全距离阈值（可以调）
        self.front_dist = None    # 最新“前方距离”

        # 10Hz 控制循环
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Simple avoid node started.")

    def scan_callback(self, msg: LaserScan):
        # 取正前方附近一小段的最小距离
        ranges = list(msg.ranges)
        n = len(ranges)
        if n == 0:
            return
        center = n // 2
        window = max(3, n // 90)  # 小窗口

        sector = ranges[center - window:center + window]
        vals = [v for v in sector if 0.01 < v < float('inf')]
        if vals:
            self.front_dist = min(vals)
        else:
            self.front_dist = float('inf')

    def control_loop(self):
        if self.front_dist is None:
            # 还没收到激光，就先不动
            return

        twist = Twist()

        if self.front_dist < self.safe_dist:
            # 离前方墙太近：停止前进，原地右转
            twist.linear.x = 0.0
            twist.angular.z = -0.2
            decision = "turn_right"
        else:
            # 前方安全：向前走
            twist.linear.x = 0.15
            twist.angular.z = 0.0
            decision = "forward"

        self.cmd_pub.publish(twist)
        self.get_logger().info(
            f"front_dist={self.front_dist:.2f} -> {decision}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = SimpleAvoidNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 停车
        node.cmd_pub.publish(Twist())
        node.get_logger().info("Simple avoid node shutting down.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

