import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class SimpleAvoidNode(Node):
    """
    简单的避障逻辑：
    - 前方安全 -> 前进
    - 前方受阻 -> 原地右转寻找出路
    """

    def __init__(self):
        super().__init__('simple_avoid_node')

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)

        self.safe_dist = 0.5    # 安全距离阈值 (单位: 米)
        self.front_dist = None  # 最新“前方距离”

        # 10Hz 控制循环
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Simple avoid node started.")

    def scan_callback(self, msg: LaserScan):
        ranges = msg.ranges
        n = len(ranges)
        if n == 0:
            return
        
        # --- 关键修正 ---
        # TurtleBot3 的雷达 0 是正前方
        # 我们取前方左右各 10 度范围的最小值
        window = 10  # 范围大小
        
        # 拼接数组末尾(350~360度)和数组开头(0~10度)
        front_ranges = ranges[-window:] + ranges[:window]
        
        # 过滤无效数据 (inf 或 0.0)
        valid_vals = [v for v in front_ranges if 0.05 < v < 10.0]
        
        if valid_vals:
            self.front_dist = min(valid_vals)
        else:
            # 如果全是 inf，说明前方非常空旷
            self.front_dist = 10.0 

    def control_loop(self):
        if self.front_dist is None:
            return

        twist = Twist()

        if self.front_dist < self.safe_dist:
            # 遇到障碍：停止并右转
            twist.linear.x = 0.0
            twist.angular.z = -0.5  # 负数是右转
            decision = "TURN RIGHT"
        else:
            # 前方安全：前进
            twist.linear.x = 0.2
            twist.angular.z = 0.0
            decision = "FORWARD"

        self.cmd_pub.publish(twist)
        
        # 减少日志刷屏，只有状态改变时或者每隔一段时间打印会更好，这里简单打印
        # self.get_logger().info(f"Dist: {self.front_dist:.2f} -> {decision}")


def main(args=None):
    rclpy.init(args=args)
    node = SimpleAvoidNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 退出前一定要停车，否则机器人会一直转
        shutdown_twist = Twist()
        node.cmd_pub.publish(shutdown_twist)
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
