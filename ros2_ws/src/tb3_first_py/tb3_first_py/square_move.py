import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class SquareMoveNode(Node):
    """
    让 TurtleBot3 走一个“近似正方形”的简单节点：
    - 前进一段时间
    - 左转 90 度左右
    - 重复 4 次
    """

    def __init__(self):
        super().__init__('square_move_node')

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # 定时器周期（秒）
        self.timer_period = 0.1  # 10Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # 参数：可以以后自己调整
        self.linear_speed = 0.15      # m/s
        self.angular_speed = 0.6      # rad/s
        self.forward_time = 5.0       # 每条边前进时间（秒）
        self.turn_time = 2.7          # 旋转时间（秒）≈ 90 度

        # 换算成“周期次数”（tick 数）
        self.forward_ticks = int(self.forward_time / self.timer_period)
        self.turn_ticks = int(self.turn_time / self.timer_period)

        # 状态机
        self.state = 'forward'  # 'forward', 'turn', 'stop'
        self.tick_count = 0
        self.side_count = 0     # 已经走了多少条边

        self.get_logger().info('square_move_node started: drawing a square...')

    def timer_callback(self):
        twist = Twist()

        if self.state == 'forward':
            # 前进阶段
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0
            self.tick_count += 1

            if self.tick_count >= self.forward_ticks:
                # 前进结束，开始转弯
                self.tick_count = 0
                self.state = 'turn'
                self.get_logger().info(f'Start turning, side={self.side_count + 1}')

        elif self.state == 'turn':
            # 原地左转阶段
            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed
            self.tick_count += 1

            if self.tick_count >= self.turn_ticks:
                self.tick_count = 0
                self.side_count += 1

                if self.side_count >= 4:
                    # 已经四条边了，停车
                    self.state = 'stop'
                    self.get_logger().info('Square complete, stopping.')
                else:
                    # 继续下一条边
                    self.state = 'forward'
                    self.get_logger().info(f'Start next side: {self.side_count + 1}')

        elif self.state == 'stop':
            # 停止：保持发布 0 速度
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        # 发布速度指令
        self.cmd_pub.publish(twist)

    def stop_robot(self):
        # 退出前再发一次 0 速度
        twist = Twist()
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = SquareMoveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.get_logger().info('Shutting down square_move_node.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
