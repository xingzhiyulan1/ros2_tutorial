import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen


class TurtleSquare(Node):
    def __init__(self):
        super().__init__('turtle_square')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_cb, 10)
        self.pen_cli = self.create_client(SetPen, '/turtle1/set_pen')
        while not self.pen_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for set_pen service...')
        self.pose = None
        self.state = 'forward'
        self.side = 2.0
        self.sides_done = 0
        self.start = None
        self.target_theta = None
        self.timer = self.create_timer(0.05, self.loop)

    def pose_cb(self, msg):
        self.pose = msg

    def set_pen(self, off):
        req = SetPen.Request()
        req.r = 0
        req.g = 0
        req.b = 0
        req.width = 2
        req.off = off
        self.pen_cli.call_async(req)

    @staticmethod
    def norm(a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    def loop(self):
        if self.pose is None:
            return
        if self.start is None:
            self.start = (self.pose.x, self.pose.y)
            self.target_theta = self.pose.theta
        twist = Twist()
        if self.state == 'forward':
            dx = self.pose.x - self.start[0]
            dy = self.pose.y - self.start[1]
            # P control: keep heading on the straight line
            err = self.norm(self.target_theta - self.pose.theta)
            twist.angular.z = max(min(2.0 * err, 1.0), -1.0)
            twist.linear.x = 0.8
            if math.hypot(dx, dy) >= self.side:
                self.state = 'turn'
                self.target_theta = self.norm(self.pose.theta + math.pi / 2)
                self.set_pen(1)  # pen up while turning
        elif self.state == 'turn':
            remaining = self.norm(self.target_theta - self.pose.theta)
            # P control: slow down near the target angle
            twist.angular.z = max(min(2.0 * remaining, 1.5), -1.5)
            if abs(remaining) < 0.03:
                self.sides_done += 1
                if self.sides_done >= 4:
                    self.set_pen(1)
                    self.get_logger().info('square done!')
                    self.timer.cancel()
                    return
                self.state = 'forward'
                self.start = (self.pose.x, self.pose.y)
                self.target_theta = self.pose.theta
                self.set_pen(0)  # pen down for the next side
        self.pub.publish(twist)


def main():
    rclpy.init()
    rclpy.spin(TurtleSquare())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
