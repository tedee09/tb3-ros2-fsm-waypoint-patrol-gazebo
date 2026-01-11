#!/usr/bin/env python3
import math
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from sensor_msgs.msg import LaserScan

def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def norm_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class State(Enum):
    INIT = 0
    ROTATE_TO_WP = 1
    DRIVE_TO_WP = 2
    AVOID_TURN = 3
    AVOID_FORWARD = 4
    ARRIVED = 5


class FSMPatrol(Node):
    def __init__(self):
        super().__init__("tb3_fsm_patrol")

        # Topics (default TB3 gazebo)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/odom")

        # waypoint list [x1,y1,x2,y2,...]
        self.declare_parameter("waypoints", [-1.84, -1.12,  1.16, -1.70,  1.65, 1.16,  -1.14, 1.66])
        self.declare_parameter("goal_tol", 0.10)

        # obstacle
        self.declare_parameter("obstacle_front", 0.35)
        self.declare_parameter("front_fov_deg", 25.0)

        # speed
        self.declare_parameter("lin_speed", 0.18)
        self.declare_parameter("ang_speed", 0.7)

        # avoid timing
        self.declare_parameter("avoid_turn_s", 1.0)
        self.declare_parameter("avoid_fwd_s", 0.8)

        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        scan_topic = self.get_parameter("scan_topic").value
        odom_topic = self.get_parameter("odom_topic").value

        wp_flat = list(self.get_parameter("waypoints").value)
        if len(wp_flat) < 2 or len(wp_flat) % 2 != 0:
            raise RuntimeError("Parameter waypoints harus [x1,y1,x2,y2,...]")

        self.waypoints = [(wp_flat[i], wp_flat[i + 1]) for i in range(0, len(wp_flat), 2)]
        self.goal_tol = float(self.get_parameter("goal_tol").value)

        self.obstacle_front = float(self.get_parameter("obstacle_front").value)
        self.front_fov = math.radians(float(self.get_parameter("front_fov_deg").value))

        self.lin_speed = float(self.get_parameter("lin_speed").value)
        self.ang_speed = float(self.get_parameter("ang_speed").value)

        self.avoid_turn_s = float(self.get_parameter("avoid_turn_s").value)
        self.avoid_fwd_s = float(self.get_parameter("avoid_fwd_s").value)

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.wp_pub = self.create_publisher(Int32, "/fsm/wp_idx", 10)
        self.goal_pub = self.create_publisher(Point, "/fsm/goal", 10)
        self.create_subscription(Odometry, odom_topic, self.on_odom, 10)
        self.create_subscription(LaserScan, scan_topic, self.on_scan, 10)

        self.timer = self.create_timer(0.05, self.loop)  # 20 Hz

        self.state = State.INIT
        self.wp_idx = 0

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.have_odom = False

        self.front_min = float("inf")
        self.left_min = float("inf")
        self.right_min = float("inf")
        self.avoid_dir = 1.0   # +1 = belok kiri, -1 = belok kanan (default)
        self.have_scan = False

        self.until_time = None
        self.last_pub_wp_idx = None

        self.get_logger().info(f"FSM Patrol ready. waypoints={self.waypoints}")

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.x, self.y = float(p.x), float(p.y)
        self.yaw = yaw_from_quat(q)
        self.have_odom = True

    def on_scan(self, msg: LaserScan):
        a = msg.angle_min

        front_min = float("inf")
        left_min = float("inf")
        right_min = float("inf")

        for r in msg.ranges:
            if not math.isfinite(r):
                a += msg.angle_increment
                continue

            # depan: +-front_fov
            if abs(a) <= self.front_fov:
                front_min = min(front_min, r)

            # kiri: 30° s/d 90°
            if math.radians(30.0) <= a <= math.radians(90.0):
                left_min = min(left_min, r)

            # kanan: -90° s/d -30°
            if -math.radians(90.0) <= a <= -math.radians(30.0):
                right_min = min(right_min, r)

            a += msg.angle_increment

        self.front_min = front_min
        self.left_min = left_min
        self.right_min = right_min
        self.have_scan = True

    def publish_cmd(self, v, w):
        t = Twist()
        t.linear.x = float(v)
        t.angular.z = float(w)
        self.cmd_pub.publish(t)

    def cur_goal(self):
        return self.waypoints[self.wp_idx]

    def dist_to_goal(self, gx, gy):
        return math.hypot(gx - self.x, gy - self.y)
    
    def set_state(self, new_state: State):
        if new_state != self.state:
            self.get_logger().info(f"STATE: {self.state.name} -> {new_state.name}")
            self.state = new_state

    def loop(self):
        if not (self.have_odom and self.have_scan):
            self.publish_cmd(0.0, 0.0)
            return

        gx, gy = self.cur_goal()
        p = Point()
        p.x = float(gx)
        p.y = float(gy)
        p.z = 0.0
        self.goal_pub.publish(p)
        msg = Int32()
        msg.data = int(self.wp_idx)
        self.wp_pub.publish(msg)
        dist = self.dist_to_goal(gx, gy)

        # event: reached
        if dist < self.goal_tol and self.state != State.ARRIVED:
            self.set_state(State.ARRIVED)

        # event: obstacle while going to goal
        if self.front_min < self.obstacle_front and self.state in (State.ROTATE_TO_WP, State.DRIVE_TO_WP):
            self.avoid_dir = 1.0 if self.left_min > self.right_min else -1.0

            self.get_logger().info(
                f"OBSTACLE front={self.front_min:.2f} left={self.left_min:.2f} right={self.right_min:.2f} -> "
                f"turn={'LEFT' if self.avoid_dir > 0 else 'RIGHT'}"
            )

            self.until_time = self.get_clock().now() + Duration(seconds=self.avoid_turn_s)
            self.set_state(State.AVOID_TURN)

        # actions
        if self.state == State.INIT:
            self.publish_cmd(0.0, 0.0)
            self.set_state(State.ROTATE_TO_WP)
            return

        if self.state == State.ROTATE_TO_WP:
            desired = math.atan2(gy - self.y, gx - self.x)
            err = norm_angle(desired - self.yaw)
            if abs(err) < math.radians(10.0):
                self.publish_cmd(0.0, 0.0)
                self.set_state(State.DRIVE_TO_WP)
            else:
                self.publish_cmd(0.0, self.ang_speed * (1.0 if err > 0 else -1.0))
            return

        if self.state == State.DRIVE_TO_WP:
            desired = math.atan2(gy - self.y, gx - self.x)
            err = norm_angle(desired - self.yaw)

            w = max(-1.0, min(1.0, 2.0 * err)) * (self.ang_speed * 0.8)
            v = self.lin_speed
            if abs(err) > math.radians(35.0):
                v = 0.0
            self.publish_cmd(v, w)
            return

        if self.state == State.AVOID_TURN:
            if self.get_clock().now() < self.until_time:
                self.publish_cmd(0.0, self.avoid_dir * self.ang_speed)
            else:
                self.set_state(State.AVOID_FORWARD)
                self.until_time = self.get_clock().now() + Duration(seconds=self.avoid_fwd_s)
            return

        if self.state == State.AVOID_FORWARD:
            if self.get_clock().now() < self.until_time:
                self.publish_cmd(self.lin_speed * 0.9, 0.0)
            else:
                self.set_state(State.ROTATE_TO_WP)
                self.until_time = None
            return

        if self.state == State.ARRIVED:
            self.publish_cmd(0.0, 0.0)
            self.wp_idx = (self.wp_idx + 1) % len(self.waypoints)
            self.get_logger().info(f"SAMPE -> next goal {self.wp_idx}: {self.cur_goal()}")
            self.set_state(State.ROTATE_TO_WP)
            return


def main():
    rclpy.init()
    node = FSMPatrol()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.publish_cmd(0.0, 0.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


# waypoint tanpa melewati obstacle
# [-0.65, -0.57,  0.68, -0.57,  0.68, 0.64,  -0.68, 0.64]

# waypoint yang melewati obstacle
# [-1.84, -1.12,  1.16, -1.70,  1.65, 1.16,  -1.14, 1.66]