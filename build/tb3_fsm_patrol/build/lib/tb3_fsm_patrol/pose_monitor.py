#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped

def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class PoseMonitor(Node):
    def __init__(self):
        super().__init__("pose_monitor")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("out_topic", "/fsm/pose")
        self.declare_parameter("rate_hz", 10.0)

        odom_topic = self.get_parameter("odom_topic").value
        out_topic  = self.get_parameter("out_topic").value
        self.rate  = float(self.get_parameter("rate_hz").value)

        self.pub = self.create_publisher(PointStamped, out_topic, 10)
        self.sub = self.create_subscription(Odometry, odom_topic, self.on_odom, 10)

        self.last_msg = None
        self.timer = self.create_timer(1.0 / self.rate, self.tick)

        self.get_logger().info(f"PoseMonitor: {odom_topic} -> {out_topic} @ {self.rate} Hz")

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = yaw_from_quat(q)

        ps = PointStamped()
        ps.header = msg.header
        ps.point.x = float(p.x)
        ps.point.y = float(p.y)
        ps.point.z = float(yaw)  # yaw kita taruh di z biar gampang
        self.last_msg = ps

    def tick(self):
        if self.last_msg is not None:
            self.pub.publish(self.last_msg)

def main():
    rclpy.init()
    node = PoseMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

# untuk jalankan

# Terminal 1 (Gazebo + TB3 spawn, tanpa FSM)
# source ~/tb3_ws/install/setup.bash
# export TURTLEBOT3_MODEL=burger
# ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2 (pose_monitor saja):
# source ~/tb3_ws/install/setup.bash
# ros2 run tb3_fsm_patrol pose_monitor

# Terminal 3
# source ~/tb3_ws/install/setup.bash
# ros2 topic echo /fsm/pose

# kalau sudah siap, kill semua gazebo/launch
# pkill -f gzserver
# pkill -f gzclient
# pkill -f robot_state_publisher
