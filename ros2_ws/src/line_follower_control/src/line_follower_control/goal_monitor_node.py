import os
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


class GoalMonitorNode(Node):
    def __init__(self):
        super().__init__("goal_monitor")
        self.declare_parameter("goal_x", 7.2)
        self.declare_parameter("goal_y", -0.4)
        self.declare_parameter("tolerance", 0.8)
        self.goal_x = float(self.get_parameter("goal_x").value)
        self.goal_y = float(self.get_parameter("goal_y").value)
        self.tolerance = float(self.get_parameter("tolerance").value)
        self.goal_reached = False

        log_dir = os.environ.get("LOG_DIR", "/logs")
        os.makedirs(log_dir, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        self.log_path = os.path.join(log_dir, f"goal_monitor_{ts}.log")
        self.log_file = open(self.log_path, "w")

        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.goal_pub = self.create_publisher(Bool, "/goal_reached", 1)
        self.get_logger().info(
            f"Goal monitor active at ({self.goal_x:.2f}, {self.goal_y:.2f}) tol={self.tolerance:.2f}"
        )

    def odom_cb(self, msg: Odometry):
        if self.goal_reached:
            return
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        dist_sq = (x - self.goal_x) ** 2 + (y - self.goal_y) ** 2
        if dist_sq <= self.tolerance**2:
            self.goal_reached = True
            self.goal_pub.publish(Bool(data=True))
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.log_file.write(f"{stamp:.3f},goal_reached\n")
            self.log_file.flush()
            self.get_logger().info("GOAL reached!")
        else:
            # Occasional debug
            if int(time.time()) % 2 == 0:
                self.get_logger().debug(
                    f"dist={dist_sq**0.5:.2f} tol={self.tolerance}",
                    throttle_duration_sec=2.0,
                )

    def destroy_node(self):
        try:
            self.log_file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GoalMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
