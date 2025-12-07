import os
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image


class LineFollowerNode(Node):
    def __init__(self):
        super().__init__("line_follower")

        self.declare_parameter("linear_speed", 0.2)
        self.declare_parameter("kp", 0.004)
        self.declare_parameter("kd", 0.002)
        self.declare_parameter("h_low1", 0)
        self.declare_parameter("h_high1", 10)
        self.declare_parameter("h_low2", 160)
        self.declare_parameter("h_high2", 180)
        self.declare_parameter("s_low", 80)
        self.declare_parameter("v_low", 80)
        self.declare_parameter("scan_row", 380)
        self.declare_parameter("lookahead_rows", 80)

        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.kp = float(self.get_parameter("kp").value)
        self.kd = float(self.get_parameter("kd").value)
        self.scan_row = int(self.get_parameter("scan_row").value)
        self.lookahead_rows = int(self.get_parameter("lookahead_rows").value)

        self.h_low1 = int(self.get_parameter("h_low1").value)
        self.h_high1 = int(self.get_parameter("h_high1").value)
        self.h_low2 = int(self.get_parameter("h_low2").value)
        self.h_high2 = int(self.get_parameter("h_high2").value)
        self.s_low = int(self.get_parameter("s_low").value)
        self.v_low = int(self.get_parameter("v_low").value)

        self.bridge = CvBridge()
        self.last_error = 0.0
        self.last_time = None
        self.error_history = deque(maxlen=100)

        log_dir = os.environ.get("LOG_DIR", "/logs")
        os.makedirs(log_dir, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        self.log_path = os.path.join(log_dir, f"line_follower_{ts}.log")
        self.log_file = open(self.log_path, "w")
        self.get_logger().info(f"Logging to {self.log_path}")

        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.image_sub = self.create_subscription(
            Image, "/camera/image_raw", self.image_callback, 10
        )

    def image_callback(self, msg: Image):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        height, width, _ = cv_img.shape

        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([self.h_low1, self.s_low, self.v_low])
        upper_red1 = np.array([self.h_high1, 255, 255])
        lower_red2 = np.array([self.h_low2, self.s_low, self.v_low])
        upper_red2 = np.array([self.h_high2, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # Focus on bottom rows to reduce noise
        scan_start = max(0, self.scan_row - self.lookahead_rows)
        roi = mask[scan_start : self.scan_row, :]
        moments = cv2.moments(roi)
        twist = Twist()
        line_found = False

        if moments["m00"] > 0:
            cx = int(moments["m10"] / moments["m00"])
            cy = int(moments["m01"] / moments["m00"])
            error = (width / 2) - cx
            now = self.get_clock().now().nanoseconds / 1e9
            dt = now - self.last_time if self.last_time else 0.0
            derivative = (error - self.last_error) / dt if dt > 1e-3 else 0.0
            angular = self.kp * error + self.kd * derivative

            twist.linear.x = self.linear_speed
            twist.angular.z = angular

            self.last_error = error
            self.last_time = now
            line_found = True

            self.log_file.write(
                f"{now:.3f},line_found,center={cx},error={error:.2f},angular={angular:.3f}\n"
            )
        else:
            # Stop if no line detected
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.log_file.write(
                f"{self.get_clock().now().nanoseconds/1e9:.3f},no_line\n"
            )

        self.cmd_pub.publish(twist)

        if not line_found:
            self.get_logger().warn("Line lost, stopping robot", throttle_duration_sec=2.0)

    def destroy_node(self):
        try:
            self.log_file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
