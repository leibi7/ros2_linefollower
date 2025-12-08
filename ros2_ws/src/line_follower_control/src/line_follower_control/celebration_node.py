import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from gazebo_msgs.srv import SpawnEntity
from gazebo_msgs.msg import EntityState
from rclpy.qos import QoSProfile


DINO_SDF = """
<?xml version='1.6'?>
<sdf version='1.6'>
  <model name='party_dino'>
    <static>true</static>
    <pose>7.2 -0.4 0 0 0 0</pose>
    <link name='body'>
      <visual name='torso'>
        <geometry>
          <cylinder><radius>0.12</radius><length>0.6</length></cylinder>
        </geometry>
        <material><ambient>0.1 0.8 0.1 1</ambient><diffuse>0.1 0.8 0.1 1</diffuse></material>
      </visual>
      <visual name='head'>
        <pose>0.35 0 0.08 0 0 0</pose>
        <geometry><sphere><radius>0.1</radius></sphere></geometry>
        <material><ambient>0.2 0.9 0.2 1</ambient><diffuse>0.2 0.9 0.2 1</diffuse></material>
      </visual>
      <visual name='tail'>
        <pose>-0.35 0 0.05 0 0 0</pose>
        <geometry><box><size>0.3 0.07 0.07</size></box></geometry>
        <material><ambient>0.05 0.6 0.05 1</ambient><diffuse>0.05 0.6 0.05 1</diffuse></material>
      </visual>
      <visual name='legs'>
        <pose>0 0 0 0 0 0</pose>
        <geometry><box><size>0.18 0.18 0.1</size></box></geometry>
        <material><ambient>0.1 0.7 0.1 1</ambient><diffuse>0.1 0.7 0.1 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>
"""


class CelebrationNode(Node):
    def __init__(self):
        super().__init__("celebration")
        qos = QoSProfile(depth=5)
        self.goal_sub = self.create_subscription(Bool, "/goal_reached", self.goal_cb, qos)
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.spawn_cli = self.create_client(SpawnEntity, "/spawn_entity")
        self.state_pub = self.create_publisher(EntityState, "/gazebo/set_entity_state", 10)
        self.triggered = False

    def goal_cb(self, msg: Bool):
        if not msg.data or self.triggered:
            return
        self.triggered = True
        self.get_logger().info("Goal reached! Stopping robot and spawning dino.")
        # Stop robot
        stop = Twist()
        for _ in range(5):
            self.cmd_pub.publish(stop)
            time.sleep(0.05)

        # Spawn dino (retry a few times)
        req = SpawnEntity.Request()
        req.name = "party_dino"
        req.xml = DINO_SDF
        req.robot_namespace = "dino"
        req.reference_frame = "world"
        for attempt in range(3):
            if not self.spawn_cli.service_is_ready():
                self.spawn_cli.wait_for_service(timeout_sec=5.0)
            try:
                future = self.spawn_cli.call_async(req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                if future.result() is not None:
                    self.get_logger().info("Dino spawned")
                    break
            except Exception as e:
                self.get_logger().warn(f"Spawn failed (try {attempt+1}): {e}")
            time.sleep(0.5)

        # Animate backflip (rotate about Y)
        self.animate_backflip()

    def animate_backflip(self):
        state = EntityState()
        state.name = "party_dino"
        state.pose.position.x = 7.2
        state.pose.position.y = -0.4
        state.pose.position.z = 0.0
        steps = 40
        duration = 1.6
        dt = duration / steps
        for i in range(steps + 1):
            angle = 2 * math.pi * (i / steps)
            # rotation about Y (pitch)
            qx, qy, qz, qw = self.quaternion_from_euler(0.0, angle, 0.0)
            state.pose.orientation.x = qx
            state.pose.orientation.y = qy
            state.pose.orientation.z = qz
            state.pose.orientation.w = qw
            self.state_pub.publish(state)
            time.sleep(dt)

    @staticmethod
    def quaternion_from_euler(roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return qx, qy, qz, qw


def main(args=None):
    rclpy.init(args=args)
    node = CelebrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
