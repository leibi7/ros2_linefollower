import math
import time
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Bool
from gazebo_msgs.srv import SpawnEntity
from gazebo_msgs.msg import EntityState
from rclpy.qos import QoSProfile
from rclpy.callback_groups import ReentrantCallbackGroup
from ament_index_python.packages import get_package_share_directory


DINO_SDF = """
<?xml version='1.6'?>
<sdf version='1.6'>
  <model name='party_dino'>
    <static>false</static>
    <pose>0 0 0 0 0 0</pose>
    <link name='body'>
      <pose>0 0 0 0 0 0</pose>
      <collision name='collision'>
        <geometry>
          <box><size>0.6 0.3 0.4</size></box>
        </geometry>
        <pose>0 0 0.2 0 0 0</pose>
      </collision>
      <visual name='mesh'>
        <geometry>
          <mesh>
            <uri>model://party_dino/meshes/party_dino.obj</uri>
          </mesh>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
"""


class CelebrationNode(Node):
    def __init__(self):
        super().__init__("celebration")
        qos = QoSProfile(depth=5)
        self.callback_group = ReentrantCallbackGroup()
        self.goal_sub = self.create_subscription(
            Bool, "/goal_reached", self.goal_cb, qos, callback_group=self.callback_group
        )
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.state_pub = self.create_publisher(EntityState, "/gazebo/set_entity_state", 10)
        self.triggered = False
        self.dino_name = "party_dino"
        self.spawn_pose = Pose()
        # Dino is pre-spawned in the world at the goal
        self.spawn_pose.position.x = 7.0
        self.spawn_pose.position.y = -0.8
        self.spawn_pose.position.z = 1.5
        self.spawn_pose.orientation.w = 1.0
        world_share = get_package_share_directory("line_follower_world")

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

        # Animate backflip (rotate about Y)
        self.animate_backflip()

    def animate_backflip(self):
        state = EntityState()
        state.name = self.dino_name
        state.pose.position.x = self.spawn_pose.position.x
        state.pose.position.y = self.spawn_pose.position.y
        state.pose.position.z = self.spawn_pose.position.z
        state.reference_frame = "world"
        steps = 60
        duration = 2.4
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
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
