#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import yaml
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseArray
from std_msgs.msg import Bool, String, Float32, Int16
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import time
import os
import numpy as np
from ament_index_python.packages import get_package_share_directory
from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import Pose
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class PoseTeller(Node):
    def __init__(self):
        super().__init__('pose_teller')
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.pose_callback, qos_profile)
        self.pose_pub = self.create_publisher(Pose, 'pose_dron', 10)

        self.pose_actual = [0.0]*6
        
    def pose_callback(self, msg: VehicleOdometry):
        try:
            self.pose_actual[0] = msg.position[0]
            self.pose_actual[1] = msg.position[1]
            self.pose_actual[2] = msg.position[2]
            quat = [msg.q[0], msg.q[1], msg.q[2], msg.q[3]]
            roll, pitch, yaw = euler_from_quaternion(quat)
            self.pose_actual[3] = roll
            self.pose_actual[4] = pitch
            self.pose_actual[5] = yaw

            # self.get_logger().info(f"X_local: {self.pose_actual[0]:.2f}, Y_local: {self.pose_actual[1]:.2f}, Z_local: {self.pose_actual[2]:.2f}")
            # self.get_logger().info(f"RX_local: {self.pose_actual[3]:.2f}, RY_local: {self.pose_actual[4]:.2f}, RZ_local: {self.pose_actual[5]:.2f}")

            dron_pose_msg = Pose()
            dron_pose_msg.position.x = float(self.pose_actual[0])
            dron_pose_msg.position.y = float(self.pose_actual[1])
            dron_pose_msg.position.z = float(self.pose_actual[2])
            dron_pose_msg.orientation.x = float(msg.q[0])
            dron_pose_msg.orientation.y = float(msg.q[1])
            dron_pose_msg.orientation.z = float(msg.q[2])
            dron_pose_msg.orientation.w = float(msg.q[3])
            self.pose_pub.publish(dron_pose_msg)

        except Exception as e:
            self.get_logger().error(f"Error en pose_callback: {e}")

def main():
    rclpy.init()
    rclpy.spin(PoseTeller())
    rclpy.shutdown()

if __name__ == "__main__":
    main()