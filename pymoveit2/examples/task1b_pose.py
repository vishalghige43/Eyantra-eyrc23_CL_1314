#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from threading import Thread
from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5
from moveit_msgs.msg import CollisionObject  # Add this import
from shape_msgs.msg import Mesh  # Add this import
import time
import math
import os

class RoboticArmController:
    def __init__(self, node):
        self.node = node
        self.node.declare_parameter("position", [0.35, 0.10, 0.68])
        self.node.declare_parameter("roll", 0.0)
        self.node.declare_parameter("pitch", 0.0)
        self.node.declare_parameter("yaw", 0.0)
        self.node.declare_parameter("cartesian", False)
        self.callback_group = ReentrantCallbackGroup()

        self.moveit2 = MoveIt2(
            node=self.node,
            joint_names=ur5.joint_names(),
            base_link_name=ur5.base_link_name(),
            end_effector_name=ur5.end_effector_name(),
            group_name=ur5.MOVE_GROUP_ARM,
            callback_group=self.callback_group,
        )

def main():
    rclpy.init()
    node = Node("task1b")

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    moveit2 = MoveIt2(
        node=node,
        joint_names=ur5.joint_names(),
        base_link_name=ur5.base_link_name(),
        end_effector_name=ur5.end_effector_name(),
        group_name=ur5.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    positions = [[0.35, 0.10, 0.68] , [-0.37, 0.12, 0.397] , [0.194, -0.43, 0.701] , [-0.37, 0.12, 0.397]]

    # Move to positions in the specified order with specified orientations
    for i in range(4):
        moveit2.move_to_pose(position=positions[i], quat_xyzw=[0.0 , 0.0 , 0.0 , 1.0 ], cartesian=True)
        moveit2.wait_until_executed()

    rclpy.shutdown()
    exit(0)

if __name__ == "__main__":
    main()