#!/usr/bin/env python3
"""
Example of using MoveIt 2 Servo to perform a circular motion.
`ros2 run pymoveit2 ex_servo.py`
"""

from threading import Thread
import numpy as np
import math, time
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5

positions = [
    [0.35, 0.10, 0.68],
    [-0.37, 0.12, 0.397],
    [0.194, -0.43, 0.701],
    [-0.37, 0.12, 0.397]
]

joints = [
    [math.radians(0), math.radians(-137), math.radians(138), math.radians(-180), math.radians(-91), math.radians(180)],
    [-0.03403341798766224, -1.2848632387872256, -1.8567441129914095, -3.185621281551551, -1.545888364367352, 3.1498768354918307],
    [math.radians(-90), math.radians(-138), math.radians(137), math.radians(-181), math.radians(-93), math.radians(180)],
]

class FrameListener(Node):

    def __init__(self):
        super().__init__('tf2_frame_listener')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Call on_timer function every second
        self.timer = self.create_timer(0.02, self.on_timer)

    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = ur5.end_effector_name()
        to_frame_rel = ur5.base_link_name()
        global curr_x,curr_y,curr_z,r,p,y

        # Look up for the transformation between target_frame and turtle2 frames
        # and send velocity commands for turtle2 to reach target_frame
        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
            curr_x = t.transform.translation.x
            curr_y = t.transform.translation.y
            curr_z = t.transform.translation.z
            # q = [t.transform.rotation.w , t.transform.rotation.x ,t.transform.rotation.y ,t.transform.rotation.z ]
            # r , p , y = R.from_quat(q).as_euler('xyz', degrees=False)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

class MoveItJointControl:
    def __init__(self):
        # Create a node for this example
        self.node = Node("moveit_joint_control")

        # Create callback group that allows execution of callbacks in parallel without restrictions
        self.callback_group = ReentrantCallbackGroup()

        # Create MoveIt 2 interface
        self.moveit2 = MoveIt2(
            node=self.node,
            joint_names=ur5.joint_names(),
            base_link_name=ur5.base_link_name(),
            end_effector_name=ur5.end_effector_name(),
            group_name=ur5.MOVE_GROUP_ARM,
            callback_group=self.callback_group,
        )

        # Spin the node in a background thread
        self.executor = rclpy.executors.MultiThreadedExecutor(2)
        self.executor.add_node(self.node)
        self.executor_thread = Thread(target=self.executor.spin, daemon=True, args=())
        self.executor_thread.start()

    def move_to_joint_positions(self, joint_positions):
        """
        Move the robot to the specified joint positions.

        :param joint_positions: List of target joint positions
        :type joint_positions: list of float
        """
        # Move to joint configuration
        self.node.get_logger().info(f"Moving to {{joint_positions: {joint_positions}}}")
        self.moveit2.move_to_configuration(joint_positions)
        self.moveit2.wait_until_executed()

    def shutdown(self):
        self.node.get_logger().info("Shutting down MoveItJointControl")
        self.executor.shutdown()
        rclpy.shutdown()

def main():
    try:
        moveit_control = MoveItJointControl()

        # Create node for the circular motion example
        node = Node("ex_servo")
        node1 = FrameListener()

        # Create publisher for sending twist commands
        twist_pub = node.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)

        def publish_twist(vx, vy, vz):
            twist_msg = TwistStamped()
            twist_msg.header.frame_id = ur5.base_link_name()
            twist_msg.header.stamp = node.get_clock().now().to_msg()
            twist_msg.twist.linear.x = vx
            twist_msg.twist.linear.y = vy
            twist_msg.twist.linear.z = vz
            twist_msg.twist.angular.x = 0.0
            twist_msg.twist.angular.y = 0.0
            twist_msg.twist.angular.z = 0.0
            twist_pub.publish(twist_msg)

        # Define the circular motion control logic
        def circular_motion():
            global i
            if i >= len(positions):
                moveit_control.shutdown()
                rclpy.shutdown()
                return

            target_pose = positions[i]
            joint_position = joints[i]
            
            # Move the robot to the specified joint positions
            moveit_control.move_to_joint_positions(joint_position)
            
            # Perform circular motion
            while True:
                # Implement your circular motion control logic here
                # Calculate vx, vy, vz based on the current and target poses

                # For example, here's a simple approach:
                marign = 0.02
                global curr_x, curr_y, curr_z   # Get the current end-effector position
                vx = (target_pose[0] - curr_x)  # Calculate linear velocity in x
                vy = (target_pose[1] - curr_y)  # Calculate linear velocity in y
                vz = (target_pose[2] - curr_z)  # Calculate linear velocity in z

                mag = math.sqrt( vx ** 2 + vy ** 2 + vz ** 2 )
                # Publish the twist command
                publish_twist(vx/mag, vy/mag, vz.mag)

                # Check if the target position is reached
                if abs(curr_x - target_pose[0]) < marign and abs(curr_y - target_pose[1]) < marign and abs(curr_z - target_pose[2]) < marign:
                    break

            i += 1

        # Create a timer for the circular motion control
        node.create_timer(0.02, circular_motion)

        # Spin the node in a background thread
        executor = rclpy.executors.MultiThreadedExecutor(2)
        executor.add_node(node1)
        executor.add_node(node)
        executor.spin()

if __name__ == "__main__":
    main()
