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

    def move_to_pose(self, position, roll, pitch, yaw, cartesian):
        self.node.get_logger().info(
            f"Moving to {{position: {list(position)}, rpy: [{roll}, {pitch}, {yaw}]}}"
        )

        quaternion = self.rpy_to_quaternion(roll, pitch, yaw)
        self.moveit2.move_to_pose(position=position, quat_xyzw=quaternion, cartesian=cartesian)
        self.moveit2.wait_until_executed()

    def add_mesh_to_planning_scene(self, mesh_file, pose):
        # Create a CollisionObject
        collision_object = CollisionObject()
        collision_object.id = os.path.basename(mesh_file)  # Use the file name as the ID

        # Define the mesh (load mesh from file)
        mesh = Mesh()
        mesh.filename = mesh_file
        collision_object.meshes.append(mesh)

        # Set the pose of the object
        collision_object.mesh_poses.append(pose)

        # Set the object type (in this case, it's a mesh)
        collision_object.operation = CollisionObject.ADD

        # Add the object to the planning scene
        self.moveit2.scene.add_collision_objects([collision_object])

    def rpy_to_quaternion(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cy * cp * cr + sy * sp * sr
        qx = cy * cp * sr - sy * sp * cr
        qy = sy * cp * sr + cy * sp * cr
        qz = sy * cp * cr - cy * sp * sr

        return [qx, qy, qz, qw]

def main():
    rclpy.init()
    node = Node("task1b")

    arm_controller = RoboticArmController(node)

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    pi = math.pi

    # Define the positions and orientations in roll, pitch, and yaw format
    p1 = [0.35, 0.10, 0.68]
    rpy1 = [0.0, pi/2, 0.0]  
    d1 = [-0.37, 0.12, 0.397]
    rpy2 = [0.0, -pi/2, 0.0] 
    p2 = [0.194, -0.43, 0.701]
    rpy3 = [pi/2, 0.0, 0.0] 
    d2 = [-0.37, 0.12, 0.397]
    rpy4 = [0.0, -pi/2, 0.0]  

    # Move to positions in the specified order with specified orientations
    arm_controller.move_to_pose(p1, *rpy1, True)
    time.sleep(0)  # Sleep for stability (adjust as needed)
    arm_controller.move_to_pose(d1, *rpy2, True)
    time.sleep(0)
    arm_controller.move_to_pose(p2, *rpy3, True)
    time.sleep(0)
    arm_controller.move_to_pose(d2, *rpy4, True)

    # Paths to mesh files
    DEFAULT_RACK_MESH = os.path.join(
        os.path.dirname(os.path.realpath(__file__)), "assets", "rack.stl"
    )
    DEFAULT_BOX_MESH = os.path.join(
        os.path.dirname(os.path.realpath(__file__)), "assets", "box.stl"
    )

    # Positions and orientations
    positions = [[0.25, -0.55, 0.53], [0.25, -0.55, 0.17], [0.46, 0.05, 0.53]]
    quat_xyzws = [[0, 0.0011262, 0.7071059, 0.7071068],
                  [0, 0.0011262, 0.7071059, 0.7071068],
                  [0.0007963, 0.0007963, 0.9999994, 6e-7]]

    # Add box meshes to the planning scene
    for i, pose in enumerate(positions):
        arm_controller.add_mesh_to_planning_scene(DEFAULT_BOX_MESH, pose)

    # Positions and orientations for rack meshes
    positions = [[0.56, 0.06, -0.58], [0.25, -0.64, -0.58], [0.25, 0.75, -0.58]]
    quat_xyzws = [[0.0, 0.0, 0.0, 0.0],
                  [0.0, 0.0, 0.7071068, 0.7071068],
                  [0.0, 0.0, -0.7071068, 0.7071068]]

    # Add rack meshes to the planning scene
    for i, pose in enumerate(positions):
        arm_controller.add_mesh_to_planning_scene(DEFAULT_RACK_MESH, pose)

    rclpy.shutdown()
    exit(0)

if __name__ == "__main__":
    main()