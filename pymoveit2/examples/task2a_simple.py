#!/usr/bin/python3

'''
Code to get all the frame names of the TF tree 
and the target poses for the ur_5 arm
Move the arm using Moveit servo and joint angles
'''

import yaml
from threading import Thread
import numpy as np
from math import cos, sin
import math, time
from copy import deepcopy
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_ros
from tf2_ros import TransformException
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from linkattacher_msgs.srv import AttachLink,DetachLink

# Joint angles list 
start = [math.radians(0),math.radians(-137),math.radians(138),math.radians(-180),math.radians(-91),math.radians(180)]
drop = [-0.03403341798766224, -1.2848632387872256, -1.8567441129914095, -3.185621281551551, -1.545888364367352, 3.1498768354918307]
right = [math.radians(-90),math.radians(-138),math.radians(137),math.radians(-181),math.radians(-93),math.radians(180)]
left = [math.radians(90),math.radians(-138),math.radians(137),math.radians(-181),math.radians(-93),math.radians(180)]

# Drop pose
drop_pose = [-0.37, 0.12, 0.397]

curr_x = -65536
curr_y = -65536
curr_z = -65536

frame_names = []

class TfFramesFinder(rclpy.node.Node):
    def __init__(self):
        super().__init__("tf2_frames_finder")
        self.get_logger().info("In class")

        self._tf_buffer = tf2_ros.buffer.Buffer()
        self._tf_listener = tf2_ros.transform_listener.TransformListener(self._tf_buffer, self)
        # time.sleep(5.0)

    def get_all_frames(self):
        _frames_dict = yaml.safe_load(self._tf_buffer.all_frames_as_yaml())
        if _frames_dict:
            # self.get_logger().info(yaml.dump(_frames_dict))
            global frame_names,target_pose
            for frame_name in _frames_dict.keys():
                if frame_name.startswith('obj_'):
                    frame_names.append(frame_name)
                #self.get_logger().info(frame_name)

        else:
            self.get_logger().warn("No frames collected")
    
    def get_pose(self,frame_name):
        t = self._tf_buffer.lookup_transform(ur5.base_link_name(),frame_name,rclpy.time.Time())
        return [t.transform.translation.x,t.transform.translation.y,t.transform.translation.z]

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
        global curr_x,curr_y,curr_z
        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
            curr_x = t.transform.translation.x
            curr_y = t.transform.translation.y
            curr_z = t.transform.translation.z
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

class ActivateGripper:
    def __init__(self):
        self.node = rclpy.create_node('gripper_Activate_node')
        self.gripper_control = self.node.create_client(AttachLink, '/GripperMagnetON')
        while not self.gripper_control.wait_for_service(timeout_sec=1.0):
            print('Waiting for service to be Activate available')
            self.node.get_logger().info('EEF service not available, waiting again...')

    def attach_link(self, box_name):
        req = AttachLink.Request()
        req.model1_name =  'box' + box_name    
        req.link1_name  = 'link'       
        req.model2_name =  'ur5'       
        req.link2_name  = 'wrist_3_link'
        future = self.gripper_control.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is not None:
            self.node.get_logger().info('Link attachment succeeded')
        else:
            self.node.get_logger().error('Failed to attach link')

class DeactivateGripper:
    def __init__(self):
        self.node = rclpy.create_node('gripper_deactivate_node')
        self.gripper_control = self.node.create_client(DetachLink, '/GripperMagnetOFF')
        while not self.gripper_control.wait_for_service(timeout_sec=1.0):
            print('Waiting for service to be Deactivate available')
            self.node.get_logger().info('EEF service not available, waiting again...')

    def detach_link(self, box_name):
        req = DetachLink.Request()
        req.model1_name =  'box' + box_name    
        req.link1_name  = 'link'       
        req.model2_name =  'ur5'       
        req.link2_name  = 'wrist_3_link'
        future = self.gripper_control.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is not None:
            self.node.get_logger().info('Link Detachment succeeded')
        else:
            self.node.get_logger().error('Failed to detach link')

def Servoing(target,marign):
    node = Node('Servo')
    node1 = FrameListener()
    callback_group = ReentrantCallbackGroup()
    twist_pub = node.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
    
    future = rclpy.Future()
    def func():
        if curr_x == -65536 and curr_y == -65536 and curr_z == -65536 :
            return
        curr_pose_x = target[0] - curr_x
        curr_pose_y = target[1] - curr_y
        curr_pose_z = target[2] - curr_z 
        print('Target: '+str(target)+' curr_pose: '+str(curr_x)+ ' ,'+str(curr_y)+ ' ,'+str(curr_z))
        mag = math.sqrt( curr_x ** 2 + curr_y ** 2 + curr_z ** 2 )
        vt = [ curr_pose_x / mag , curr_pose_y / mag , curr_pose_z / mag ]
        # if( math.sqrt((curr_pose_x)**2 + (curr_pose_y)**2 + (curr_pose_z)**2) < marign):
        if( abs(curr_x-target[0])<marign and abs(curr_y-target[1])<marign and abs(curr_z-target[2])<marign):
            twist_msg = TwistStamped()
            twist_msg.header.frame_id = ur5.base_link_name()
            twist_msg.header.stamp = node.get_clock().now().to_msg()
            twist_msg.twist.linear.x = 0.0
            twist_msg.twist.linear.y = 0.0
            twist_msg.twist.linear.z = 0.0
            twist_msg.twist.angular.x = 0.0
            twist_msg.twist.angular.y = 0.0
            twist_msg.twist.angular.z = 0.0 
            twist_pub.publish(twist_msg)
            print('Reached target')
            future.set_result(True)
            return
        twist_msg = TwistStamped()
        twist_msg.header.frame_id = ur5.base_link_name()
        twist_msg.header.stamp = node.get_clock().now().to_msg()
        twist_msg.twist.linear.x = vt[0]
        twist_msg.twist.linear.y = vt[1]
        twist_msg.twist.linear.z = vt[2]
        twist_msg.twist.angular.x = 0.0
        twist_msg.twist.angular.y = 0.0
        twist_msg.twist.angular.z = 0.0
        twist_pub.publish(twist_msg) 
    # Create timer for moving the end effector
    node.create_timer(0.02, func)
    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node1)
    executor.add_node(node)
    executor.spin_until_future_complete(future)
    # executor.shutdown()
    node.destroy_node()
    node1.destroy_node()
    print('DOne killing all nodes')
    time.sleep(1)

def main():
    rclpy.init()
    node_finder = TfFramesFinder()
    moveit_control = MoveItJointControl()

    try:
        # Spin the node for 2 seconds.
        for _ in range(200):
            rclpy.spin_once(node_finder)
            time.sleep(0.01)
        node_finder.get_all_frames()
    except KeyboardInterrupt as e:
        print("k/b interrupted")
    global frame_names,target_pose
    print(frame_names)

    marign = 0.45
    # Now making the ur_5 move
    for i in range(0,len(frame_names)):
        to_frame = frame_names[i]
        rclpy.spin_once(node_finder)
        target = node_finder.get_pose(to_frame) # Get the current pose of 'obj_<marker_id>'

        # Determining whether to turn in left direction or right direction to pick the current object
        jointList = []
        if target[1] >= 0.37 :
            jointList = [start,left]
        elif target[1] <= -0.28 :
            jointList = [start,right]
        else :
            jointList = [start]
        
        for joint in jointList:
            moveit_control.move_to_joint_positions(joint)
            time.sleep(2)

        # Servoing to boxes 
        Servoing(target,marign)
        print('Reached Target')
        time.sleep(1)

        # Activate the gripper
        activategrip = ActivateGripper()
        activategrip.attach_link(to_frame[4:])
        print('Attached Box')
        time.sleep(1)

        # Move to Drop location
        moveit_control.move_to_joint_positions(jointList[-1]) # MOving to the last joint positions to avoid collisions 
        time.sleep(2)
        moveit_control.move_to_joint_positions(drop) # Setting joints to reach Drop location
        time.sleep(2)

        Servoing(drop_pose,marign) # Precisely reaching drop location using Servoing

        # Moving the drop location further as there will be a box placed there previously
        drop_pose[0] -= 0.2

        # Deactivate the gripper
        deactivategrip = DeactivateGripper()
        deactivategrip.detach_link(to_frame[4:])
        print('Detached Box')
        time.sleep(1)
    rclpy.shutdown()
if __name__ == "__main__":
    main()