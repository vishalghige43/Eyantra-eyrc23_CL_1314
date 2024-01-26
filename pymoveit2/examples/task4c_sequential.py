#!/usr/bin/python3

# Team ID:          [ 1314 ]
# Author List:		[ Swaraj Zende, Vishal Ghige, Vipul Pardeshi, Vishal Singh ]
# Filename:		    task4c.py

'''
Dock into the rack specified in yaml file
Place the rack at arm pose 1(AP1)
Get all the frame names of the TF tree 
and the target poses for the ur_5 arm
Move the arm using Moveit servo and joint angles
Place the box on the table
'''

################### IMPORT MODULES #######################
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
from geometry_msgs.msg import TwistStamped,PoseStamped,Twist
from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.callback_groups import ReentrantCallbackGroup
from linkattacher_msgs.srv import AttachLink,DetachLink
from ebot_docking.srv import DockSw 
from tf_transformations import euler_from_quaternion,quaternion_from_euler
import time
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from linkattacher_msgs.srv import AttachLink,DetachLink
from std_srvs.srv import Trigger

################### GLOBAL VARIABLES #######################

# Joint angles list 
start = [math.radians(0),math.radians(-137),math.radians(138),math.radians(-180),math.radians(-91),math.radians(180)]
drop = [-0.03403341798766224, -1.2848632387872256, -1.8567441129914095, -3.185621281551551, -1.545888364367352, 3.1498768354918307]
right = [math.radians(-90),math.radians(-138),math.radians(137),math.radians(-181),math.radians(-93),math.radians(180)]
left = [math.radians(90),math.radians(-138),math.radians(137),math.radians(-181),math.radians(-93),math.radians(180)]
center_right = [math.radians(-35),math.radians(-138),math.radians(137),math.radians(-181),math.radians(-35),math.radians(180)]
center_left = [math.radians(35),math.radians(-138),math.radians(137),math.radians(-181),math.radians(215),math.radians(180)]

# Drop pose
drop_pose = [-0.57, 0.12, 0.237]

drop_offset=[[0.0 , +0.15 , 0.0 ],
             [0.0 , -0.15 , 0.0 ],
             [0.0 ,  0.0  , 0.2 ]]

curr_x = -65536
curr_y = -65536
curr_z = -65536
curr_rot = -65536

robot_pose_x = 0.0
robot_pose_y = 0.0

frame_names = []

# Anonymous count
anon_cnt = 0

# List of Arm poses
AP_Prepose = [[1.61 , -3.255, -1.57],
                [0.5 , -2.355 , 3.14],
                [1.61, -0.75, 1.57]]

AP = [[1.61 , -3.0, -1.57],
      [0.85 , -2.365 , 3.14],
      [1.61, -1.75, 1.57]]

AP_Postpose = [[1.61 , -4.255, -1.57],
                [0.0 , -2.355 , 3.14],
                [1.61, -0.75, 1.57]]

################### CLASS DEFINITION #######################

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
        _,_,yaw = euler_from_quaternion([t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w])
        yaw = math.degrees(yaw)
        return [t.transform.translation.x,t.transform.translation.y,t.transform.translation.z,yaw]

##############################################################

class FrameListener(Node):

    def __init__(self):
        global anon_cnt
        anon_cnt = anon_cnt + 1
        super().__init__('tf2_frame_listener'+str(anon_cnt))

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
            _,_,curr_rot = euler_from_quaternion([t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w])
            curr_rot = math.degrees(curr_rot)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

##############################################################

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

##############################################################

class ActivateGripper:
    def __init__(self):
        self.node = rclpy.create_node('gripper_Activate_node')
        self.gripper_control = self.node.create_client(AttachLink, '/GripperMagnetON')
        while not self.gripper_control.wait_for_service(timeout_sec=1.0):
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

##############################################################

class DeactivateGripper:
    def __init__(self):
        self.node = rclpy.create_node('gripper_deactivate_node')
        self.gripper_control = self.node.create_client(DetachLink, '/GripperMagnetOFF')
        while not self.gripper_control.wait_for_service(timeout_sec=1.0):
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

####################  SERVOING FUNCTION  #############################
def make_servoing_service_call():
    # Initialize the ROS node
    node = rclpy.create_node('service_call_node')

    # Create a client for the service
    client = node.create_client(Trigger, '/servo_node/start_servo')

    # Wait for the service to be available
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().info('Service not available')
        return

    # Create the request
    request = Trigger.Request()

    # Call the service
    future = client.call_async(request)

    # Wait for the service call to complete
    rclpy.spin_until_future_complete(node, future)

    # Check if the service call was successful
    if future.result() is not None:
        node.get_logger().info('Servoing Service call succeeded')
    else:
        node.get_logger().info('Servoing Service call failed')

    # Shutdown the node
    node.destroy_node()

###############################################################################
def Servoing(target,marign):
    global anon_cnt
    anon_cnt = anon_cnt + 1
    node = Node('Servo'+str(anon_cnt))
    node1 = FrameListener()
    callback_group = ReentrantCallbackGroup()
    twist_pub = node.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
    flag = False
    
    while not flag:
        rclpy.spin_once(node1)
        if curr_x == -65536 and curr_y == -65536 and curr_z == -65536 :
            continue
        curr_pose_x = target[0] - curr_x
        curr_pose_y = target[1] - curr_y
        curr_pose_z = target[2] - curr_z 
        mag = math.sqrt( curr_x ** 2 + curr_y ** 2 + curr_z ** 2 )
        vt = [ curr_pose_x / mag , curr_pose_y / mag , curr_pose_z / mag ]
        # print('target : '+str(target))
        # print('curr_pose : ' +str(curr_pose_x)+' '+str(curr_pose_y)+' '+str(curr_pose_z))
        # print('current mag : '+str(mag))
        if( abs(curr_pose_x) <= marign and abs(curr_pose_y) <= marign and abs(curr_pose_z) <= marign):
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
            flag = True
            break
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
    
    node.destroy_node()
    node1.destroy_node()
    time.sleep(1)

########################################################################################
    
def yaw_Servoing(target,marign):
    global anon_cnt
    anon_cnt = anon_cnt + 1
    node = Node('Servo'+str(anon_cnt))
    node1 = FrameListener()
    callback_group = ReentrantCallbackGroup()
    twist_pub = node.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
    
    future = rclpy.Future()
    def func():
        if curr_rot == -65536 :
            return
        if( abs(target-curr_rot)<marign):
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
            future.set_result(True)
            return
        twist_msg = TwistStamped()
        twist_msg.header.frame_id = ur5.base_link_name()
        twist_msg.header.stamp = node.get_clock().now().to_msg()
        twist_msg.twist.linear.x = 0.0
        twist_msg.twist.linear.y = 0.0
        twist_msg.twist.linear.z = 0.0
        twist_msg.twist.angular.x = 0.0
        twist_msg.twist.angular.y = 0.0
        twist_msg.twist.angular.z = 0.5 if target - curr_rot > 0 else -0.5
        twist_pub.publish(twist_msg) 
    # Create timer for moving the end effector
    node.create_timer(0.02, func)
    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node1)
    executor.add_node(node)
    executor.spin_until_future_complete(future)
    node.destroy_node()
    node1.destroy_node()
    time.sleep(1)

##############################################################
class LinkAttacher(Node):
    def __init__(self):
        super().__init__('Link_Attacher')
        self.link_attach_cli = self.create_client(AttachLink, '/ATTACH_LINK')

        while not self.link_attach_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Link attacher service not available, waiting again...')

    def attach_link(self, model2_name):
        req = AttachLink.Request()
        req.model1_name =  'ebot'     
        req.link1_name  = 'ebot_base_link'       
        req.model2_name =  model2_name    
        req.link2_name  = 'link' 

        self.link_attach_cli.call_async(req)

##############################################################

class LinkDetacher(Node):
    def __init__(self):
        super().__init__('Link_Detacher')
        self.link_detach_cli = self.create_client(DetachLink, '/DETACH_LINK')

        while not self.link_detach_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Link detacher service not available, waiting again...')

    def detach_link(self, model2_name):
        req = DetachLink.Request()
        req.model1_name =  'ebot'     
        req.link1_name  = 'ebot_base_link'       
        req.model2_name =  model2_name    
        req.link2_name  = 'link' 

        self.link_detach_cli.call_async(req)

############################################################

class Docking_Client(Node):

    def __init__(self):
        super().__init__('Docking_Client')
        self.cli = self.create_client(DockSw, '/dock_control')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = DockSw.Request()

    def send_request(self, angle, distance = 0.0 ,linear = False, angular=False):
        self.req.distance = distance
        self.req.orientation = angle
        self.req.linear_dock = linear
        self.req.orientation_dock = angular
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

############################################################

class Arm_Docking(Node):
    def __init__(self):
        super().__init__('Arm_pose_Docking')
        # Subscribe to odometry data for robot pose information
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)
    
    def odometry_callback(self,msg):
        # Extract and update robot pose information from odometry message
        global robot_pose_x,robot_pose_y
        robot_pose_x = msg.pose.pose.position.x
        robot_pose_y = msg.pose.pose.position.y

def ArmDocking(target,left_or_right,marign):
    global anon_cnt
    anon_cnt = anon_cnt + 1
    node = Node('Arm_docking'+str(anon_cnt))
    node1 = Arm_Docking()
    callback_group = ReentrantCallbackGroup()
    twist_pub = node.create_publisher(Twist, "/cmd_vel", 10)
    flag = False
    
    while not flag:
        rclpy.spin_once(node1)
        global robot_pose_x,robot_pose_y
        if robot_pose_x == 0 and robot_pose_y == 0 :
            continue

        if (left_or_right and abs(target - robot_pose_y)<= marign) or (not left_or_right and abs(target - robot_pose_x) <= marign):    
            twist_msg = Twist()
            twist_msg.linear.x = 0.0
            twist_msg.linear.y = 0.0
            twist_msg.linear.z = 0.0
            twist_msg.angular.x = 0.0
            twist_msg.angular.y = 0.0
            twist_msg.angular.z = 0.0 
            twist_pub.publish(twist_msg)
            flag = True
            break

        print(f'{left_or_right}  target={target} x = {robot_pose_x}  y = {robot_pose_y}')        
        twist_msg = Twist()
        twist_msg.linear.x = -0.5
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0
        twist_pub.publish(twist_msg) 
    
    node.destroy_node()
    node1.destroy_node()
    time.sleep(1)
################### MAIN FUNCTION #######################
def main():
    rclpy.init()

    # Make servoing service call to start servoing
    make_servoing_service_call()

    navigator = BasicNavigator()

    # Set initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.x = 0.0
    initial_pose.pose.orientation.y = 0.0
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()
    navigator.changeMap('/home/saz/colcon_ws/src/ebot_nav2/maps/map.yaml') # Specifying the map to load

    # READING from config.yaml and determing which rack to pick and its pick position
    file_path = '/home/saz/colcon_ws/src/pymoveit2/examples/config.yaml'
    with open(file_path, "r") as file:
        data = yaml.safe_load(file)

    # Accessing the data
    position_data = data.get("position", [])
    rack_id_list = data.get("package_id", [])

    i = 0 # for drop pose offset
    k = 0 # for Arm pose or rack end pose

    for rack_id in rack_id_list:
        rack_id = int(rack_id)
        rack_pose = []
        for item in position_data:
            for rack, coordinates in item.items():
                if int(rack[4:]) == rack_id:
                    rack_pose = coordinates
                    rack_pose[0] = float(rack_pose[0])
                    rack_pose[1] = float(rack_pose[1])
                    rack_pose[2] = float(rack_pose[2])

        # Calculate pre Rack pose
        preRackPose = rack_pose
        if(rack_pose[2] == 0.0 or rack_pose[2] == 3.14):
            preRackPose[0] = rack_pose[0] + (-0.5  if rack_pose[0]>0 else  0.5)
            preRackPose[1] = rack_pose[1] + ( 0.25 if rack_pose[1]>0 else -0.25)
        elif(rack_pose[2] == 1.57 or rack_pose[2] == -1.57):
            preRackPose[0] = rack_pose[0] + ( 0.25 if rack_pose[0]>0 else -0.25)
            preRackPose[1] = rack_pose[1] + (-0.5  if rack_pose[1]>0 else  0.5) 
        # Going to specified rack from config.yaml file 
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = preRackPose[0] 
        goal_pose.pose.position.y = preRackPose[1] 
        orientations = quaternion_from_euler(0.0, 0.0, float(rack_pose[2]))
        goal_pose.pose.orientation.x = orientations[0]
        goal_pose.pose.orientation.y = orientations[1]
        goal_pose.pose.orientation.z = orientations[2]
        goal_pose.pose.orientation.w = orientations[3]

        navigator.goToPose(goal_pose) # Use navigator node to achieve the goal_pose

        j = 0
        while not navigator.isTaskComplete():
            j = j + 1
            feedback = navigator.getFeedback()
            if feedback and j % 5 == 0:
                print('Estimated time of arrival: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                      + ' seconds.')

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

        # Call the Docking service here
        dockClient = Docking_Client()
        dockClient.send_request(angle = float(rack_pose[2]) , distance = 0.01)

        # Call the Attach Link Service call here
        # Attach the rack to the eBot 
        rack_attach = LinkAttacher()
        rack_attach.attach_link('rack'+str(rack_id))

        # Go to AP1
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = AP_Prepose[k][0]
        goal_pose.pose.position.y = AP_Prepose[k][1]
        orientations = quaternion_from_euler(0.0, 0.0, float(AP_Prepose[k][2]))
        goal_pose.pose.orientation.x = orientations[0]
        goal_pose.pose.orientation.y = orientations[1]
        goal_pose.pose.orientation.z = orientations[2]
        goal_pose.pose.orientation.w = orientations[3]

        navigator.goToPose(goal_pose) # Use navigator node to achieve the goal_pose

        j = 0
        while not navigator.isTaskComplete():
            j = j + 1
            feedback = navigator.getFeedback()
            if feedback and j % 5 == 0:
                print('Estimated time of arrival: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                      + ' seconds.')

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

        # Call the Docking service here
        dockClient.send_request(angle = float(AP_Prepose[k][2]), distance = 0.0 , linear=True ,angular=False )

        # Go closer to arm
        if AP[k][2] == 3.14:
            ArmDocking(AP[k][0],left_or_right=False,marign=0.3)
        else:
            ArmDocking(AP[k][1],left_or_right=True,marign=0.3)

        # Call the Detach Link Service call here
        # Detach the rack from eBot
        rack_detach = LinkDetacher()
        rack_detach.detach_link('rack'+str(rack_id))

        # Go to origin Position
        # Get out of the rack 
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = AP_Postpose[k][0]
        goal_pose.pose.position.y = AP_Postpose[k][1]
        # orientations = quaternion_from_euler(0.0, 0.0, 0.0)
        goal_pose.pose.orientation.x = orientations[0]
        goal_pose.pose.orientation.y = orientations[1]
        goal_pose.pose.orientation.z = orientations[2]
        goal_pose.pose.orientation.w = orientations[3]

        navigator.goToPose(goal_pose) # Use navigator node to achieve the goal_pose

        j = 0
        while not navigator.isTaskComplete():
            j = j + 1
            feedback = navigator.getFeedback()
            if feedback and j % 5 == 0:
                print('Estimated time of arrival: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                      + ' seconds.')

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

        k = k + 1

    ### Done with ebot work
    print('Done with eBot Work')
    time.sleep(1)

    node_finder = TfFramesFinder()
    moveit_control = MoveItJointControl()
    activategrip = ActivateGripper()
    deactivategrip = DeactivateGripper()
    ### Look for the frame name for the boxes
    # try:
    #     # Spin the node for 2 seconds.
    #     for _ in range(300):
    #         rclpy.spin_once(node_finder)
    #         time.sleep(0.01)
    #     node_finder.get_all_frames()
    # except KeyboardInterrupt as e:
    #     print("k/b interrupted")

    for _ in range(500):
        rclpy.spin_once(node_finder)
        time.sleep(0.01)
    node_finder.get_all_frames()
    global frame_names,target_pose
    print(frame_names)

    marign = 0.01185
    # Distance from boxes
    dist_for_pick = 0.05 # For Pre Pick Pose
    dist_for_drop = 0.12 # For Pre Drop Pose
    preDropOffset = 0.01 # Pre Drop pose offset in Z axis

    # Now making the ur_5 move
    for j in range(0,len(frame_names)):
        to_frame = frame_names[j]
        rclpy.spin_once(node_finder)
        target = node_finder.get_pose(to_frame) # Get the current pose of 'obj_<marker_id>'
        print()
        print(target)

        # Determining whether to turn in left direction or right direction to pick the current object
        jointList = []
        prePickPose = []
        preDropPose = []

        if target[1] >= 0.37 : # Left
            print('Going to Left')
            prePickPose = [target[0],target[1] - dist_for_pick,target[2]]
            preDropPose = [target[0],target[1] - dist_for_drop,target[2] + preDropOffset]
            jointList = [start,left]
        elif target[1] <= -0.28 : # Right
            print('Going to Right')
            prePickPose = [target[0],target[1] + dist_for_pick,target[2]]
            preDropPose = [target[0],target[1] + dist_for_drop,target[2] + preDropOffset]
            jointList = [start,right]
        else : # Center
            prePickPose = [target[0] - dist_for_pick,target[1],target[2]]
            preDropPose = [target[0] - dist_for_drop,target[1],target[2] + preDropOffset]
            print('Going Center')
            jointList = [start]

        for joint in jointList:
            moveit_control.move_to_joint_positions(joint)
            time.sleep(2)

        # Servoing to prePose of boxes
        #Servoing(prePickPose,marign)
        print('\n Reached PrePickPose \n')

        # Align the Yaw of the boxes
        # yaw_Servoing(target[3],4.0)
        print('\n Yaw Aligned \n')

        # Servoing to boxes 
        Servoing(target,marign)
        print('\n Reached Target \n')
        time.sleep(0.5)

        # Activate the gripper
        activategrip.attach_link(to_frame[4:])
        time.sleep(0.5)

        # Servoing back to prePose of boxes
        Servoing(preDropPose,marign)
        print('\n Reached PreDropPose \n')

        # Move to Drop location
        moveit_control.move_to_joint_positions(jointList[-1]) # Muving to the last joint positions to avoid collisions 
        time.sleep(1)
        moveit_control.move_to_joint_positions(drop) # Setting joints to reach Drop location
        time.sleep(1)

        # Moving the drop location further as there will be a box placed there previously
        Servoing([drop_pose[0]+drop_offset[i][0],drop_pose[1]+drop_offset[i][1],drop_pose[2]+drop_offset[i][2]],marign) # Precisely reaching drop location using Servoing
        time.sleep(0.5)
        i = i + 1

        # Deactivate the gripper
        deactivategrip.detach_link(to_frame[4:])
        time.sleep(0.5)

    navigator.lifecycleShutdown()
    dockClient.destroy_node()
    rack_attach.destroy_node()
    rack_detach.destroy_node()
    rclpy.shutdown()
    exit(0)


if __name__ == '__main__':
    main()
