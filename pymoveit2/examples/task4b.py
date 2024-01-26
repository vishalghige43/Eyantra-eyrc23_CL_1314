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
from ur_msgs.srv import SetIO
from controller_manager_msgs.srv import SwitchController # module call
from tf_transformations import euler_from_quaternion
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

# Joint angles list 
# start = [math.radians(0),math.radians(-137),math.radians(138),math.radians(-180),math.radians(-91),math.radians(180)]
# drop = [-0.03403341798766224, -1.2848632387872256, -1.8567441129914095, -3.185621281551551, -1.545888364367352, 3.1498768354918307]
start = [0.0, -2.398, 2.43, -3.15, -1.58, 3.15]
drop = [-0.027, -1.803, -1.3658, -3.039, -1.52, 3.15]
right = [math.radians(-90),math.radians(-138),math.radians(137),math.radians(-181),math.radians(-93),math.radians(180)]
left = [math.radians(90),math.radians(-138),math.radians(137),math.radians(-181),math.radians(-93),math.radians(180)]
center_right = [math.radians(-35),math.radians(-138),math.radians(137),math.radians(-181),math.radians(-35),math.radians(180)]
center_left = [math.radians(35),math.radians(-138),math.radians(137),math.radians(-181),math.radians(215),math.radians(180)]
center_back = [math.radians(0),math.radians(-164),math.radians(153),math.radians(-165),math.radians(-89),math.radians(181)]

# Drop pose
drop_pose = [-0.57, 0.12, 0.237]

drop_offset=[[0.0 , +0.15 , 0.0 ],
             [0.0 , -0.15 , 0.0 ],
             [0.0 ,  0.0  , 0.2 ]]

curr_x = -65536
curr_y = -65536
curr_z = -65536
curr_rot = -65536

frame_names = []

# Anonymous count
anon_cnt = 0

positions_array = []

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
            self.get_logger().info(yaml.dump(_frames_dict))
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
        global curr_x,curr_y,curr_z,curr_rot
        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
            curr_x = t.transform.translation.x
            curr_y = t.transform.translation.y
            curr_z = t.transform.translation.z
            _,_,curr_rot = euler_from_quaternion([t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w])
            # print()
            # print(curr_rot)
            # print()
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

class GripperController(Node):
    def __init__(self):
        super().__init__('Gripper_Controller')
        self.gripper_control = self.create_client(SetIO, '/io_and_status_controller/set_io')
        while not self.gripper_control.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('EEF Tool service not available, waiting again...')

    def gripper_call(self, state):
        req         = SetIO.Request()
        req.fun     = 1
        req.pin     = 16
        req.state   = float(state)
        self.gripper_control.call_async(req)
        return state

class ContextSwitcher(Node):
    def __init__(self):
        super().__init__('Context_Switcher')
        self.__contolMSwitch = self.create_client(SwitchController, "/controller_manager/switch_controller")

    def activateMoveit(self):
        # Parameters to switch controller
        switchParam = SwitchController.Request()
        switchParam.activate_controllers = ["scaled_joint_trajectory_controller"] # for normal use of moveit
        switchParam.deactivate_controllers = ["forward_position_controller"] # for servoing
        switchParam.strictness = 2
        switchParam.start_asap = False

        # calling control manager service after checking its availability
        while not self.__contolMSwitch.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(f"Service control Manager is not yet available...")
        self.__contolMSwitch.call_async(switchParam)
        print("[CM]: Switching Complete")
    
    def activateServoing(self):
        # Parameters to switch controller
        switchParam = SwitchController.Request()
        switchParam.deactivate_controllers = ["scaled_joint_trajectory_controller"] # for normal use of moveit
        switchParam.activate_controllers = ["forward_position_controller"] # for servoing
        switchParam.strictness = 2
        switchParam.start_asap = False

        # calling control manager service after checking its availability
        while not self.__contolMSwitch.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(f"Service control Manager is not yet available...")
        self.__contolMSwitch.call_async(switchParam)
        print("[CM]: Switching Complete")

class StateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')
        self.subscription = self.create_subscription(
            JointState,  # Specify the message type
            '/joint_states',
            self.state_callback,
            10)

    def state_callback(self, msg):
        print('Enterd callback')
        global positions_array
        positions_array = list(msg.position)

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
        node.get_logger().info('Service call succeeded')
    else:
        node.get_logger().info('Service call failed')

    # Shutdown the node
    node.destroy_node()

def Servoing(target,marign):
    global anon_cnt
    anon_cnt = anon_cnt + 1
    node = Node('Servo'+str(anon_cnt))
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
        mag = math.sqrt( curr_x ** 2 + curr_y ** 2 + curr_z ** 2 )
        vt = [ curr_pose_x / mag , curr_pose_y / mag , curr_pose_z / mag ]
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
            future.set_result(True)
            return
        twist_msg = TwistStamped()
        twist_msg.header.frame_id = ur5.base_link_name()
        twist_msg.header.stamp = node.get_clock().now().to_msg()
        twist_msg.twist.linear.x = 2 * vt[0]
        twist_msg.twist.linear.y = 2 * vt[1]
        twist_msg.twist.linear.z = 2 * vt[2]
        twist_msg.twist.angular.x = 0.0
        twist_msg.twist.angular.y = 0.0
        twist_msg.twist.angular.z = 0.0
        twist_pub.publish(twist_msg) 
    # Create timer for moving the end effector
    node.create_timer(0.002, func)
    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node1)
    executor.add_node(node)
    executor.spin_until_future_complete(future)
    node.destroy_node()
    node1.destroy_node()
    time.sleep(1)

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

def main():
    rclpy.init()

    # Make servoing service call to start servoing
    make_servoing_service_call()

    node_finder = TfFramesFinder()
    moveit_control = MoveItJointControl()
    gripper_controller = GripperController()
    context_switcher = ContextSwitcher()
    joints_sub = StateSubscriber()

    try:
        # Spin the node for 2 seconds.
        for _ in range(500):
            rclpy.spin_once(node_finder)
            time.sleep(0.01)
        node_finder.get_all_frames()
    except KeyboardInterrupt as e:
        print("k/b interrupted")
    global frame_names,target_pose
    print(frame_names)

    marign = 0.01
    # Distance from boxes
    dist_for_pick = 0.05 # For Pre Pick Pose
    dist_for_drop = 0.14 # For Pre Drop Pose
    preDropOffset = 0.01 # Pre Drop pose offset in Z axis
    postPickOffset = 0.4

    # Now making the ur_5 move
    for i in range(0,len(frame_names)):
        to_frame = frame_names[i]
        rclpy.spin_once(node_finder)
        target = node_finder.get_pose(to_frame) # Get the current pose of 'obj_<marker_id>'
        print()
        print(target)

        # Determining whether to turn in left direction or right direction to pick the current object
        jointList = []
        prePickPose = []
        preDropPose = []
        postDropPose = None

        if target[1] >= 0.37 : # Left
            print('Going to Left')
            prePickPose = [target[0],target[1] - dist_for_pick, target[2]]
            preDropPose = [target[0],target[1] - dist_for_drop, target[2] + preDropOffset]
            pose = [target[0],target[1] - postPickOffset, target[2] + preDropOffset]
            jointList = [start,left]
        elif target[1] <= -0.28 : # Right
            print('Going to Right')
            prePickPose = [target[0],target[1] + dist_for_pick,target[2]]
            preDropPose = [target[0],target[1] + dist_for_drop,target[2] + preDropOffset]
            pose = [target[0],target[1] + postPickOffset,target[2] + preDropOffset]
            jointList = [start,right]
        else : # Center
            prePickPose = [target[0] - dist_for_pick,target[1],target[2]]
            preDropPose = [target[0] - dist_for_pick,target[1],target[2] + preDropOffset]
            pose = [target[0] - postPickOffset,target[1],target[2] + preDropOffset]  
            print('Going Center')
            jointList = [start]
        
        context_switcher.activateMoveit()
        for joint in jointList:
            moveit_control.move_to_joint_positions(joint)
            time.sleep(0.5)

        # Servoing to prePose of boxes
        context_switcher.activateServoing()
        Servoing(prePickPose,marign)
        print('\n Reached PrePickPose \n')

        # Align the Yaw of the boxes
        yaw_Servoing(target[3],4.0)
        print('\n Yaw Aligned \n')

        # Servoing to boxes 
        Servoing(target,marign)
        print('\n Reached Target \n')

        # Activate the gripper
        gripper_controller.gripper_call(True)
        time.sleep(0.5)

        # Servoing back to prePose of boxes
        Servoing(preDropPose,marign)
        time.sleep(0.5)
        print('\n Reached PreDropPose \n')

        global positions_array
        while len(positions_array)==0:
            print('Waiting for current joint psoitions')
            rclpy.spin_once(joints_sub)
        
        print(positions_array)
        
        last = positions_array.pop()
        positions_array.insert(0, last)
        positions_array[-2] = -1.57
        context_switcher.activateMoveit()
        moveit_control.move_to_joint_positions(positions_array)
        time.sleep(2)
        positions_array = []

        # Coming back with box attached
        context_switcher.activateServoing()
        Servoing(pose,marign)
        time.sleep(1)

        # Move to Drop location
        context_switcher.activateMoveit()
        moveit_control.move_to_joint_positions(jointList[-1]) # MOving to the last joint positions to avoid collisions 
        time.sleep(0.5)
        moveit_control.move_to_joint_positions(drop) # Setting joints to reach Drop location
        time.sleep(0.5)

        # Moving the drop location further as there will be a box placed there previously
        # context_switcher.activateServoing()
        # Servoing([drop_pose[0]+drop_offset[i][0],drop_pose[1]+drop_offset[i][1],drop_pose[2]+drop_offset[i][2]],marign) # Precisely reaching drop location using Servoing

        # Deactivate the gripper
        gripper_controller.gripper_call(False)
        time.sleep(0.5)
    rclpy.shutdown()
if __name__ == "__main__":
    main()
