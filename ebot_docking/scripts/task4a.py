#! /usr/bin/env python3
# Team ID:          [ 1314 ]
# Author List:		[ Swaraj Zende, Vishal Ghige, Vipul Pardeshi, Vishal Singh ]
# Filename:		    task4a.py

################### IMPORT MODULES #######################
from geometry_msgs.msg import PoseStamped,Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from linkattacher_msgs.srv import AttachLink,DetachLink
from ebot_docking.srv import DockSw 
from tf_transformations import euler_from_quaternion
import time

from usb_relay.srv import RelaySw
import yaml
from tf_transformations import quaternion_from_euler
"""
Script to dock the rack and go at the correct pose and place it
"""

################### GLOBAL VARIABLES #######################
# Origin
origin = [ 0.0 , 0.0 , 0.0 ]
# AP1 (Arm Pose 1)
AP1 = [1.0 , -2.355 , 3.14]

################### CLASS DEFINITION #######################

class USBAttacher(Node):
    def __init__(self):
        super().__init__('USB_Attaching_Client')
        self.cli = self.create_client(RelaySw , '/usb_relay_sw')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Attaching service not available, waiting again...')
        self.req = RelaySw.Request()

    def send_request(self, arduino_reset : bool , electromagnet : bool):
        if arduino_reset :
            self.req.relaychannel = 1
            self.req.relaystate = True
            self.future1 = self.cli.call_async(self.req)
            rclpy.spin_until_future_complete(self, self.future1)
            time.sleep(1.0)
            self.req.relaystate = False
            self.future2 = self.cli.call_async(self.req)
            rclpy.spin_until_future_complete(self, self.future2)
            return self.future1.result() and self.future2.result()
        else :
            # Turn the electromagnet ON or OFF
            self.req.relaychannel = 0
            self.req.relaystate = electromagnet
            self.future = self.cli.call_async(self.req)
            rclpy.spin_until_future_complete(self, self.future)
            return self.future.result()

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

################### MAIN FUNCTION #######################
def main():
    rclpy.init()

    navigator = BasicNavigator()
    usbRelay = USBAttacher()

    # Set our demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = origin[0]
    initial_pose.pose.position.y = origin[1]
    orientations = quaternion_from_euler(0.0, 0.0, float(origin[2]))
    initial_pose.pose.orientation.x = orientations[0]
    initial_pose.pose.orientation.y = orientations[1]
    initial_pose.pose.orientation.z = orientations[2]
    initial_pose.pose.orientation.w = orientations[3]
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
    rack_ids = data.get("package_id", [])

    for rack_id in rack_ids:
        rack_id = int(rack_id)
        rack_pose = []
        for item in position_data:
            for rack, coordinates in item.items():
                if int(rack[4:]) == rack_id:
                    rack_pose = coordinates
        # Go to Rack Pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = rack_pose[0]
        goal_pose.pose.position.y = rack_pose[1]
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
        usbRelay.send_request(arduino_reset = False, electromagnet = True)
         
        # Go to AP1 Position
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = AP1[0]
        goal_pose.pose.position.y = AP1[1]
        orientations = quaternion_from_euler(0.0, 0.0, float(AP1[2]))
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
        dockClient.send_request(angle = float(AP1[2]), distance = 0.0 , linear=True ,angular=False )
        
        # Call the Detach Link Service call here
        usbRelay.send_request(arduino_reset = False, electromagnet = False)
        
        # Go to origin Position
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = origin[0]
        goal_pose.pose.position.y = origin[1]
        orientations = quaternion_from_euler(0.0, 0.0, float(origin[2]))
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
        

    navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()