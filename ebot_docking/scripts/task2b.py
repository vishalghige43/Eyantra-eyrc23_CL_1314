#! /usr/bin/env python3
# Team ID:          [ 1314 ]
# Author List:		[ Swaraj Zende, Vishal Ghige, Vipul Pardeshi, Vishal Singh ]
# Filename:		    task2b.py

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
"""
Script to dock the rack and go at the correct pose and place it
"""

################### GLOBAL VARIABLES #######################
# Poses to be achieved
points = [ [0.0 , 0.0],     # Origin
           [0.45, 4.6],     # Pre Rack1 pose
           [0.9 , -2.455],  # AP1
           [0.0 , -2.455]]  # Getting out of under the rack

# quaternion values respective poses
orientations = [[ 0.0 , 0.0 , 0.0 , 1.0 ],
                [ 0.0 , 0.0 , -0.9999997, 0.0007963 ], # 3.14 radian in z axis
                [ 0.0 , 0.0 ,  0.0, 1.0 ], # 0.0 radian in z axis
                [ 0.0 , 0.0 ,  0.9999997, 0.0007963 ]]

################### CLASS DEFINITION #######################

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

################### MAIN FUNCTION #######################
def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Set our demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = points[0][0]
    initial_pose.pose.position.y = points[0][0]
    initial_pose.pose.orientation.x = orientations[0][0]
    initial_pose.pose.orientation.y = orientations[0][1]
    initial_pose.pose.orientation.z = orientations[0][2]
    initial_pose.pose.orientation.w = orientations[0][3]
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()
    navigator.changeMap('/home/saz/colcon_ws/src/ebot_nav2/maps/map.yaml') # Specifying the map to load

    i = 1
    while i < len(points):
        # Go to Rack 1 Pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = points[i][0]
        goal_pose.pose.position.y = points[i][1]
        goal_pose.pose.orientation.x = orientations[i][0]
        goal_pose.pose.orientation.y = orientations[i][1]
        goal_pose.pose.orientation.z = orientations[i][2]
        goal_pose.pose.orientation.w = orientations[i][3]

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
        
        i = i + 1

        # Call the Docking service here
        dockClient = Docking_Client()
        dockClient.send_request(angle = 3.14 , distance = 0.01)


        # Call the Attach Link Service call here
        rack_attach = LinkAttacher()
        rack_attach.attach_link('rack1')
         
        # Go to AP1 Position
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = points[i][0]
        goal_pose.pose.position.y = points[i][1]
        goal_pose.pose.orientation.x = orientations[i][0]
        goal_pose.pose.orientation.y = orientations[i][1]
        goal_pose.pose.orientation.z = orientations[i][2]
        goal_pose.pose.orientation.w = orientations[i][3]

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
        
        i = i + 1

        # Call the Docking service here
        dockClient.send_request(angle = 3.14, distance = 0.0 , linear=True ,angular=False )
        
        # Call the Detach Link Service call here
        rack_detach = LinkDetacher()
        rack_detach.detach_link('rack1')
        
        # Go to origin Position
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = points[i][0]
        goal_pose.pose.position.y = points[i][1]
        goal_pose.pose.orientation.x = orientations[i][0]
        goal_pose.pose.orientation.y = orientations[i][1]
        goal_pose.pose.orientation.z = orientations[i][2]
        goal_pose.pose.orientation.w = orientations[i][3]

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
        
        i = i + 1

    navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()