#! /usr/bin/env python3
# Team ID:          [ 1314 ]
# Author List:		[ Swaraj Zende, Vishal Ghige, Vipul Pardeshi, Vishal Singh ]
# Filename:		    task1c.py

################### IMPORT MODULES #######################
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

"""
Basic navigation demo to go to achieve 3 poses.
"""

################### GLOBAL VARIABLES #######################
# Poses to be achieved
points = [ [0.0,0.0],   # Origin
           [1.8, 1.5],  # P1
           [2.0, -7.0], # P2
           [-3.0, 2.5], # P3
           [-1.0, -8.0]]# P4

# quaternion values respective poses
orientations = [[0.0 , 0.0 , 0.0 , 1.0 ],
                [ 0.0, 0.0, 0.7068252, 0.7073883 ],
                [ 0.0, 0.0, -0.7068252, 0.7073883 ],
                [ 0.0, 0.0, 0.7068252, 0.7073883 ],
                [ 0.0, 0.0, 0.7068252, 0.7073883 ]]

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
    navigator.changeMap('/home/saz/colcon_ws/src/ebot_nav2/maps/keepout_mask.yaml') # Specifying the map to load

    for i in range(1,len(points)):
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

    navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()