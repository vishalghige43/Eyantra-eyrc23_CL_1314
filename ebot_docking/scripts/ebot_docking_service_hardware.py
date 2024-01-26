#!/usr/bin/env python3

## Overview

# ###
# This ROS2 script is designed to control a robot's docking behavior with a rack. 
# It utilizes odometry data, ultrasonic sensor readings, and provides docking control through a custom service. 
# The script handles both linear and angular motion to achieve docking alignment and execution.
# ###

# Import necessary ROS2 packages and message types
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range,Imu
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf_transformations import euler_from_quaternion
from ebot_docking.srv import DockSw  # Import custom service message
import math, statistics
from std_msgs.msg import Float32,Float32MultiArray
from sensor_msgs.msg import Imu

# Define a class for your ROS2 node
class MyRobotDockingController(Node):

    def __init__(self):
        # Initialize the ROS2 node with a unique name
        super().__init__('my_robot_docking_controller')
        self.get_logger().info('Started Docking Server')

        # Create a callback group for managing callbacks
        self.callback_group = ReentrantCallbackGroup()

        # Subscribe to odometry data for robot pose information
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)

        # Subscribe to ultrasonic sensor data for distance measurements
        self.ultra_sub = self.create_subscription(Float32MultiArray, 'ultrasonic_sensor_std_float', self.ultra_callback, 10)
        # self.ultrasonic_rl_sub = self.create_subscription(Range, '/ultrasonic_rl/scan', self.ultrasonic_rl_callback, 10)
        # # Add another one here
        # self.ultrasonic_rr_sub = self.create_subscription(Range,'/ultrasonic_rr/scan', self.ultrasonic_rr_callback, 10)

        # Subscribe to IMU data on /imu topic
        self.imu_sub = self.create_subscription(Imu, '/sensors/imu1', self.imu_callback, 10) 

        # Create a ROS2 service for controlling docking behavior, can add another custom service message
        self.dock_control_srv = self.create_service(DockSw, 'dock_control', self.dock_control_callback, callback_group=self.callback_group)

        # Create a publisher for sending velocity commands to the robot
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Initialize all  flags and parameters here
        # Boolean Flags
        self.is_docking = False
        self.docked_linearly = False
        self.docked_angularly = False
        self.dock_aligned = False
        # Params
        self.robot_pose = [0.0 , 0.0 , 0.0]
        self.linear_goal = 0.0
        self.angular_goal = 0.0
        self.angular_Kp = 1
        self.linear_Kp = 0.5

        # Initialize a timer for the main control loop
        self.controller_timer = self.create_timer(0.1, self.controller_loop)

    # Callback function for odometry data
    def odometry_callback(self, msg):
        # Extract and update robot pose information from odometry message
        self.robot_pose[0] = msg.pose.pose.position.x
        self.robot_pose[1] = msg.pose.pose.position.y
        quaternion_array = msg.pose.pose.orientation
        orientation_list = [quaternion_array.x, quaternion_array.y, quaternion_array.z, quaternion_array.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.robot_pose[2] = yaw

    # callback for ultrasonic subscription(unit in cm || appropriate distance from rack is 15 cm)
    def ultra_callback(self,msg):
        self.usrleft_value = msg.data[4]
        self.usrright_value = msg.data[5]
        # print(f'left= {self.usrleft_value} right = {self.usrright_value}')

    def imu_callback(self, msg):
        _,_,self.curr_yaw = euler_from_quaternion([msg.orientation.x ,msg.orientation.y ,msg.orientation.z ,msg.orientation.w])
        self.curr_yaw = self.normalize_angle(self.curr_yaw)
        # print(f'from cl_1314 {self.curr_yaw}')
        

    # Utility function to normalize angles within the range of -π to π (OPTIONAL)
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    # Main control loop for managing docking behavior

    def controller_loop(self):

        # The controller loop manages the robot's linear and angular motion 
        # control to achieve docking alignment and execution
        angular_tolerance = 0.06
        linear_tolerance = 0.5
        if self.is_docking:
            # ...
            # Implement control logic here for linear and angular motion
            # For example P-controller is enough, what is P-controller go check it out !
            # ...

            '''
            # IMP Info
            Distance between the 2 ultrasonic sensor is 0.3 m
            Distance given by each ultrasonic sensor is in meters

            # Steps
            While loop until linear distance is aligned and angular value is achieved
            1. Align with rack in angular value
            2. Align with rack in distance
            3.Publish the Twist message at every instant
            '''
            msg = Twist()
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0
            if not self.docked_angularly :
                # Align the bot in proper orientation first
                # Calculate the current angle using ultrasonic data
                # curr_yaw = math.atan((self.usrleft_value - self.usrright_value) / 0.3) # Using ultrasonic
                # 
                # Using IMU align in angular values 
                if abs(self.curr_yaw - self.angular_goal) < angular_tolerance :
                    msg.angular.z = 0.0
                    self.docked_angularly = True
                    if self.docked_linearly:
                        self.is_docking = False
                        self.dock_aligned = True
                else :
                    print(f'current yaw {self.curr_yaw} and angular goal is {self.angular_goal}')
                    msg.angular.z = self.angular_Kp * (self.angular_goal - self.curr_yaw)
            elif not self.docked_linearly :
                # Achieve the specified distance
                # Calculate the distance from the surface
                curr_dist = (self.usrleft_value + self.usrright_value)/2.0 # Averaging the distance of both Ultrasonic Sensors
                if (abs(curr_dist - self.linear_goal) < linear_tolerance) :
                    msg.linear.x = 0.0
                    self.docked_linearly = True
                    self.is_docking = False
                    self.dock_aligned = True
                else :
                    msg.linear.x = self.linear_Kp * (self.linear_goal - curr_dist)
                    print(f'current dist {curr_dist} and linear goal is {self.linear_goal}')
            self.cmd_vel_pub.publish(msg)

    # Callback function for the DockControl service
    def dock_control_callback(self, request, response):
        # Extract desired docking parameters from the service request
        self.linear_goal = request.distance
        self.angular_goal = request.orientation

        # Reset flags and start the docking process
        self.is_docking = True
        self.docked_linearly = request.linear_dock
        self.docked_angularly = request.orientation_dock
        self.dock_aligned = False

        # Log a message indicating that docking has started
        self.get_logger().info(" ----- ")
        self.get_logger().info(" Docking started! ")
        self.get_logger().info("For angular goal of "+str(self.angular_goal)+" and for linear goal of "+str(self.linear_goal))

        # Create a rate object to control the loop frequency
        rate = self.create_rate(2, self.get_clock())

        # Wait until the robot is aligned for docking
        while not self.dock_aligned:
            self.get_logger().info("Waiting for alignment...")
            self.controller_loop()
            rate.sleep()

        self.get_logger().info("!!! DONE WITH DOCKING")
        # Set the service response indicating success
        response.success = True
        response.message = "Docking control initiated"
        return response

# Main function to initialize the ROS2 node and spin the executor
def main(args=None):
    rclpy.init(args=args)

    my_robot_docking_controller = MyRobotDockingController()

    executor = MultiThreadedExecutor()
    executor.add_node(my_robot_docking_controller)

    executor.spin()

    my_robot_docking_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
