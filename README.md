# Cosmo-Logistic-eYRC-2023-24

## Team ID :: 1314

## Theme Introduction
The “Cosmo Logistic” theme of eYRC 2023-24 is set in a warehouse used for inter-planet logistics from a space station. A robotic arm and mobile robot collaborate to sort and prepare packages to be transported to different planets. In this theme, teams will develop an algorithm for sorting packages autonomously with the help of a robotic arm and mobile robot shown in the figure. Teams will learn to navigate this mobile robot with the help of SLAM (simultaneous localization and mapping) method in a warehouse, detect and localise the packages placed on racks, and manipulate the robotic arm to pick them.

## Tasks
### __Task 1__
> [___Task 1A (Object Pose Estimation)___ __::__](/ur_description/scripts/)

__Locating Aruco in the world with respect to arm__

Subtasks::
1. Detecting and Locating Aruco markers using OpenCV. Calculating the Yaw angle to be published using Rotational Vector
2. Publishing Transform using TF2 library between 'camera_link' and 'cam_<marker_id>'
3. Looking for transform between 'base_link' and 'cam_<marker_id>' (as 'camera_link' and 'base_link' are connected in TF tree)
4. Publishing a tranform of 'obj_<marker_id>' with respect to 'base_link'

```sh
$ ros2 launch ur_description ur5_gazebo_launch.py
```
```sh
$ ros2 launch ur_description spawn_ur5_launch.py
```
```sh
$ ros2 run ur_description task1a.py
```

---

> [___Task 1B (Arm Manipulation using Moveit)___ __::__](/pymoveit2/examples/)

__Manipulating Robotic Arm using Moveit framework__

Subtasks::
1. Using joint goals to achieve particular position
2. Using moveit_servo to achieve the box's position in a feedback loop
3. Using TF Listener to estimate the pose of end effector (i.e. gripper ) and calculating the unit vector to provide to moveit servo linear velocities
4. Repeat the task multiple hard coded positions 
```sh
$ ros2 launch ur_description ur5_gazebo_launch.py
```
```sh
$ ros2 launch ur5_moveit spawn_ur5_launch_moveit.launch.py
```
```sh
$ ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {} # Enables Servoing
$ ros2 run pymoveit2 task1b.py
```
---

> [___Task 1C (Navigation)___ __::__](/ebot_nav2/scripts/)

__Navigation with ROS2 Navigation Stack (Nav2)__

Subtasks::
1. Mapping the warehouse using teleop twist and saving the generated map using slam_toolbox
2. Using Simple Commander API to various positions 
3. Tuning the parameters in nav2_param.yaml to set the tolerance in position and orientation 
4. Marking a Avoiding Zone in a map

```sh
$ ros2 launch ebot_description ebot_gazebo_launch.py
```
```sh
$ ros2 launch ebot_nav2 ebot_bringup_launch.py
```
```sh
$ ros2 run ebot_nav2 task1c.py
```

---
---
---


### __Task 2__
> [___Task 2A (Manipulation with vision)___ __::__](/pymoveit2/examples/)

__Motion planning of robotic arm with vision__

Subtasks::
1. Getting the transform generated in Task1A to get the Transform of randomly spawned Aruco boxes
2. Determining the pre pose of ur5 arm based on the position of box 
3. Using joint goals to achieve pre position for picking the box
4. Using moveit_servo to achieve the box's position in a feedback loop
5. Activating the magnetic gripper using ros2 service call
6. Going to drop position adding offset to drop position so that boxes dont collide
7. Deactivate the gripper and drop the box on the table

```sh
$ ros2 launch ur_description ur5_gazebo_launch.py
```
```sh
$ ros2 launch ur5_moveit spawn_ur5_launch_moveit.launch.py
```
```sh
$ ros2 run ur_description task1a.py
```
```sh
$ ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {} # Enables Servoing
$ ros2 run pymoveit2 task2a.py
```

[Video Link](https://www.youtube.com/watch?v=V2NQAFl0xxk)

---

> [___Task 2B (Dock And Place)___ __::__](/ebot_docking/scripts/)

__eBot Navigation with Docking and Placing the Rack__

Subtasks::
1. Going to Rack1 pose using Simple Commander API of Nav2 stack
2. Creating a Docking Server to Align the bot angularly first then in Linear distance
3. Using a P controller docking the eBot to the Rack
4. Using Attach_Link service to attach the Rack to the eBot
5. Going to Arm Pose 1 (AP1) and use the Docking service to achieve certain angle and place the Rack
6. Using Detach_Link service to detach the Rack from the eBot
7. Go to Home Position Again

```sh
$ ros2 launch ebot_description ebot_gazebo_launch.py
```
```sh
$ ros2 launch ebot_nav2 ebot_bringup_launch.py
```
```sh
$ ros2 run ebot_docking ebot_docking_service.py # Starts the Docking service server
```
```sh
$ ros2 run ebot_docking task2b.py
```

[Video Link](https://youtu.be/pV-qAvbujhw)

### __Task 3__
> [___Task 3A (Remote Hardware Task - Object Pose Estimation)___ __::__](/ur_description/scripts/)

__3A Aruco and TF on Hardware__

Subtasks::
1. Implementation of Task 1A on hardware
2. Parameters tuning of Aruco Detection Algorithm for real world scenarios  
3. Using and applying filters on input image for detection of Aruco in different lighting conditions

---

> [___Task 3B (Dock , Pick And Place)___ __::__](/pymoveit2/examples/)

__eBot Navigation with Docking and Placing the Rack and Picking Aruco Boxes__

Subtasks::
1. Going to specified rack pose in config.yaml file 
2. Using Simple Commander API of Nav2 stack to navigate to specified rack
3. Using a P controller docking the eBot to the Rack
4. Navigating to Arm Pose 1 (AP1) and use the Docking service to achieve certain angle and place the Rack 
5. Looking for boxes in proximity of the arm
6. Picking the Aruco boxes from the Rack and placing it on the table

```sh
$ ros2 launch eyantra_warehouse task3a.launch.py
```
```sh
$ ros2 launch ebot_nav2 ebot_bringup_launch.py
```
```sh
$ ros2 launch ur5_moveit spawn_ur5_launch_moveit.launch.py
```
```sh
$ ros2 run ebot_docking ebot_docking_service.py # Starts the Docking service server
```
```sh
$ ros2 run ur_description task1a.py
```
```sh
$ ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {} # Enables Servoing
$ ros2 run pymoveit2 task3b.py
```

[Video Link](https://youtu.be/Z2tGPUlRXYg)
