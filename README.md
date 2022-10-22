# RoboticsND: Project #5
This repository contains the files for Udacity's Robotics Nanodegree's Fifth Project.

Find Udacity's Robotics Software Engineering Nanodegree [here](https://www.udacity.com/course/robotics-software-engineer--nd209).

## Project Description

This project serves as a simulation of a real-life home service robot in Gazebo using ROS. The robot uses its sensors to localize itself and map the environment in order to pick up different elements inside the environment and transport them to other locations using Dijkstra's algorithm.

The purpose of this final project is to implement all previous major skills learnt throughout the Udacity Nanodegree. These skills include but are not limited to the following.

- Building simulation environments
- Simulating robots using ROS and Gazebo
- Developing ROS software (nodes, publishers, subscribers, etc.)
- Performing robot localization (through the Adaptive Monte-Carlo Localization algorithm)
- Performing SLAM (through the Real-Time Appearance-Based Mapping package)
- Performing path planning (using Dijkstra's algorithm)
- Integrating various ROS packages into one functional stack

## Project requirements

- Design a simulation environment inside Gazebo
- Simulate the turtlebot robot inside the designed environment
- Generate a 2D occurancy grid map (`map/map.pgm`) of the simulation environment through the `gmapping` package by teleoperating the robot
- Perform localization through the `amcl` package
- Test the navigation stack through sending 2D Nav Goals to the robot inside RViz
- Write a pick_objects node that is responsible of actuating the robot to various locations inside the mapped environment
- Write an add_markers node that is responsible of publishing marker locations to simulate picking up and dropping off objects

![](https://i.imgur.com/Bi0z5xi.png)
<p align = "center"><b>Fig.1 - Robot inside Gazebo simulation environment</b></p>

## Installation
In order to successfully run all functionalities of this project, you should have ROS Kinetic and Gazebo 7 installed.

#### 1. Create a `catkin_ws/`

```sh
$ mkdir -p ~/catkin_ws/src/
$ cd ~/catkin_ws/src/
$ catkin_init_workspace
```
#### 2. Clone the project inside `catkin_ws/src/`

```sh
$ git clone https://github.com/moudallal/RoboticsND-Project5.git
```
#### 3. Build your catkin workspace

```sh
$ cd ~/catkin_ws
$ catkin_make
```
#### 4. Source your environment

```sh
$ source devel/setup.bash
```
#### 5. Change permissions on the scripts file to execute them

```sh
$ cd src/scripts
$ sudo chmod +x *.sh
```

## SLAM

In order to test the SLAM algorithm, run the following command in your terminal.

```sh
$ ./test_slam.sh
```

You can teleoperate the robot around the environment in order to map it. The 2D and 3D maps of the environment are shown below.
<br />
<p align = "center">
	<img src="https://i.imgur.com/2OayBj6.png" />
	<br />
	<b>Fig.2 - Generated 2D Map of the simulation environment</b>
  <br />
</p>
<p align = "center">
  <img src="https://i.imgur.com/WuB3Bqp.png" />
	<br />
	<b>Fig.3 - Generated 3D Map of the simulation environment</b>
</p>

## Navigation

In order to test the navigation algorithm, run the following command in your terminal.

```sh
$ ./test_navigation.sh
```

You can press on the <b>2D Nav Goal</b> button inside RViz and point in different locations inside the map to let the robot plan paths insire the environment

## Integration

In order to run the full project, run the following command in your terminal.
```sh
$ ./home_service.sh
```

This will make the robot pickup a red cube object from one side of the map and then transport it to drop it off at the opposite side of the map. Screenshots of the process can be found below.

![](https://i.imgur.com/k96oFOT.png)
<p align = "center"><b>Fig.4 - Robot picking up red cube</b></p>
<br />

![](https://i.imgur.com/UEJjTWm.png)
<p align = "center"><b>Fig.5 - Robot planning paths inside the simulation environment</b></p>

You can change the different pickup and dropoff positions inside `add_markers/src/add_markers_node.cpp` and test out different processes.

## Credits

This project was fully done by myself.
