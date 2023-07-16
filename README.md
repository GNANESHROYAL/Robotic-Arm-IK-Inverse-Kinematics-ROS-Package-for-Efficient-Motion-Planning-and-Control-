# ROS-IK-Solver: Robotic Arm Inverse Kinematics Solver for ROS

## Introduction

This package provides an implementation of the inverse kinematics for a robotic arm in ROS. It allows you to control the arm by entering desired coordinates and observing the arm movement.

## Prerequisites

Before running the package, make sure you have the following prerequisites:

- ROS installed on your system. If you haven't installed ROS yet, please refer to the official ROS documentation for installation instructions.
- A working Catkin workspace. If you don't have a Catkin workspace, follow the steps mentioned in the section below to create one.

## Creating a Catkin Workspace in ROS

A Catkin workspace is a directory where you can build and organize your ROS packages. Follow the steps below to create a new Catkin workspace in ROS.

### Steps

1. Open a terminal.

2. Create a new directory for your Catkin workspace. You can choose any name for your workspace. In this example, we'll use `catkin_ws`:

 ```
$ mkdir -p ~/catkin_ws/src
 ```

4. Build the Catkin workspace by running the `catkin_make` command. This will create a `build` and a `devel` directory:

 ```
$ cd ~/catkin_ws/
$ catkin_make
 ```

5. Source the setup script to add the workspace to your ROS environment:

 ```
 $ source devel/setup.bash
 ```

> **Note:** You'll need to run this command every time you open a new terminal to work with your Catkin workspace.

6. Congratulations! You have successfully created a Catkin workspace in ROS.

![1](https://github.com/GNANESHROYAL/ROS-Inverse_kinematics/assets/113758576/a799dde7-e18d-4e4c-997b-68ae534948c0)

## Creating a Catkin Package in ROS

In ROS, a package is the basic organizational unit. It contains libraries, executables, scripts, and other resources that contribute to a specific functionality. Follow the steps below to create a package within your Catkin workspace.

### Steps

1. Open a terminal and navigate to your Catkin workspace directory:

 ```
$ cd catkin_ws
 ```

2. Navigate to the `src` directory within your Catkin workspace:

 ```
$ cd src
 ```

3. Use the `catkin_create_pkg` command to create a new package. Replace `package_name` with the desired name of your package, and list any dependencies your package requires after the `DEPENDENCIES` argument:

 ```
$catkin_create_pkg robotic_arm std_msgs rospy roscpp
 ```

This will create a new package named `package_name` with dependencies on `std_msgs`, `rospy`, and `roscpp`.


4. Inside the `src` directory, you'll find a folder named `robotic_arm`. This folder contains the package template files. You can start adding your code and resources to this package.

- Place your C++ source code in the `src` directory.
- Place your Python scripts in the `scripts` directory.
- Place any additional resource files in the `resources` directory.

5. Open a terminal and navigate back to your Catkin workspace directory. Run the `catkin_make` command to build your workspace, including the newly created package:

 ```
$ cd catkin_ws
$ catkin_make
 ```

6. Once the build is complete, source the setup script to add the package to your ROS environment:

 ```
 $ source devel/setup.bash
 ```

8. You can now use the packages and executables within your Catkin workspace.

![2](https://github.com/GNANESHROYAL/ROS-Inverse_kinematics/assets/113758576/f9c5da4b-41e5-4069-8e34-1b2503b631f2)

## Running the Robotic Arm Package

To run the robotic_arm package and its nodes, follow the steps below:

### Steps

1. Copy the following files into the `robotic_arm` package directory:

- `arm.urdf`
- `arm.launch`
- `inverse_kinematics_ros_urdf.py`

![3](https://github.com/GNANESHROYAL/ROS-Inverse_kinematics/assets/113758576/05b3089e-3afb-4f82-a141-c98275e1d28b)

2. Open a terminal and navigate to your Catkin workspace directory:

 ```
 $ cd catkin_ws
 ```

3. Build your Catkin workspace by running the `catkin_make` command:

 ```
 $ catkin_make
 ```

4. Launch the `arm.launch` file using the `roslaunch` command:

 ```
 $ roslaunch robotic_arm arm.launch
 ```

This will start the RViz visualization tool with the robotic arm model loaded.

![4](https://github.com/GNANESHROYAL/ROS-Inverse_kinematics/assets/113758576/e79ae751-bd6f-44f7-866c-d141e1c6ba6d)

5. In a new terminal, run the `rostopic list` command to see a list of all available topics:

 ```
 $ rostopic list
 ```

This will display a list of topics that are currently being published and subscribed to in ROS. You should see the `joint_states` topic among them.

6. Run the `rosnode list` command to see a list of all running nodes:

 ```
 $ rosnode list
 ```

This will display a list of active nodes in the ROS system. Check for the `joint_state_publisher` node, which is responsible for publishing joint states.

7. To get more information about the `joint_state_publisher` node, use the `rosnode info` command followed by the node name:

 ```
 $ rosnode info /joint_state_publisher
 ```

This will provide detailed information about the `joint_state_publisher` node, such as its publications, subscriptions, and node URI.

8. To stop the `joint_state_publisher` node, run the `rosnode kill` command followed by the node name:

 ```
 $ rosnode kill /joint_state_publisher
 ```

This will terminate the node as it is publishing to the `joint_states` topic.

9. Finally, in a new terminal, run the `inverse_kinematics_ros_urdf.py` script using the `rosrun` command:

 ```
 $ rosrun robotic_arm inverse_kinematics_ros_urdf.py
 ```

 This will start the inverse kinematics node, allowing you to enter your desired coordinates for the robotic arm.

![5](https://github.com/GNANESHROYAL/ROS-Inverse_kinematics/assets/113758576/89f9d06c-36b5-432e-82a1-fa9f1a16e883)

10. Follow the instructions prompted by the `inverse_kinematics_ros_urdf.py` script in the terminal to enter your desired coordinates and observe the arm movement.

![6](https://github.com/GNANESHROYAL/ROS-Inverse_kinematics/assets/113758576/83d04c57-10ed-438a-9a7a-e86e37e7a30a)

## Additional Resources

- [ROS Wiki: roslaunch](http://wiki.ros.org/roslaunch)
- [ROS Wiki: rosnode](http://wiki.ros.org/rosnode)
- [ROS Wiki: rosrun](http://wiki.ros.org/rosrun)
- [ROS Wiki: RViz](http://wiki.ros.org/rviz)

## Tags

`ROS`
`Robotics`
`Inverse Kinematics`
`Manipulator`
`Kinematics`
`ROS Package`
`URDF`
`Robot Arm`
`Motion Planning`
`Robot Control`
`Python`
`ROS Nodes`
`ROS Launch`
`Robot Simulation`
`Rviz`
`ROS Visualization`
`Robotic Arm`
`ROS Development`
`Open Source`
