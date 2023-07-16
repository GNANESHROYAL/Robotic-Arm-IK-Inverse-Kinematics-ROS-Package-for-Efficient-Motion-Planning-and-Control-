# ROS-Inverse_kinematics

# Creating a Catkin Workspace in ROS

A Catkin workspace is a directory where you can build and organize your ROS packages. Follow the steps below to create a new Catkin workspace in ROS.

## Prerequisites

Before creating a Catkin workspace, make sure you have ROS installed on your system. If you haven't installed ROS yet, please refer to the official ROS documentation for installation instructions.

## Steps

1. Open a terminal.

2. Create a new directory for your Catkin workspace. You can choose any name for your workspace. In this example, we'll use `catkin_ws`:

3. Initialize the Catkin workspace by running the `catkin_init_workspace` command. This will create a `CMakeLists.txt` file in the `src` directory:

4. Build the Catkin workspace by running the `catkin_make` command. This will create a `build` and a `devel` directory:

5. Source the setup script to add the workspace to your ROS environment:

> **Note:** You'll need to run this command every time you open a new terminal to work with your Catkin workspace.

6. Test your Catkin workspace by running the `rospack` command. It should display the list of packages in your workspace:

If everything is set up correctly, you should see the packages in your `src` directory listed.

7. Congratulations! You have successfully created a Catkin workspace in ROS.

## Adding Packages

To add packages to your Catkin workspace, navigate to the `src` directory and clone or create new packages in separate directories. For example:

Once you have added packages, repeat steps 4 and 5 to build and source your workspace again.

## Additional Resources

- [ROS Wiki: Catkin Workspaces](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
- [ROS Wiki: Building Packages](http://wiki.ros.org/catkin/CMakeLists.txt)
- [ROS Wiki: Package Manifest](http://wiki.ros.org/catkin/package.xml)

# Creating a Catkin Package in ROS

In ROS, a package is the basic organizational unit. It contains libraries, executables, scripts, and other resources that contribute to a specific functionality. Follow the steps below to create a package within your Catkin workspace.

## Prerequisites

Before creating a package, ensure that you have set up a Catkin workspace as described in the previous section.

## Steps

1. Open a terminal and navigate to your Catkin workspace directory. For example:

2. Use the `catkin_create_pkg` command to create a new package. Replace `package_name` with the desired name of your package, and list any dependencies your package requires after the `DEPENDENCIES` argument. For example:


This will create a new package named `package_name` with dependencies on `std_msgs`, `rospy`, and `roscpp`.

3. Navigate to the `src` directory within your Catkin workspace:


4. Inside the `src` directory, you'll find a folder named `package_name`. This folder contains the package template files. You can start adding your code and resources to this package.

- Place your C++ source code in the `src` directory.
- Place your Python scripts in the `scripts` directory.
- Place any additional resource files in the `resources` directory.

5. Open a terminal and navigate back to your Catkin workspace directory. Run the `catkin_make` command to build your workspace, including the newly created package:


6. Once the build is complete, source the setup script to add the package to your ROS environment:


7. You can now use the packages and executables within your Catkin workspace.

## Additional Resources

- [ROS Wiki: Creating a Package](http://wiki.ros.org/catkin/creating_a_workspace)
- [ROS Wiki: Catkin Package Manifest](http://wiki.ros.org/catkin/package.xml)
- [ROS Wiki: Building Packages](http://wiki.ros.org/catkin/CMakeLists.txt)

![1](https://github.com/GNANESHROYAL/ROS-Inverse_kinematics/assets/113758576/342e78e1-400b-4da9-b702-bad691c7a4cf)

![2](https://github.com/GNANESHROYAL/ROS-Inverse_kinematics/assets/113758576/4a8f2a29-c4c8-42e6-b945-e928da125067)

![3](https://github.com/GNANESHROYAL/ROS-Inverse_kinematics/assets/113758576/f7e4f6f7-a022-4d67-b2f6-3332e4b0621d)

![4](https://github.com/GNANESHROYAL/ROS-Inverse_kinematics/assets/113758576/1111cf28-bc53-408a-a4f0-cb23907d8408)

![5](https://github.com/GNANESHROYAL/ROS-Inverse_kinematics/assets/113758576/7559da48-71c5-4ea5-8104-9ab47cea8eef)

![6](https://github.com/GNANESHROYAL/ROS-Inverse_kinematics/assets/113758576/0f8b6091-cc06-4263-88c6-2e1d7f1e265f)
