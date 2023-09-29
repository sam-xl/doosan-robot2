

# [Doosan Robotics](http://www.doosanrobotics.com/kr/)<img src="https://user-images.githubusercontent.com/47092672/97660147-142f1f00-1ab4-11eb-9d14-48f30a666cdc.PNG" width="10%" align="right">
[![license - apache 2.0](https://img.shields.io/:license-Apache%202.0-yellowgreen.svg)](https://opensource.org/licenses/Apache-2.0)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
[![support level: community](https://img.shields.io/badge/support%20level-community-lightgray.png)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)


# NB: This port is non-functional
These packages have been modified to compile under ROS2 Humble on Ubuntu 22.04. This also requires modified versions of `ros2_control`, `ros2_controllers` and `gazebo_ros2_control`. These are based on outdated commits of the official packages, which have been modified to compile under ROS2 Humble. Newer versions of the packages cannot be used, since the `dsr_control2` package uses an outdated hardware interface API.

This port of `doosan_robot2` from ROS2 Foxy to ROS2 Humble is not currently functional, due to a lack of the [DRFL](https://github.com/doosan-robotics/API-DRFL) library for Ubuntu 22.04. The file `common2/lib/humble/x86_64/libDRFL.a` is compiled for Ubuntu 20.04 and does not run under Ubuntu 22.04. No testing has been done for ROS2 Humble in Ubuntu 20.04.

The build instructions below should work, however whenever the DRFL library is called, the program may crash. For example, when attempting to connect to the robot using `Drfl.open_connection(...)`, a segmentation fault occurs.

# Build
**Read the note above before attempting to build**
``` bash
### Prerequisite installation elements before package installation
sudo apt install libpoco-dev ros-humble-moveit-msgs ros-humble-control-msgs ros-humble-realtime-tools ros-humble-xacro ros-humble-joint-state-publisher-gui

### We assume that you have installed the ros-humble-desktop package
### We recommand /home/<user_home>/ros2_ws/src
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone -b humble-dev https://github.com/sam-xl/doosan-robot2.git
git clone -b humble-doosan https://github.com/sam-xl/ros2_control.git
git clone -b humble-doosan https://github.com/sam-xl/ros2_controllers.git
git clone -b humble-doosan https://github.com/sam-xl/gazebo_ros2_control.git

rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y

colcon build --symlink-install

source install/setup.bash
```

\
\
\
\
\
\
\
\
&nbsp;

# Old README contents for reference only

# *overview*
    
    This package provides the function to control all models of Doosan robots in the ROS2(Foxy) environment.
    
    ※ Currently, ROS2 related packages are being updated rapidly. 
       Doosan packages will also be updated from time to time and features will be upgraded.
       
# _*article*_
<img src="https://user-images.githubusercontent.com/47092672/116173431-8a50cd80-a747-11eb-839c-b9f374d9a774.jpeg" width="80%">

[Doosan  Robotics Unveils Industry's First ROS Package that Supports ROS 2 Foxy Fitzroy](https://apnews.com/press-release/pr-newswire/technology-business-south-korea-materials-industry-robotics-511fcf63df0d36340748142a30e88319)




# *usage* <a id="chapter-3"></a>
### Joint State Publish
It can be run independently without a controller.
Using the robot model and the `joint_state_publisher_gui` package, you can see the robot moving on Rviz.
```bash
$ ros2 launch dsr_launcher2 dsr_joint_state_pub.launch.py model:=a0912 color:=blue
``` 
<img src="https://user-images.githubusercontent.com/47092672/97652654-40da3b00-1aa2-11eb-8621-2a36e3159de0.png" width="70%">

### Virtual Mode
If the "mode" argument is set to virtual, the DRCF emulator is automatically executed when launch.py ​​is executed.

##### Run dsr_control2 node 
You can execute the Control Node by using the command below.
```bash
$ ros2 launch dsr_launcher2 single_robot_rviz.launch.py model:=a0912 color:=blue
```
##### Run the example scripts
The robot can be driven by using the example scripts included in the dsr_example2 package.
Check that the controller and robot are connected normally, and enter the command below.
```bash
$ ros2 run dsr_example2_py dsr_service_motion_simple
```

### Real Mode
Use __real mode__ to drive a real robot   
The default IP of the robot controller is _192.168.127.100_ and the port is _12345_.

##### Run dsr_control2 node 
```bash
$ ros2 launch dsr_launcher2 single_robot_rviz.launch.py mode:=real host:=192.168.127.100 port:=12345
```

##### Run the example scripts
The robot can be driven by using the example scripts included in the dsr_example2 package.
Check that the controller and robot are connected normally, and enter the command below.
```bash
$ ros2 run dsr_example2_py dsr_service_motion_simple
```
---
### dsr_launcher2
The dsr_launcher2 package contains the launch.py ​​file that can link control node, rviz, and gazebo.
Therefore, it is necessary to connect with a real robot controller or emulator before executing the launch file.
Simulation can be performed with the command below.
```bash
$ ros2 launch dsr_launcher2 single_robot_rviz.launch.py
```
<img src="https://user-images.githubusercontent.com/47092672/97654894-3f5f4180-1aa7-11eb-83f0-90eb071d1f60.gif" width="70%">

```bash
$ ros2 launch dsr_launcher2 single_robot_gazebo.launch.py
```
<img src="https://user-images.githubusercontent.com/47092672/99232226-fe9c5200-2834-11eb-8719-f87cc56d55c7.gif" width="70%">

### Moveit
To use the moveit2 package, you need to install the following packages.
```bash
$ cd ~/ros2_ws/src
$ git clone https://github.com/ros-planning/moveit2
$ git clone -b ros2 --single-branch https://github.com/ros-planning/warehouse_ros
$ git clone -b ros2 --single-branch  https://github.com/ros-planning/warehouse_ros_mongo
$ git clone -b ros2 --single-branch https://github.com/ros-planning/srdfdom
$ git clone -b ros2 --single-branch https://github.com/ros-planning/geometric_shapes
$ git clone -b use_new_joint_handle https://github.com/ShotaAk/fake_joint
```
    
Please do the additional work below to build a fake_joint package that is compatible with our ROS2 package.
```bash
$ cd ~/ros2_ws/src
$ cp doosan-robot2/common2/resource/fake_joint_driver_node.cpp fake_joint/fake_joint_driver/src/fake_joint_driver_node.cpp
```
You can install the dependency package through the command below.
```bash
$ cd ~/ros2_ws
$ rosdep install -r --from-paths src --ignore-src --rosdistro foxy -y
$ rm -rf src/doosan-robot2/moveit_config_*/COLCON_IGNORE    # Command to activate the moveit package before colcon build
$ colcon build
$ . install/setup.bash
```
You can run moveit2 with fake_controller from the moveit_config package.
Please refer to the command format below.

> ros2 launch moveit_config_<robot_model> <robot_model>.launch.py
```bash
$ ros2 launch moveit_config_m1013 m1013.launch.py
```
Moveit2 can be executed in conjunction with the control node. Enter the following command to use moveit2's planning function in conjunction with the actual robot.
Refer to the arguments of the __Run dsr_control2 node__ item mentioned above.
```bash
$ ros2 launch dsr_control2 dsr_moveit2.launch.py
```
<img src="https://user-images.githubusercontent.com/47092672/102734069-3f324280-4382-11eb-9165-cdec6b52de17.gif" width="80%">
