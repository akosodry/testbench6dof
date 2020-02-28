# testbench6dof
A free-to-use Robot Operating System (ROS) package that enables both the generation of MARG-based measurements and the testing of filter performances.

**CITE THIS as:**

**Odry, Á.; Kecskes, I.; Sarcevic, P.; Vizvari, Z.; Toth, A.; Odry, P. A Novel Fuzzy-Adaptive Extended Kalman Filter for Real-Time Attitude Estimation of Mobile Robots. Sensors 2020, 20, 803.**

https://www.mdpi.com/1424-8220/20/3/803


A ROS-based framework developed for attitude estimation filter development:

This 6 DOF test bench can be utilized to both simulate various (accelerating, non-accelerating, and vibrating) dynamic behaviors and measure the real attitude of the sensor frame, along with the raw MARG data.
It consists of three prismatic joints and three revolute joints. The prismatic joints make the sensor frame slide back and forth, up and down in the three dimensional (3D) space. The revolute joints set the instantaneous attitude (Euler angles) of the sensor frame.
The MARG unit is attached to a plate at the end of this kinematic chain and, so, the 6 DOF system enables both the spatial coordinates and orientation of the sensor frame to be set and measured.
Therefore, a variety of dynamic (vibrating and accelerating) system conditions can be simulated, where both the raw sensor data and real joint states are recorded.

Video demo: 

https://youtu.be/Ikv3vJW7KYE

https://youtu.be/iMqh_2s04UI

https://youtu.be/6FHk7J0k-Ck

http://appl-dsp.com/faekf/


The **magdist** folder contains the MATLAB script to generate artificial magnetic disturbances.


How to use my package?
1. Install ROS

http://wiki.ros.org/kinetic/Installation/Ubuntu

2. Setup the environment

http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

3. Setup Gazebo

sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control

4. Install additional packages

sudo apt-get install ros-kinetic-ros-control

sudo apt-get install ros-kinetic-ros-controllers

sudo apt-get install ros-kinetic-diagnostics

sudo apt-get install ros-kinetic-moveit-ros

sudo apt-get install ros-kinetic-moveit

sudo apt-get install ros-kinetic-moveit-visual-tools

5. Download additional packages into the catkin workspace

hector_gazebo from https://github.com/tu-darmstadt-ros-pkg/hector_gazebo

hector_quadrotor from https://github.com/tu-darmstadt-ros-pkg/hector_quadrotor

6. Download my package (testbench6dof) into the catkin workspace

7. Start the ROS+Gazebo environment with

roslaunch testbench6dof spawn_testenv.launch

8. Start the execute_trajectory node with

rosrun testbench6dof exec_traj

9. Real-time plot the results with rqt or save the results with rosbag
