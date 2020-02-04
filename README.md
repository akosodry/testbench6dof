# testbench6dof
A free-to-use Robot Operating System (ROS) package that enables both the generation of MARG-based measurements and the testing of filter performances.

Odry, Á.; Kecskes, I.; Sarcevic, P.; Vizvari, Z.; Toth, A.; Odry, P. A Novel Fuzzy-Adaptive Extended Kalman Filter for Real-Time Attitude Estimation of Mobile Robots. Sensors 2020, 20, 803.
https://www.mdpi.com/1424-8220/20/3/803

A ROS-based framework developed for attitude estimation filter development:

This 6 DOF test bench can be utilized to both simulate various (accelerating, non-accelerating, and vibrating) dynamic behaviors and measure the real attitude of the sensor frame, along with the raw MARG data.
It consists of three prismatic joints and three revolute joints. The prismatic joints make the sensor frame slide back and forth, up and down in the three dimensional (3D) space. The revolute joints set the instantaneous attitude (Euler angles) of the sensor frame.
The MARG unit is attached to a plate at the end of this kinematic chain and, so, the 6 DOF system enables both the spatial coordinates and orientation of the sensor frame to be set and measured.
Therefore, a variety of dynamic (vibrating and accelerating) system conditions can be simulated, where both the raw sensor data and real joint states are recorded.
