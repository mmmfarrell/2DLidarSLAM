# 2D Lidar SLAM
By: Michael Farrell

![](example_vid.gif)

The simulation uses Open Scene Graph to display the robot in the maze and to simulate the lidar sensor.
The overall gui uses Qt.

Robot dynamics, kinematics and control based on "Control of Unicycle Type
Robots" by R. Carona, et. al.
[pdf](http://vislab.isr.ist.utl.pt/publications/08-jetc-rcarona-vcontrol.pdf)

2D Lidar SLAM based on google cartographer and the corresponding paper
"Real-Time Loop Closure in 2D LIDAR SLAM" by W. Hess, et. al.
[pdf](https://static.googleusercontent.com/media/research.google.com/en//pubs/archive/45466.pdf).
I implemented the ceres scan
matching for local submaps, but loop closure and pose graph optimiztion are not
implemented.
