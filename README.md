# 🤖 RB-Y1 Isaacsim Simulation with ROS2
This repository provides Isaac-based simulation environments for the [Rainbow Robotics RB-Y1](https://www.rainbow-robotics.com/rby1/en) robot with ROS2 integration. It includes simulation environments configured for basic keyboard teleoperation and mobile manipulation tasks.

This project has been tested and is supported on the following environments and software versions:
* Ubuntu 22.04
* ROS2 Humble
* Isaac sim 5.1.0
* Python (>= 3.10)

## 📌 Notable Improvements
We have improved the official RB-Y1 Isaac Sim asset to ensure stable locomotion. 
![RB-Y1 isaacsim issue](https://github.com/RainbowRobotics/rby1-sdk/issues/127)
![Imrovement](assets/improvement.mp4)


## 🌐 IsaacSim Installation
See the official ![Isaacsim documentation](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/quick-install.html#quick-install).


## 💡 ROS2 Interfaces
* ROS2 Topics:


| Message | Description | ROS Topic Name |
|-|-|-|
| Image | RGB image rendered from the head-mounted camera in IsaacSim. The image is at 640/480 resolution with 80 deg VFOV.    | `/rgb` |
| Odometry | Robot base pose and velocity estimated directly from Isaac simulation state. | `/odom` |
| LaserScan | 360° LaserScan in the robot base frame. | `/laser_scan` |
| Twist | Velocity command for differential drive control. | `/cmd_vel` |

### 🖼 Simulation Preview
![Preview](assets/simulation.png)
![Enviroment of Robocup @home](assets/robocup2024.png)

## 🚧 TODOs
[X] Organize Isaacsim-based RB-Y1 simulation core <br>
[X] Keyboard teleop & ROS2 Nav2 pkg integration <br>
[ ] Release dual-arm controller *(in progress)* <br>
[ ] Release system-level RB-Y1 simulation evaluation code

## 🙏 Acknowledgements
This project is built upon the following frameworks and resources:
- [Rainbow Robotics](https://www.rainbow-robotics.com/)
- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [Nav2](https://docs.nav2.org/)
- [YCB Benchmarks - Object and Model Set](https://www.ycbbenchmarks.com/)
- [Isaac Simulation](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/index.html)
