# Spider Bot 🕷️ - Open-Source Quadruped Robot

Spider Bot is an open-source quadruped robot designed for robotics enthusiasts, researchers, and developers. Built with **ROS 2** and **micro-ROS**, this project aims to provide an accessible platform for experimenting with quadruped robotics, gait algorithms, and advanced robot control.  

This repository contains all the files, code, and instructions necessary to build, simulate, and control Spider Bot, including **3D printable parts**, **URDF descriptions**, and **microcontroller integration**.

---

## 🎯 Features  
- **Quadruped Design**: 12-DOF structure with three servo-controlled joints per leg.  
- **ROS 2 and micro-ROS Integration**: Support for both desktop and microcontroller environments.  
- **Gazebo Simulation**: Visualize Spider Bot in a physics-based environment.  
- **Inverse Kinematics**: Implemented for smooth gait movement and advanced motion control.  
- **3D Printed Parts**: All parts are designed for 3D printing and can be customized.  
- **ESP32 Controller**: Uses the ESP32 development board for real-time control.  

---

## 📂 Repository Structure  
├── 3d_models/ # STL files for 3D printing Spider Bot parts
├── urdf/ # URDF files for ROS 2 integration
├── scripts/ # Python and C++ scripts for control and simulation
├── micro-ros/ # Code for microcontroller (ESP32)
├── launch/ # ROS 2 launch files for easy simulation and testing
├── docs/ # Documentation and schematics
└── README.md # This file

---

## 🚀 Getting Started  

### Prerequisites  
- **Hardware**:  
  - 12x MG90S Servos  
  - ESP32 Development Board  
  - 3D Printed Parts  
  - Power Supply and Circuit Components  

- **Software**:  
  - ROS 2 Humble  
  - Gazebo (Simulation)  
  - Python 3.8+  
  - micro-ROS Agent  

---

### Installation  

1. **Clone the Repository**:  
   ```bash
   git clone https://parsaHedayati/Robot/new/master.git
   cd spider-bot
 2. setup the workspace
mkdir -p ws_spider/src  
cd ws_spider/src  
ln -s <path_to_spider_bot> ./  
cd ..  
colcon build  
source install/setup.bash 

3.Launch Simulation:

4.Deploy to ESP32:
Follow the steps in the micro-ros/README.md to flash the micro-ROS firmware to your ESP32.


🛠️ Development Status
Spider Bot is under active development. Current focus areas include:

Gait optimization and advanced motion algorithms
Enhanced micro-ROS support for real-time control
Improved Gazebo simulation fidelity
Contributions are welcome!
