# Map My World – Robotics Software Engineering Nanodegree  

This project was developed as part of the **Udacity Robotics Software Engineering Nanodegree**.  
It implements a mobile robot in a custom world, equipped with LIDAR for mapping and localization.  
The system uses **Gazebo simulation, URDF robot description, and ROS navigation stack** to perform **SLAM (Simultaneous Localization and Mapping)** and teleoperation.  

---

## 🚀 Project Overview  
- **Robot Model**: Differential drive robot with a Hokuyo LIDAR sensor.  
- **Simulation Environment**: Custom Gazebo world (`pooyan_world.world`).  
- **Mapping**: Uses the `gmapping` package to build a 2D occupancy grid.  
- **Localization**: Adaptive Monte Carlo Localization (AMCL) for robot pose estimation.  
- **Teleoperation**: Keyboard teleop node for manual robot control.  

---

## 📂 Repository Structure  
Map My World/catkin_ws/src\
│── my_robot/\
│ ├── CMakeLists.txt # Build configuration\
│ ├── package.xml # Package manifest\
│ ├── launch/ # Launch files\
│ │ ├── localization.launch # AMCL localization setup\
│ │ ├── mapping.launch # GMapping SLAM setup\
│ │ ├── robot_description.launch # Loads URDF model\
│ │ └── world.launch # Starts Gazebo world with robot\
│ ├── meshes/\
│ │ └── hokuyo.dae # LIDAR sensor mesh\
│ ├── urdf/\
│ │ ├── my_robot.gazebo # Gazebo-specific plugins\
│ │ └── my_robot.xacro # Robot URDF (description + sensors)\
│ └── worlds/\
│ └── pooyan_world.world # Custom Gazebo world\
│\
└── teleop_twist_keyboard/\
├── teleop_twist_keyboard.py # Python teleop node\
├── CMakeLists.txt\
├── package.xml\
├── CHANGELOG.rst\
└── README.md


---

## ⚙️ Installation  

1. Clone the repository into your ROS workspace:  
```bash
cd ~/catkin_ws/src
git clone https://github.com/pnnayyeri/Robotics_Udacity.git
cd ..
catkin_make
```

2. Source the workspace:
```bash
source devel/setup.bash
```


## 🕹️ Usage
1. Launch the world with the robot
```bash
roslaunch my_robot world.launch
```
2. Run SLAM mapping
```bash
roslaunch my_robot mapping.launch
```
3. Run localization (AMCL)
```bash
roslaunch my_robot localization.launch
```
4. Teleoperate the robot
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```


## 📊 Results
* Mapping Mode: Drive the robot around the world using teleop to generate a map.
* Localization Mode: Use the saved map for AMCL localization.
* The generated occupancy grid can be visualized in RViz.

## 🛠️ Tech Stack
* ROS (Kinetic / Melodic / Noetic depending on setup)
* Gazebo
* RViz
* GMapping (SLAM)
* AMCL (Localization)

## 👤 Author
Pooyan Nayyeri
[LinkedIn](https://www.linkedin.com/in/pnnayyeri/)

## 📜 License
This project is licensed under the MIT License. See [LICENSE](https://opensource.org/license/mit).
