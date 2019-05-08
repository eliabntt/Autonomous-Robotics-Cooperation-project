# Autonomous Robotics, a.y. 2018-2019

Devs: Elia Bonetto, Filippo Rigotto

The course's project is about cooperation between robots. Specifically we have to detect some objects on a table, take them with a robotic arm (UR10 + 3-finger Robotiq gripper) and place them over a mobile robot that has to navigate throughout a possibly dynamic enviroment with a narrow passage.

This repo contains all the developed packages necessary to simulate the described challenge.

Every subfolder, one for each package, is provided with a README containing the instructions to run the simulation.

Please note that in the final version all the components work under the means of a (simple) finite state machine: all the infos are inside the [README](g01_challenge/README.md) file placed in the folder `g01_challenge`.
If you want to study the parts separately please checkout at the `HW3-delivery` tag.

## Requirements

- ROS Kinetic + Gazebo
- Ubuntu 16.04
- Python 2
- Packages from APT
    - ros-kinetic-controller-manager
    - ros-kinetic-moveit*
    - ros-kinetic-gazebo-ros-pkgs
    - ros-kinetic-gazebo-ros-control
    - ros-kinetic-joint-state-controller
    - ros-kinetic-position-controllers
    - ros-kinetic-velocity-controllers
    - ros-kinetic-effort-controllers
    - ros-kinetic-joint-trajectory-controller
    - ros-kinetic-usb-cam
    - ros-kinetic-soem
    - ros-kinetic-socketcan-interface
    - ros-kinetic-openni-launch
    - python-pymodbus
- Packages from GitHub
    - [RIVeR-Lab/apriltags_ros](https://github.com/RIVeR-Lab/apriltags_ros)
    - [ros-industrial/universal_robot](https://github.com/ros-industrial/universal_robot)
    - [mrjogo/ur_modern_driver.git](https://github.com/mrjogo/ur_modern_driver) (branch `kinetic_workaround`)
    - [JenniferBuehler/common-sensors](https://github.com/JenniferBuehler/common-sensors)
    - [JenniferBuehler/general-message-pkgs](https://github.com/JenniferBuehler/general-message-pkgs)
    - [shadow-robot/gazebo-pkgs](https://github.com/shadow-robot/gazebo-pkgs) (branch `F_kinetic_c++11`)
    - [pal-robotics/gazebo_ros_link_attacher](https://github.com/pal-robotics/gazebo_ros_link_attacher)
- Other packages not publicly available for the arena setup

## Setup

We will assume a correct and working configuration of ROS inside the folder `~/ros_ws`. If this is different just change it with your folder configuration

```bash
cd ~/ros_ws/src/
git init
git remote add origin https://github.com/eliabntt/Autonomous-Robotics-Cooperation-project.git
git pull origin master
```

Remember to give

```bash
cd ~/ros_ws
catkin_make
```

...otherwise nothing will work.

## Results

The video of the simulation can be found [here](video/one_and_half_run.mp4).

The main problems faced within this challenge are related to the simulation of the physics inside Gazebo, especially related to the moving robot. As you can easily see the movements and the interaction with the other objects are pretty unrealistic. Anyway it was not our duty to provide that and to assess this problems and we could not change that much in other packages. One simple example can be seen [here](video/spinning_wheel.mkv) where we can note the slipping wheel (which is not much repeatable) and the interaction between the robot and the cylinders.
Other problems are AMCL-related, especially in the rotational movements. These problems though are not present in the real version of the robot, even if the poor quality of the laser reads gave us other difficulties.

---

(C) 2018-19 Elia Bonetto and Filippo Rigotto. Released under MIT license (see [LICENSE](LICENSE) file).