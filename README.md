# Autonomous Robotics, a.y. 2018-2019

Devs: Elia Bonetto, Filippo Rigotto

#### Translation still in progress

The course's project is about cooperation between robots. Specifically we have to detect some objects on a table, take them with a robotic arm (UR10 + 3-finger Robotiq gripper) and place them over a mobile robot that has to navigate throughout a possibly dynamic enviroment with a narrow passage.

This repo contains all the developed packages necessary to simulate the described challenge.

Every subfolder, one for each package, is provided with a README containing the instructions to run the simulation.

Please note that in the final version all the components work under the means of a (simple) finite state machine: all the infos are inside the [README](g01_challenge/README.md) file placed in the folder `g01_challenge`.
If you want to study the parts separately please checkout at the `HW3-delivery` tag.

## Requirements

- ROS Kinetic + Gazebo
- Ubuntu 16.04
- Python 2

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

The video of the simulation can be found [here]().
 
The main problems faced within this challenge are related to the simulation of the physics inside Gazebo, especially related to the moving robot. As you can easily see the movements and the interaction with the other objects are pretty unrealistic. Anyway that wasn't our duty to provide that and to assess this problems and we couldn't change that much. One simple example can be seen [here]() or [here]() where we can see the slipping wheel (which is not much repeatable) and the interaction between the robot and the cylinders.
Other problems are AMCL-related, especially in the rotational movements. These problems though are not present in the real version of the robot, even if the poor quality of the laser reads gave us other difficulties.
