# Homework 4 - FSM

## Principal aspects

The finite state machine commands and synchronizes the entire challenge.
It works by message exchange between marrtino and manipulator modules, on the `g01_fsm_state` topic.

When marrtino reaches the start of the corridor, the awake command for the perception part of the manipulator will be published:
tags positions will be read so to optimize execution times (a single reading takes an average of 4-5 seconds in a real environment).
When marrtino enters the loading zone, the manipulator is allowed to move objects.
Marrtino's pose will be used to create an occupancy and poses matrix where objects could be loaded at:
based on the occupancy state of the box at the top of marrtino, the manipulator module will communicate if another round is needed to complete the transfer.
If it is needed, while at the unload zone marrtino will listen on the `g01_start_run` topic before returning to the unload zone.
These behaviors will be repeated until all the requested objects will be transferred.

## Commands (for simulation)

This module contains the two launch file needed to start the entire challenge: 
other than gazebo and rviz, launch first the planner group (apriltags, MoveIt, Marrtino) and then the package's one.

```
roslaunch challenge_arena challenge.launch sim:=true
```

```
rosrun rviz rviz -d `rospack find g01_move`/rviz/marrtino_config.rviz
```

```
roslaunch g01_challenge challenge_planners.launch sim:=true
```

```
roslaunch g01_challenge challenge_packages.launch sim:=true ids:="[[frame_id],]"
```

Frame IDs are the same as the input of the perception module, described in the [README](../g01_perception/README.md).

To start another round, when more objects are needed, use the command

```
rostopic pub --once g01_start_run std_msgs/Bool 'true'
```

It does not work if the task was completed successfully.

### Additional notes

Routines were added for the docking operation at the loading and unloading zones, and to perform a rotation in case another round is needed.
Rotation at the unload point will be executed to the original direction where marrtino came from, so we are quite sure the robot faces an object-free direction.

To avoid localization issues, a dynamic configuration of the goal's angle precision was made for the first and the last checkpoint, where the coming direction cannot be deterministic.
