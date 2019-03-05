# Homework 3 - Marrtino

This homework needs the latest versions of the arena and marrtino packages.

## Principal aspects

Starting from the initial position, the robot uses the planner, loaded with personalized configurations, to reach three intermediate positions and to approach the corridor entrance;
positions are at the end of the open area and not too near the walls, to leave the planner freedom of movement, and they make a curvilinear trajectory, until the start of the corridor.
The corridor is crossed in a manual way following the left wall until the loading zone is reached, where the robot waits to be loaded.
Then, it rotates of 180° to the right; the rotation is not exactly in place, to push away from the left wall, avoiding contacts and localization issues.
The planner is used along with the reached good level of localization to approach the other corridor entrance.
Then, the left wall is followed until the corridor is over.
Marrtino then deviates slightly to the right and returns to the first two intermediate positions (in inverse order), and then goes to the unload zone.

## Commands (for simulation)

Launch Gazebo and rviz in separate shells.

```
roslaunch challenge_arena challenge.launch sim:=true
```

```
rosrun rviz rviz -d `rospack find g01_move`/rviz/marrtino_config.rviz
```

This module can be launched by keeping the navigation part separated (recommended), using two shells:

```
roslaunch g01_move robot_navigation.launch
roslaunch g01_move move.launch sim:=true
```

or with a single command (navigation output goes to log file):

```
roslaunch g01_move move_nav.launch sim:=true
```

**IMPORTANT**: with the addition of the finite state machine, this module depends from the manipulator one and it is no more possible to use it individually
(previous commands do not lead to a complete round).

### Additional notes

Personalized config files for planner and costmaps are found in this module and are automatically used by launch files instead of the arena's default ones.
