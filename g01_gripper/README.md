# Homework 2 - UR10 + Gripper

This part of the project needs the `g01_perception` module to successfully work.

## Principal aspects

First the perception part will be launched and two separate topics will become available: one for the objects to be transported, one for the objects that must be avoided. _Custom_ messages have been created for the communication of the tag name together with the poses, so now the `/g01_tags_grab | avoid` topics publish an array of PoseStamped.
The main header of the message contains the reference frame of the readings, `/ world`, and the header of each PoseStamped object contains the name of the tag in the `frame_id` field. When ready, the UR10 will move to the "zero" position with the gripper open; the program then waits to receive messages from topics for at most 5 seconds, then ends.
Perception is a blocking element: without objects on the table the second step of the program is not reached; only at that point the scene is initialized by creating the collision objects over the table. The simulating perimeter walls are initialized only *after* the robot has moved to the "zero" position as the starting position would collide with one of them. In case of a real robot they are instead initialized immediately to avoid possible strange or dangerous behaviour of the arm in the first movement towards the "zero" position.
Each object is associated with a triplet of values ​​representing its dimensions and therefore we can distinguish between cubes, parallelepipeds and prisms. The hexagonal-based parallelepipeds have been modelled for comfort with a square base being as wide as the cube. The triangular section prisms have been modelled through the respective mesh present in the *challenge_arena* package (not included here). Once the list of objects to be moved is obtained, start moving the parallelepipeds, i.e. the most bulky objects, then proceed with cubes and finally with triangular prisms, transporting them over the final area where they will be released. Only the "zero" position is given in angles for each joint, the trajectories of the movements in the *pick and place* routine are calculated on intermediate Cartesian points so as to try to force a linear movement, thus minimizing the loss of time as much as possible.

## Commands (for simulation)

**IMPORTANT**: with the addition of the finite state machine this module depends on marrtino's one so it's no longer possible to use it separately.

Within separate terminals launch: the challenge_arena in Gazebo, the manipulator's planner, apriltag module, perception module and finally this part.

```
roslaunch challenge_arena challenge.launch sim:=true
```

```
roslaunch ur10_platform_challenge_moveit_config ur10_platform_challenge_moveit_planning_execution.launch sim:=true
```

```
roslaunch challenge_arena apriltag.launch sim:=true
```

```
roslaunch g01_perception discover.launch [ids:="[[frame_id],]"] forever:=true sim:=true
```

```
roslaunch g01_gripper grip.launch sim:=true
```

Alternatively to the last two:

```
roslaunch g01_gripper grip_perc.launch sim:=true [ids:="[[frame_id],]"]
```

Rviz visualizer (if desired, not mandatory)
```
roslaunch ur10_platform_challenge_moveit_config moveit_rviz.launch sim:=true
```

### Additional notes

A second launch file makes possible to directly run the perception module from the *pick and place* program.

The movement planning, made in segments, will be repeated if it does not reach a given threshold of feasibility, editing the number of generated intermediate points.
If there still is a failure the movement is aborted and the robot goes back to the initial position (a beginning of failure recovery).

In simulation, it is necessary to forcibly attach the object to the gripper by the means of a fake link.

In simulation we could not use gripper's flag *gSTA* so the control of the grasping status has not been tuned.