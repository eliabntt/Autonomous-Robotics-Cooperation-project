# Homework 2 - MoveIt

## Aspetti principali


## Modalità di funzionamento

```
roslaunch challenge_arena challenge.launch sim:=true
```

```
roslaunch ur10_platform_challenge_moveit_config ur10_platform_challenge_moveit_planning_execution.launch sim:=true
```

```
roslaunch ur10_platform_challenge_moveit_config moveit_rviz.launch sim:=true
```

```
roslaunch challenge_arena apriltags.launch sim:=true
```

```
roslaunch g01_perception discover.launch sim:=true forever:=false
```

```
catkin_make && roslaunch g01_gripper grip.launch forever:=true sim:=true
```

Note aggiuntive

