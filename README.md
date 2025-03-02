# robot_arm

```
ros2 launch robot_arm_description robot_arm_launch.py 
```

```
rviz2
```

```
ros2 run robot_arm ik_solver 
```

```
ros2 service call /ik robot_arm_msg/srv/Coordinate "{x: 100, y: 100, z: 100, phi: 0.0}"
```
