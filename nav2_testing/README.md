# Nav2 Exploration
ROS2 Navigation2 stack testing tutorials for thesis prep

## Quick start:

```
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/qev3d/mambaforge/envs/driverless_env/share/turtlebot3_gazebo/models
```

```
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
ros2 launch nav2_bringup tb3_simulation_launch.py slam:=True headless:=False
```
