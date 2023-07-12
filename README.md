# Navigation2+ QUTMS Stack Integration

## Installation

If using Robostack
```
mamba activate driverless_env
mamba install --file conda_requirements
```

If not using Robostack
```
rosdep install --from-paths src --ignore-src -r -y
```

## Building

Assuming the sim and stack have been built already
```
colcon build --symlink-install --packages-up-to qutms_nav2
```

## Running with EUFS Sim

Launch the simulation in one terminal
```
ros2 launch eufs_launcher eufs_launcher.launch.py
```
Launch with `'RViz'`, `'Use Simulated Perception'`, `'Publish Ground Truth'`, `'Lidar'` options


Launch the navigation stack in another terminal
```
ros2 launch fs_nav2 nav2.launch.py
```
