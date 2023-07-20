# Navigation2+ QUTMS Stack Integration

## Installation

If using Robostack
```
mamba activate driverless_env
mamba install --file conda_requirements
```

If not using Robostack
```
rosdep install --from-paths QUTMS_Nav_Integration --ignore-src -r -y
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
Checkbox options for running mapping nodes: `'RViz'`, `'Use Simulated Perception'`, `'Publish Ground Truth'`, `'Laserscan'`

