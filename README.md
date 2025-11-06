# ROS Battery Monitoring
This repository contains utilities for battery monitoring in [ROS](https://ros.org/) with [ros2_control](https://control.ros.org):

* [battery_state_broadcaster](battery_state_broadcaster) broadcasts BatteryState messages from ros2_control state interfaces. See [it's README](battery_state_broadcaster/README.md) for details.
* [battery_state_rviz_overlay](battery_state_rviz_overlay) converts BatteryState messages to display messages for [rviz 2d overlays](https://github.com/teamspatzenhirn/rviz_2d_overlay_plugins).

**Note:**
- Install RViz2 prerequisites: `sudo apt install ros-${ROS_DISTRO}-rviz-2d-overlay-plugins`
- This package is used in the following project: https://github.com/slgrobotics/robots_bringup
- Compiles and works under ROS2 *Jazzy* and *Kilted*
