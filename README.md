# ROS Battery Monitoring
This repository contains utilities for battery monitoring in ROS2 (*Jazzy, Kilted...*) with [ros2_control](https://control.ros.org):

* [battery_state_broadcaster](battery_state_broadcaster) broadcasts BatteryState messages from ros2_control state interfaces. See [it's README](battery_state_broadcaster/README.md) for details.
* [battery_state_rviz_overlay](battery_state_rviz_overlay) converts BatteryState messages to display messages for [rviz 2d overlays](https://github.com/teamspatzenhirn/rviz_2d_overlay_plugins).

Here is how to use it: https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/BatteryStateBroadcaster.md

<img width="1138" height="428" alt="Screenshot from 2026-01-14 10-01-47" src="https://github.com/user-attachments/assets/e85c1073-e3d7-4e26-8f0c-ba1495c39f3d" />

**Note:**
- A [video](https://youtu.be/iWyUaSFRExg) by Josh Newans explains the basics of battery monitoring 
- Install RViz2 prerequisites: `sudo apt install ros-${ROS_DISTRO}-rviz-2d-overlay-plugins`
- This package is used in the following project: [https://github.com/slgrobotics/robots_bringup](https://github.com/slgrobotics/articubot_one/wiki)
- Compiles and works under ROS2 *Jazzy* and *Kilted*
- Example of using it on a robot: [config](https://github.com/slgrobotics/articubot_one/blob/main/robots/seggy/config/controllers.yaml)
[launch](https://github.com/slgrobotics/articubot_one/blob/dev/launch/drive.launch.py#L75)
[rviz](https://github.com/slgrobotics/articubot_one/blob/dev/launch/launch_rviz.launch.py#L32)
[arduino](https://github.com/slgrobotics/robots_bringup/tree/main/Docs/MakeYourOwn#base-control---the-arduino-way)

-------

See my [Main Project Wiki](https://github.com/slgrobotics/articubot_one/wiki) and [Useful Links](https://github.com/slgrobotics/robots_bringup/blob/main/Docs/ROS-Jazzy/README.md#useful-links) for more.
