See my [Main Project Wiki](https://github.com/slgrobotics/articubot_one/wiki)

## ROS Battery Monitoring
This repository branch contains RViz plugin for battery monitoring in ROS2 Lyrical:

* [battery_state_rviz_overlay](battery_state_rviz_overlay) converts *BatteryState* messages to display messages for [rviz 2d overlays](https://github.com/teamspatzenhirn/rviz_2d_overlay_plugins).

Port to ROS 2 Lyrical is explained [here](https://github.com/slgrobotics/articubot_one/wiki/Jazzy-to-Lyrical-migration#battery-monitoring).

> **Important:**
> - [`battery_state_broadcaster`](https://github.com/slgrobotics/ros_battery_monitoring/tree/lyrical/battery_state_broadcaster) is retired in Lyrical. It was used in Jazzy/Kilted to broadcast *BatteryState* messages from older *ros2_control* state interfaces.
> - The [`diffdrive_arduino`](https://github.com/slgrobotics/diffdrive_arduino) driver was also updated to work with the latest Battery Broadcaster interfaces.

There is no need to clone this repo branch on the robot, but the workstation still needs it for displaying battery data in RViz - for simulation and real robots:
```
# On the Workstation only:
# - Install RViz2 prerequisites: 
sudo apt install ros-${ROS_DISTRO}-rviz-2d-overlay-plugins

# - Clone the "lyrical" branch to be compiled next to your robot's workstation-side code: 
cd ~/robot_ws/src
git clone https://github.com/slgrobotics/ros_battery_monitoring.git -b lyrical

# - Run Rviz (assuming you cloned other parts of articubot_one project):
cd ~/robot_ws
colcon build
source install/setup.bash
ros2 launch articubot_one launch_rviz.launch.py use_sim_time:=false  # also runs joystick node
```

Here is how to use it: https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/BatteryStateBroadcaster.md

<img width="1138" height="428" alt="Screenshot from 2026-01-14 10-01-47" src="https://github.com/user-attachments/assets/e85c1073-e3d7-4e26-8f0c-ba1495c39f3d" />

### Data flow

For the real Seggy robot, the configured connections are:

| Topic | Publisher | Subscriber / purpose |
|---|---|---|
| `/battery/raw_battery_states` | Official `battery_state_broadcaster` | `battery_state_compat` selects `main_battery` from this array. |
| `/battery/aggregate_state` | Official `battery_state_broadcaster` | Available for diagnostics. |
| `/battery/battery_state` | `battery_state_compat` | `battery_state_rviz_overlay` and `battery_pie_chart_relay`. This is the compatible battery message with percentage on the 0–1 scale. |
| `/battery/percentage` | `battery_pie_chart_relay` (`topic_tools/relay_field`) | Intended for a battery percentage "Pie Chart" display. It contains only the percentage as `std_msgs/Float32`. |
| `/battery_display_text` | `battery_state_rviz_overlay` | RViz’s **Battery Status** text overlay. Contains formatted display text and its appearance settings. |
| `/battery_state_broadcaster/transition_event` | The broadcaster’s lifecycle infrastructure | For lifecycle monitoring tools. |

<img alt="Battery Monitoring" src="https://github.com/user-attachments/assets/deb2a108-0106-4cfe-9560-b0ab25e5b33a" />


**Note:**
- A [video](https://www.youtube.com/watch?v=iWyUaSFRExg&t=567s) by Josh Newans explains the basics of battery monitoring

-------

See my [Main Project Wiki](https://github.com/slgrobotics/articubot_one/wiki) for more.
