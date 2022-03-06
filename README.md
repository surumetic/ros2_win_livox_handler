# ROS2 Livox Handler for Windows 

- Unofficial ros2 package for handling livox lidar
- Tested under following environment:
    - windows 11
    - ros2 galactic
    - livox sdk v2.3.0
    - livox avia

# Prerequisite

- Install Livox SDK according to windows install instructions
- Build based on visual studio 2019
- Bring /lib/livox_sdk_static.lib from livox sdk

# Run 

```
run ros2_win_livox_handler ros2_win_livox_handler
```

# rviz2 views

- point color : axis-z

![rviz2_axis](./doc/rviz2_axis.jpg "rviz2 axis based color")

- point color : intensity 
  - each point's instensity is represented as rgb attribute. 

![rviz2_reflectivity](./doc/rviz2_reflectivity.jpg "rviz2 reflectivity based color")

