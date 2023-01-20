# Carla-AutowareUniverse_Custom_Planner
![1](https://user-images.githubusercontent.com/111143533/213495290-c44d633a-0720-4303-979c-f593ce66a12d.png)

# Requirements
* ROS2 Galactic
* Autoware.Universe (galatic branch)
* CARLA 0.9.13 (optional, for visual purposes only)
* Carla-Universe bridge [[ref](https://github.com/CL2-UWaterloo/carla-universe-bridge)]
# Installation
Run ```cd carla-universe-bridge``` in the terminal.

Run ```colcon build```

Run ```source install/setup.bash```

Run ```ros2 launch custom_planner test_launch.xml```
# Verification
First launch the custom planner node by following the installation steps above, launch Autoware.Universe AV stack and the CARLA simulator (optional)

Run ```rqt_graph``` in the terminal.

A secondary GUI will launch showing the custom planner node subscribed and publishing to the appropriate topics associated with Autoware.Universes planning stack. If you have CARLA running with the carla-ros-bridge/autoware.universe bridge [[ref](https://github.com/CL2-UWaterloo/carla-universe-bridge)], then you can observe a demo functionality of the custom planner artificially drive the ego vehicle forward.
