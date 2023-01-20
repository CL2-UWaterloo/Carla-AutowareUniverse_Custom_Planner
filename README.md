# Carla-AutowareUniverse_Custom_Planner
![1](https://user-images.githubusercontent.com/111143533/213495290-c44d633a-0720-4303-979c-f593ce66a12d.png)

# Requirements
* ROS2 Galactic
* Autoware.Universe
* CARLA 0.9.13 (optional, for visual purposes only)
* Carla-Universe bridge [ref]
# Installation
Run ```cd Carla-AutowareUniverse_Custom_Planner``` in the terminal.

Run ```colcon build```

Run ```ros2 launch custom_planner test_launch.xml```
# Verification
First launch the custom planner node, Autoware.Universe AV stack and the CARLA simulator (optional)

Run ```rqt_graph``` in the terminal.

A secondary GUI will launch showing the custom planner node subscribed and publishing to the appropriate topics associated with Autoware.Universes planning stack. If you have CARLA running, then you can observe a demo functionality of the custom planner artificially drive the ego vehicle forward.
