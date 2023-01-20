# Carla-AutowareUniverse_Custom_Planner
![1](https://user-images.githubusercontent.com/111143533/213495290-c44d633a-0720-4303-979c-f593ce66a12d.png)

# Requirements
* ROS2 Galactic
* Autoware.Universe
* CARLA 0.9.13 (optional, for visual purposes only)
* Carla-Universe bridge [ref]
# Installation
```cd Carla-AutowareUniverse_Custom_Planner```
```colcon build```
```ros2 launch custom_planner test_launch.xml```
# Verification
First launch the custom planner node, Autoware.Universe AV stack and the CARLA simulator (optional)
```rqt_graph``` should show the custom planner node subscribed and publishing to the appropriate topics associated with Autoware.Universes planning stack.
