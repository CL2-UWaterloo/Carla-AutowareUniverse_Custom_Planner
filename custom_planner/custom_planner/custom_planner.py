import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid
from autoware_auto_mapping_msgs.msg import HADMapBin
from autoware_auto_perception_msgs.msg import PredictedObjects, TrafficSignalArray
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from autoware_auto_planning_msgs.msg import Trajectory
from autoware_auto_vehicle_msgs.msg import TurnIndicatorsCommand, HazardLightsCommand
from carla_msgs.msg import CarlaEgoVehicleControl

class MyNode(Node):
    """The base_link frame is used frequently throughout 
    the AV stack, and is the center of the rear axle of the
    vehicle.
    
    - Localization module outputs the map to the base_link transformation.
    - Planning module plans the poses for where the
    base_link frame should be in the future.
    - Control module tries to fit base_link to incoming poses.
    
    Inputs | Topic Name
    1. Vehicle Pose | /tf 
    2. Vehicle Kinematics | /localization/kinematic_state (nav_msgs/msg/Odometry)
    3. Map data | /map/vector_map (autoware_auto_mapping_msgs/msg/HADMapBin)
    4. Detected Object Information | /perception/object_recognition/objects (autoware_auto_perception_msgs/msg/PredictedObjects)
    5. Detected Obstacle Information | /perception/obstacle_segmentation/pointcloud (sensor_msgs/msg/PointCloud2)
    6. Occupancy Map Information | /perception/occupancy_grid_map/map (nav_msgs/msg/OccupancyGrid)
    7. TrafficLight recognition result | /perception/traffic_light_recognition/traffic_signals (autoware_auto_perception_msgs/msg/TrafficSignalArray)
    8. Check point position | /planning/mission_planning/check_point (geometry_msgs/PoseStamped)
    9. Goal position | /planning/mission_planning/goal (geometry_msgs/PoseStamped)
    
    Output | Topic
    1. Trajectory | /planning/trajectory (autoware_auto_planning_msgs/msg/Trajectory)
    2. Turn Signal | /planning/turn_indicators_cmd (autoware_auto_vehicle_msgs/msg/TurnIndicatorsCommand)
    3. Hazard Signal | /planning/hazard_lights_cmd (autoware_auto_vehicle_msgs/msg/HazardLightsCommand)
    """
    def __init__(self):
        super().__init__("custom_planner_test")

        # Publisher decleration
        self.Trajectory = self.create_publisher(Trajectory, "custom_trajectory_topic", 10)
        self.Turn_Signal = self.create_publisher(TurnIndicatorsCommand, "custom_turn_indicator_topic", 10)
        self.Hazard_Signal = self.create_publisher(HazardLightsCommand, "custom_hazard_lights_topic", 10)

        # Subscriber decleration
        self.Vechicle_Kinematics = self.create_subscription(Odometry, "/localization/kinematic_state", self.listener_callback, 10)
        self.Map_Data = self.create_subscription(HADMapBin, "/map/vector_map", self.listener_callback, 10)
        self.Detected_Obj = self.create_subscription(PredictedObjects, "perception/object_recognition/objects", self.listener_callback, 10)
        self.Detected_Obs = self.create_subscription(PointCloud2, "/perception/obstacle_segmentation/pointcloud", self.listener_callback, 10)
        self.occupancy_grid_map_1 = self.create_subscription(OccupancyGrid, "/perception/occupancy_grid_map/map", self.listener_callback, 10)
        self.Traffic_Lights = self.create_subscription(TrafficSignalArray, "/perception/traffic_light_recognition/traffic_signals", self.listener_callback, 10)
        self.Check_Point = self.create_subscription(PoseStamped, "/planning/mission_planning/check_point", self.listener_callback, 10)
        self.Goal = self.create_subscription(PoseStamped, "/planning/mission_planning/goal", self.listener_callback, 10)
        
        # Demo publisher
        self.control_pub = self.create_publisher(CarlaEgoVehicleControl, "/carla/ego_vehicle/vehicle_control_cmd", 10)
        self.create_timer(0.1, self.demo)            
        self.get_logger().info("Hello ROS2")

    def listener_callback(self):
        return true

    def demo(self):
        msg = CarlaEgoVehicleControl()
        msg.throttle = 10.0
        msg.steer = 0.0
        msg.brake = 0.0
        msg.hand_brake = False
        msg.reverse = False
        msg.gear = 1
        msg.manual_gear_shift = False
        self.control_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclp.shutdown()

if __name__ == "__main__":
    main()