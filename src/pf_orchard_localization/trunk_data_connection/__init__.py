from .trunk_data_connection import TrunkDataConnection, TrunkDataConnectionCachedData

# only import the ROS2 classes if ROS2 is available
try:
    from .trunk_data_connection_ros2 import TrunkDataConnectionRos2
except ImportError:
    print("Unable to import TrunkDataConnectionRos2, ROS2 is likely not installed")
    pass