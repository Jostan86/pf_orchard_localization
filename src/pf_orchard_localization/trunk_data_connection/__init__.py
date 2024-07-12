from .trunk_data_connection import TrunkDataConnection, TrunkDataConnectionCachedData

# only import the ROS2 classes if ROS2 is available
try:
    from .trunk_data_connection_ros import TrunkDataConnectionRosService, TrunkDataConnectionRosSub
except ImportError:
    print("Unable to import TrunkDataConnectionRosService, ROS2 is likely not installed")
    pass