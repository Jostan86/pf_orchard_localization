from dataclasses import dataclass
import numpy as np
from enum import Enum
import utm
import cv2
from typing import Union

import logging
logger = logging.getLogger(__name__)

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from nav_msgs.msg import Odometry as RosOdometry
    from sensor_msgs.msg import Imu as RosImu
    from sensor_msgs.msg import NavSatFix as RosGnss
    from builtin_interfaces.msg import Time as RosTime
    from trunk_width_estimation.width_estimation import TrunkAnalyzerData

class MsgType(Enum):
    """Enum for setting the message type"""

    WHEEL_ODOM = 0
    VISUAL_ODOM = 1
    IMAGE = 2
    IMU = 3
    GNSS_UNCORRECTED = 4
    GNSS_CORRECTED = 5
    POSE_ESTIMATE = 6

class Source(Enum):
    """Enum for setting the message source"""

    ROS1BAG = 0  # using ros1 rosbag package
    ROS2BAG = 1  # using python rosbags package
    CACHED = 2

@dataclass
class Timestamp:
    """Class for handling timestamps"""

    sec: int
    nanosec: int
    
    @classmethod
    def from_decimal(cls, decimal_timestamp: float) -> 'Timestamp':
        """Create a timestamp from a decimal value"""
        return cls(int(decimal_timestamp), int((decimal_timestamp % 1) * 1e9))
    
    @classmethod
    def from_ros_time(cls, ros_time: 'RosTime') -> 'Timestamp':
        """Create a timestamp from a ROS time object"""
        return cls(ros_time.sec, ros_time.nanosec)
    
    def to_sec(self) -> float:
        """Convert the timestamp to seconds"""
        return self.sec + self.nanosec / 1e9
    
    def _check_type(self, other) -> None:
        """Check if the other object is a timestamp"""
        if not isinstance(other, Timestamp):
            raise TypeError("Cannot compare Timestamp with {}".format(type(other)))
    
    def __sub__(self, other: 'Timestamp') -> 'Timestamp':
        """Subtract two timestamps"""
        self._check_type(other)
        return Timestamp.from_decimal(self.to_sec() - other.to_sec())
    
    def __add__(self, other: 'Timestamp') -> 'Timestamp':
        """Add two timestamps"""
        self._check_type(other)
        return Timestamp.from_decimal(self.to_sec() + other.to_sec())
    
    def __lt__(self, other: 'Timestamp') -> bool:
        self._check_type(other)
        return self.to_sec() < other.to_sec()
    
    def __le__(self, other: 'Timestamp') -> bool:
        self._check_type(other)
        return self.to_sec() <= other.to_sec()
    
    def __gt__(self, other: 'Timestamp') -> bool:
        self._check_type(other)
        return self.to_sec() > other.to_sec()
    
    def __ge__(self, other: 'Timestamp') -> bool:
        self._check_type(other)
        return self.to_sec() >= other.to_sec()
    
    def __eq__(self, other: 'Timestamp'):
        self._check_type(other)
        return self.sec == other.sec and self.nanosec == other.nanosec
    
    def __ne__(self, other: 'Timestamp'):
        self._check_type(other)
        return self.sec != other.sec or self.nanosec != other.nanosec
    
    def __str__(self):
        """Convert the timestamp to a string"""
        return f"{self.sec}.{str(self.nanosec).zfill(9)}"  
    
@dataclass
class Odom:
    """Class for holding odometry data"""

    msg_timestamp: Timestamp
    angular_velocity: float
    
    message_type: MsgType
    source: Source

    bag_timestamp: Timestamp = None

    linear_velocity: float = None
    linear_displacement: float = None
    
    @classmethod
    def from_ros_wheel_odom(cls, odom_msg: 'RosOdometry', bag_timestamp: Timestamp = None) -> 'Odom':
        """Create an odometry data object from a ROS odometry message

        Args:
            msg (Odometry): The odometry data message
        """
        return cls(msg_timestamp=Timestamp.from_ros_time(odom_msg.header.stamp),
                   bag_timestamp=bag_timestamp,
                   linear_velocity=odom_msg.twist.twist.linear.x,
                   angular_velocity=odom_msg.twist.twist.angular.z,
                   message_type=MsgType.WHEEL_ODOM,
                   source=Source.ROS2BAG)
    
    @classmethod
    def from_visual_odom_data(cls, linear_displacement: float, angular_velocity: float, msg_timestamp: Timestamp, bag_timestamp: Timestamp = None) -> 'Odom':
        """Create an odometry data object from visual odometry data"""

        return cls(msg_timestamp=msg_timestamp,
                   bag_timestamp=bag_timestamp,
                   linear_displacement=linear_displacement,
                   angular_velocity=angular_velocity,
                   message_type=MsgType.VISUAL_ODOM,
                   source=Source.ROS2BAG)
    
    @classmethod
    def from_cached_data(cls, cached_data_dict: dict) -> 'Odom':
        """Create an odometry data object from cached data"""        

        return cls(msg_timestamp=Timestamp.from_decimal(cached_data_dict["msg_timestamp"]),
                   bag_timestamp=Timestamp.from_decimal(cached_data_dict["bag_timestamp"]),
                   linear_velocity=cached_data_dict["linear_velocity"],
                   linear_displacement=cached_data_dict["linear_displacement"],
                   angular_velocity=cached_data_dict["angular_velocity"],
                   message_type=MsgType[cached_data_dict["message_type"]],
                   source=Source.CACHED)
    
    def to_cache_dict(self) -> dict:
        """Convert the odometry data to a dictionary to save to a json file"""

        return {
            "message_type": self.message_type.name,
            "msg_timestamp": self.msg_timestamp.to_sec(),
            "bag_timestamp": self.bag_timestamp.to_sec(),
            "linear_velocity": self.linear_velocity,
            "linear_displacement": self.linear_displacement,
            "angular_velocity": self.angular_velocity,
        }


@dataclass
class Image:
    """Class for storing image data"""

    msg_timestamp: Timestamp
    message_type: MsgType
    source: Source
    
    bag_timestamp: Timestamp = None
    
    depth_image: np.ndarray = None
    rgb_image: np.ndarray = None

    processed: bool = False
    
    object_locations: np.ndarray = None
    object_widths: np.ndarray = None
    object_classes: np.ndarray = None  
    x_positions_in_image: np.ndarray = None 

    segmented_image: np.ndarray = None
    unfiltered_segmented_image: np.ndarray = None

    visualized_depth_image: np.ndarray = None

    @classmethod
    def from_ros_rgbd(cls, rgb_image: np.ndarray, depth_image: np.ndarray, msg_timestamp: Timestamp, bag_timestamp: Timestamp = None) -> 'Image':
        """
        Create an image data object from ROS RGBD image messages

        Args:
            rgb_msg (Image): The RGB image message
            depth_msg (Image): The depth image message
        """
        return cls(msg_timestamp=msg_timestamp,
                   bag_timestamp=bag_timestamp,
                   depth_image=depth_image, 
                   rgb_image=rgb_image, 
                   message_type=MsgType.IMAGE, 
                   source=Source.ROS2BAG)
    
    @classmethod
    def from_cached_data(cls, cached_data_dict: dict) -> 'Image':
        """Create an image data object from cached data"""        

        object_locations = np.array(cached_data_dict["object_locations"]) if cached_data_dict["object_locations"] is not None else None
        object_widths = np.array(cached_data_dict["object_widths"]) if cached_data_dict["object_widths"] is not None else None
        x_positions_in_image = np.array(cached_data_dict["x_positions_in_image"]) if cached_data_dict["x_positions_in_image"] is not None else None
        object_classes = np.array(cached_data_dict["object_classes"]) if cached_data_dict["object_classes"] is not None else None

        return cls(msg_timestamp=Timestamp.from_decimal(cached_data_dict["msg_timestamp"]),
                   bag_timestamp=Timestamp.from_decimal(cached_data_dict["bag_timestamp"]),
                   object_locations=object_locations, 
                   object_widths=object_widths, 
                   x_positions_in_image=x_positions_in_image,
                   object_classes=object_classes, 
                   message_type=MsgType[cached_data_dict["message_type"]], 
                   source=Source.CACHED)
    
    def add_trunk_data(self, trunk_analyzer_data: 'TrunkAnalyzerData'):
        """Add trunk data to the image data"""

        self.object_locations = trunk_analyzer_data.object_locations
        self.object_widths = trunk_analyzer_data.object_widths
        self.x_positions_in_image = trunk_analyzer_data.x_positions_in_image
        self.object_classes = trunk_analyzer_data.classes

        self.processed = True

    def to_cache_dict(self) -> dict:
        """Convert the image data to a dictionary to save to a json file"""

        return {
            "message_type": self.message_type.name,
            "msg_timestamp": self.msg_timestamp.to_sec(),
            "bag_timestamp": self.bag_timestamp.to_sec(),
            "object_locations": self.object_locations.tolist() if self.object_locations is not None else None,
            "object_widths": self.object_widths.tolist() if self.object_widths is not None else None,
            "x_positions_in_image": self.x_positions_in_image.tolist() if self.x_positions_in_image is not None else None,
            "object_classes": self.object_classes.tolist() if self.object_classes is not None else None,
        }
    
    def _load_cached_img(self, cached_img_directory: str):
        """Load the cached image from the cached image directory
        
        Args:
            timestamp (int): The time stamp of the image

        Returns:
            np.ndarray: The image
        """
        file_name = str(self.bag_timestamp) + ".png"
        file_path = cached_img_directory + "/" + file_name
        img = cv2.imread(file_path)
        self.segmented_image = img

@dataclass
class Imu:
    """Class to hold IMU data"""

    msg_timestamp: Timestamp

    orientation_x: float
    orientation_y: float
    orientation_z: float
    orientation_w: float
    acceleration_x: float
    acceleration_y: float
    acceleration_z: float

    message_type: MsgType
    source: Source

    bag_timestamp: Timestamp = None

    @classmethod
    def from_ros_imu(cls, imu_msg: 'RosImu', bag_timestamp: Timestamp = None) -> 'Imu':
        """Create an IMU data object from a ROS IMU message

        Args:
            msg (Imu): The IMU data message
        """
        return cls(msg_timestamp=Timestamp.from_ros_time(imu_msg.header.stamp),
                   bag_timestamp=bag_timestamp,
                   orientation_x=imu_msg.orientation.x,
                   orientation_y=imu_msg.orientation.y,
                   orientation_z=imu_msg.orientation.z,
                   orientation_w=imu_msg.orientation.w,
                   acceleration_x=imu_msg.linear_acceleration.x,
                   acceleration_y=imu_msg.linear_acceleration.y,
                   acceleration_z=imu_msg.linear_acceleration.z,
                   message_type=MsgType.IMU,
                   source=Source.ROS2BAG)
    
    @classmethod
    def from_cached_data(cls, cached_data_dict: dict) -> 'Imu':
        """Create an IMU data object from cached data"""

        return cls(msg_timestamp=Timestamp.from_decimal(cached_data_dict["msg_timestamp"]),
                   bag_timestamp=Timestamp.from_decimal(cached_data_dict["bag_timestamp"]),
                   orientation_x=cached_data_dict["orientation_x"],
                   orientation_y=cached_data_dict["orientation_y"],
                   orientation_z=cached_data_dict["orientation_z"],
                   orientation_w=cached_data_dict["orientation_w"],
                   acceleration_x=cached_data_dict["acceleration_x"],
                   acceleration_y=cached_data_dict["acceleration_y"],
                   acceleration_z=cached_data_dict["acceleration_z"],
                   message_type=MsgType[cached_data_dict["message_type"]],
                   source=Source.CACHED)
    
    def to_cache_dict(self) -> dict:
        """Convert the IMU data to a dictionary to save to a json file"""

        return {
            "message_type": self.message_type.name,
            "msg_timestamp": self.msg_timestamp.to_sec(),
            "bag_timestamp": self.bag_timestamp.to_sec(),
            "orientation_x": self.orientation_x,
            "orientation_y": self.orientation_y,
            "orientation_z": self.orientation_z,
            "orientation_w": self.orientation_w,
            "acceleration_x": self.acceleration_x,
            "acceleration_y": self.acceleration_y,
            "acceleration_z": self.acceleration_z,
        }

@dataclass
class Gnss:
    """Data class for GNSS data"""

    msg_timestamp: Timestamp

    latitude: float
    longitude: float

    map_x: float
    map_y: float

    message_type: MsgType
    source: Source    

    bag_timestamp: Timestamp = None

    @classmethod
    def from_rosbags_msg(cls, gnss_msg: 'RosGnss', bag_timestamp: Timestamp = None, corrected: bool = False) -> 'Gnss':
        """Create a GNSS data object from a ROS GNSS message"""

        latitude = gnss_msg.latitude
        longitude = gnss_msg.longitude

        if np.isclose(latitude, 0) or np.isclose(longitude, 0):
            return
        
        utm_coords = utm.from_latlon(latitude, longitude)        
        # TODO: these should be set automatically somehow
        easting = utm_coords[0]
        map_x = easting - 293414.02 - 0.8
        northing = utm_coords[1] 
        map_y = northing - 5128452.65 - 0.8
    
        # TODO: these should be set automatically somehow
        if map_x > 100 or map_x < -25 or map_y > 180 or map_y < -25:
            logger.warning("GPS data appears out of range")
            return
        
        if corrected:
            message_type = MsgType.GNSS_CORRECTED
        else:
            message_type = MsgType.GNSS_UNCORRECTED

        return cls(msg_timestamp=Timestamp.from_ros_time(gnss_msg.header.stamp),
                   bag_timestamp=bag_timestamp,
                   latitude=latitude,
                   longitude=longitude,
                   map_x=map_x,
                   map_y=map_y,
                   message_type=message_type,
                   source=Source.ROS2BAG)  

    @classmethod
    def from_cached_data(cls, cached_data_dict: dict) -> 'Gnss':
        """Create a GNSS data object from cached data"""        

        return cls(msg_timestamp=Timestamp.from_decimal(cached_data_dict["msg_timestamp"]),
                   bag_timestamp=Timestamp.from_decimal(cached_data_dict["bag_timestamp"]),
                   latitude=cached_data_dict["latitude"],
                   longitude=cached_data_dict["longitude"],
                   map_x=cached_data_dict["map_x"],
                   map_y=cached_data_dict["map_y"],
                   message_type=MsgType[cached_data_dict["message_type"]],
                   source=Source.CACHED)
    
    def to_cache_dict(self) -> dict:
        """Convert the GNSS data to a dictionary to save to a json file"""

        return {
            "message_type": self.message_type.name,
            "msg_timestamp": self.msg_timestamp.to_sec(),
            "bag_timestamp": self.bag_timestamp.to_sec(),
            "latitude": self.latitude,
            "longitude": self.longitude,
            "map_x": self.map_x,
            "map_y": self.map_y,
        }
        

@dataclass
class PoseEstimate:
    """Data class for pose estimate"""

    msg_timestamp: Timestamp
    bag_timestamp: Timestamp

    x: float
    y: float
    theta: float

    message_type: MsgType = MsgType.POSE_ESTIMATE
    source: Source = Source.CACHED

    @classmethod
    def from_particle_pose(cls, particle: np.ndarray, msg_timestamp: Timestamp, bag_timestamp: Timestamp = None) -> 'PoseEstimate':
        """Create a pose estimate object from a particle"""

        return cls(msg_timestamp=msg_timestamp,
                   bag_timestamp=bag_timestamp,
                   x=particle[0],
                   y=particle[1],
                   theta=particle[2])

    @classmethod
    def from_cached_data(cls, cached_data_dict: dict) -> 'PoseEstimate':
        """Create a pose estimate object from cached data"""        

        return cls(msg_timestamp=Timestamp.from_decimal(cached_data_dict["msg_timestamp"]),
                   bag_timestamp=Timestamp.from_decimal(cached_data_dict["bag_timestamp"]),
                   x=cached_data_dict["x"],
                   y=cached_data_dict["y"],
                   theta=cached_data_dict["theta"])
    
    def to_cache_dict(self) -> dict:
        """Convert the pose estimate data to a dictionary to save to a json file"""
        
        return {
            "message_type": self.message_type.name,
            "msg_timestamp": self.msg_timestamp.to_sec(),
            "bag_timestamp": self.bag_timestamp.to_sec(),
            "x": self.x,
            "y": self.y,
            "theta": self.theta,
        }

    def to_numpy(self) -> np.ndarray:
        """Convert the pose estimate to a numpy array"""

        return np.array([self.x, self.y, self.theta])
