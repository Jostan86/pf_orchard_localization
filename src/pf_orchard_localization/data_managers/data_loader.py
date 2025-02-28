import bisect
import numpy as np
from typing import TYPE_CHECKING, Union, List, Dict, Tuple
from abc import ABC, abstractmethod
import json
from pathlib import Path
from cv_bridge import CvBridge, CvBridgeError
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

from pf_orchard_localization.data_managers import data_msgs
from pf_orchard_localization.utils.parameters import ParametersCachedData, ParametersBagData

if TYPE_CHECKING:
    import rosbag.Bag
    from sensor_msgs.msg import Image
    from nav_msgs.msg import Odometry

import logging
logger = logging.getLogger(__name__)

class Base(ABC):
    """Abstract base class for various data loader implementations.
    
    Provides common functionality for loading and navigating recorded data from different sources.
    Data loaders can provide sequential access to messages and allow seeking to specific timestamps.
    """

    def __init__(self):

        self.timestamp_floats: Union[List[float], np.ndarray] = []
        self.cur_data_pos = 0
        self.msg_list: List[data_msgs.AnyDataMsg] = []
        self.msg_order: Union[List[int], np.ndarray] = []

        self.current_data_file_path = None

        self.reached_end_of_data = False
        self.reached_start_of_data = True

    def num_messages(self, message_type: data_msgs.MsgType) -> int:
        """Counts the number of messages of a specific type in the loaded data.

        Args:
            message_type (data_msgs.MsgType): The message type to count

        Returns:
            int: Number of messages of the specified type
        """
        return np.sum(self.msg_order == message_type.value)
    
    @property
    def num_wheel_odom_msgs(self) -> int:
        """Counts the wheel odometry messages in the loaded data.
        
        Returns:
            int: Number of wheel odometry messages
        """
        return self.num_messages(data_msgs.MsgType.WHEEL_ODOM)

    @property
    def num_visual_odom_msgs(self) -> int:
        """Counts the visual odometry messages in the loaded data.
        
        Returns:
            int: Number of visual odometry messages
        """
        return self.num_messages(data_msgs.MsgType.VISUAL_ODOM)

    @property
    def num_img_msgs(self) -> int:
        """Counts the image messages in the loaded data.
        
        Returns:
            int: Number of image messages
        """
        return self.num_messages(data_msgs.MsgType.IMAGE)
    
    @property
    def num_imu_msgs(self) -> int:
        """Counts the IMU messages in the loaded data.
        
        Returns:
            int: Number of IMU messages
        """
        return self.num_messages(data_msgs.MsgType.IMU)
    
    @property
    def num_gnss_uncorrected_msgs(self) -> int:
        """Counts the uncorrected GNSS messages in the loaded data.
        
        Returns:
            int: Number of uncorrected GNSS messages
        """
        return self.num_messages(data_msgs.MsgType.GNSS_UNCORRECTED)
    
    @property
    def num_gnss_corrected_msgs(self) -> int:
        """Counts the corrected GNSS messages in the loaded data.
        
        Returns:
            int: Number of corrected GNSS messages
        """
        return self.num_messages(data_msgs.MsgType.GNSS_CORRECTED)

    @property
    def at_end_of_data(self) -> bool:
        """Checks if current position is at the end of available data.
        
        Returns:
            bool: True if at or past the last message
        """
        return self.cur_data_pos >= len(self.msg_list)

    @property
    def at_start_of_data(self) -> bool:
        """Checks if current position is at the beginning of data.
        
        Returns:
            bool: True if at or before the first message
        """
        return self.cur_data_pos <= 0
    
    def get_time_relative_to_start(self) -> float:
        """Calculates elapsed time from the start of the dataset.
        
        Returns:
            float: Time in seconds since the first message
        """
        return (self.get_current_data_file_timestamp() - self.msg_list[0].bag_timestamp).to_sec()
    
    def at_message_type(self, message_type: data_msgs.MsgType) -> bool:
        """Checks if current message is of the specified type.

        Args:
            message_type (data_msgs.MsgType): Message type to check against

        Returns:
            bool: True if current message matches the specified type
        """
        return self.current_msg.message_type == message_type

    @property
    def at_img_msg(self) -> bool:
        """Checks if current message is an image.
        
        Returns:
            bool: True if current message is an image message
        """
        return self.at_message_type(data_msgs.MsgType.IMAGE)

    def get_current_data_file_timestamp(self) -> data_msgs.Timestamp:
        """Retrieves timestamp of current message.
        
        Returns:
            data_msgs.Timestamp: Timestamp of current message
        """
        return self.current_msg.bag_timestamp

    def message_type_current_position(self, message_type: data_msgs.MsgType) -> int:
        """Calculates position of current message among messages of the same type.
        
        Args:
            message_type (data_msgs.MsgType): Type of message to count position for
            
        Returns:
            int: Index of current message among messages of specified type
        """
        return np.sum(self.msg_order[:self.cur_data_pos] == message_type.value)
    
    @property
    def current_img_position(self) -> int:
        """Gets the index of current image among all image messages.
        
        Returns:
            int: Index of current image in sequence of image messages
        """
        return self.message_type_current_position(data_msgs.MsgType.IMAGE)

    @property
    def current_msg(self) -> data_msgs.AnyDataMsg:
        """Retrieves the current message at cursor position.
        
        Returns:
            data_msgs.AnyDataMsg: Current message
        """
        return self.msg_list[self.cur_data_pos]

    @property
    def current_data_file_name(self) -> str:
        """Gets the filename of the current data file without path.
        
        Returns:
            str: Filename of the currently loaded data file
        """
        return self.current_data_file_path.split('/')[-1]

    def close(self):
        """Closes the current data file and resets internal state.
        """

        self.timestamp_floats = []
        self.cur_data_pos = 0
        self.msg_list = []
        self.msg_order = []

        self.current_data_file_path: str = None

        self.reached_end_of_data = False
        self.reached_start_of_data = True

    def get_next_msg(self) -> Union[data_msgs.AnyDataMsg, None]:
        """Advances to the next message in the sequence.
        
        Returns:
            Union[data_msgs.AnyDataMsg, None]: The next message, or None if at end
        """
        self.cur_data_pos += 1
        if self.at_end_of_data:
            self.cur_data_pos = len(self.msg_list) - 1
            self.reached_end_of_data = True
            return None
        else:
            self.reached_start_of_data = False
            return self.current_msg

    def get_next_img_msg(self) -> Union[data_msgs.Image, None]:
        """Advances to the next image message, skipping other message types.

        Returns:
            Union[data_msgs.Image, None]: Next image message, or None if no more images
        """
        self.cur_data_pos += 1

        while True:
            if self.at_end_of_data:
                self.reached_end_of_data = True
                self.cur_data_pos = len(self.msg_list) - 1
                return None
            if self.at_message_type(data_msgs.MsgType.IMAGE):
                self.reached_start_of_data = False
                return self.current_msg
            else:
                self.cur_data_pos += 1

    def get_prev_img_msg(self) -> Union[data_msgs.Image, None]:
        """Moves to previous image message, skipping other message types.

        Returns:
            Union[data_msgs.Image, None]: Previous image message, or None if at beginning
        """
        self.cur_data_pos -= 1

        while True:
            if self.at_start_of_data:
                self.reached_start_of_data = True
                self.cur_data_pos = 0
                return None
            if self.at_message_type(data_msgs.MsgType.IMAGE):
                self.reached_end_of_data = False
                return self.current_msg
            else:
                self.cur_data_pos -= 1
    
    def set_file_time_relative_to_start(self, timestamp: float) -> bool:
        """Sets current position to a time offset from dataset start.

        Args:
            timestamp (float): Time in seconds from start of dataset

        Returns:
            bool: True if successfully positioned at requested time
        """
        return self.set_file_time(self.msg_list[0].bag_timestamp.to_sec() + timestamp)

    def set_file_time(self, timestamp: float) -> bool:
        """Sets current position to a specific absolute timestamp.

        Uses binary search to find the message closest to the requested time.

        Args:
            timestamp (float): Absolute timestamp in seconds

        Returns:
            bool: True if successfully positioned at requested time
        """

        previous_pos = self.cur_data_pos

        # Find the position of the time stamp in the list of time stamps
        timestamp_pos = bisect.bisect_left(self.timestamp_floats, timestamp)

        # Check if the position is valid
        if timestamp_pos >= len(self.timestamp_floats):
            logger.error("Time stamp is too large")
            return False

        # Set the current position to the position of the time stamp
        self.cur_data_pos = timestamp_pos

        img_msg = self.get_next_img_msg()

        if img_msg is None:
            self.cur_data_pos = previous_pos
            logger.error("Could not find image message at time stamp, time stamp may be too large")
            return False
        else:
            self.reached_end_of_data = False
            logger.info(f"Set time stamp to {self.get_time_relative_to_start():.2f} seconds")
            return True

    @abstractmethod
    def open_file(self, file_path: str):
        """Opens and loads a data file.
        
        Args:
            file_path (str): Path to the data file to open
        """
        pass


class Ros1Bag(Base):
    """Data loader for ROS1 bag files.
    
    Loads data from ROS1 bag files, parsing messages and converting them to standardized 
    internal data types. Maintains message order and allows navigation by timestamp.
    """

    ros1_bag_opener: "rosbag.Bag" = None
    ros1_bag_opener_loaded: bool = False

    def __init__(self, file_path, data_parameters: ParametersBagData):
        """Initializes ROS1 bag data loader.

        Args:
            file_path (str): Path to the ROS1 bag file
            data_parameters (ParametersBagData): Configuration parameters for data loading
        """

        super().__init__()
       
        if not Ros1Bag.ros1_bag_opener_loaded:
            self.rosbag_import()

        self.bridge = CvBridge()
        self.depth_topic = data_parameters.depth_topic
        self.rgb_topic = data_parameters.rgb_topic
        self.odom_topic = data_parameters.odom_topic
        self.orientation_topic = data_parameters.orientation_topic
        self.gnss_uncorrected_topic = data_parameters.gnss_uncorrected_topic
        self.gnss_corrected_topic = data_parameters.gnss_corrected_topic
        self.every_nth_image = data_parameters.every_nth_image

        self.depth_msg: "Image" = None
        self.rgb_msg: "Image" = None
        # self.t_start = None

        self.image_idx = 0

        self.set_message_source()

        self.open_file(file_path)
    
    def rosbag_import(self):
        if not Ros1Bag.ros1_bag_opener_set:
            import rosbag
            Ros1Bag.ros1_bag_opener = rosbag.Bag
            Ros1Bag.ros1_bag_opener_set = True

    def set_message_source(self):
        """Set the message source to ROS1BAG"""
        self.message_source = data_msgs.Source.ROS1BAG

    @staticmethod
    def bag_timestamp_to_sec(time):
        """Convert the ros1 bag timestamp to seconds

        Args:
            time (rospy.Time): The timestamp to convert
        """
        return time.to_sec()

    def handle_image_message(self, bag_timestamp) -> Union[data_msgs.Image, None]:
        """Pair the depth and image messages together if they have the same timestamp

        Args:
            d_msg (sensor_msgs.msg.Image): The depth image message
            img_msg (sensor_msgs.msg.Image): The image message

        Returns:
            tuple: The depth image, color image, and timestamp
        """
        # Need to wait till there is one of each message type
        if self.depth_msg is None or self.rgb_msg is None:
            return None

        # Only want to pair the messages if they have the same timestamp
        if self.depth_msg.header.stamp != self.rgb_msg.header.stamp:
            return None
        
        self.image_idx += 1
        if self.image_idx % self.every_nth_image != 0:
            return None
        
        try:
            depth_img = self.bridge.imgmsg_to_cv2(self.depth_msg, "passthrough")
            rgb_img = self.bridge.imgmsg_to_cv2(self.rgb_msg, "bgr8")
            timestamp_img = data_msgs.Timestamp.from_ros_time(self.depth_msg.header.stamp)

        except CvBridgeError as e:
            print(e)

        msg = data_msgs.Image.from_ros_rgbd(rgb_img, depth_img, timestamp_img, bag_timestamp=bag_timestamp)

        return msg
        
    def open_file(self, file_path):
        """Open the ros1 bag file and load the data

        Args:
            file_path (str): The path to the rosbag file
        """

        self.current_data_file_path = file_path
        
        bag_data = self.ros1_bag_opener(file_path)        

        # self.t_start = None
        for topic, msg, t in bag_data.read_messages(topics=[self.rgb_topic, self.depth_topic, self.odom_topic]):
            self.handle_message(topic, msg, t)
        
    


    def handle_message(self, topic, msg, t):
        """Handle the message based on the topic

        Args:
            topic (str): The topic of the message
            msg (object): The message data
            t (rospy.Time): The timestamp of the message
        """

        bag_timestamp = data_msgs.Timestamp.from_decimal(self.bag_timestamp_to_sec(t))

        if topic == self.depth_topic:
            self.depth_msg = msg
            
            msg = self.handle_image_message(bag_timestamp=bag_timestamp)
            
            # Checking if the images were successfully paired
            if msg is None:
                return
            
            message_type = data_msgs.MsgType.IMAGE

        elif topic == self.rgb_topic:
            self.rgb_msg = msg
            
            msg = self.handle_image_message(bag_timestamp=bag_timestamp)
            
            # Checking if the images were successfully paired
            if msg is None:
                return
            
            message_type = data_msgs.MsgType.IMAGE

        elif topic == self.odom_topic:
            message_type = data_msgs.MsgType.WHEEL_ODOM
            msg = data_msgs.Odom.from_ros_wheel_odom(msg, bag_timestamp=bag_timestamp)
        
        elif topic == self.orientation_topic:
            message_type = data_msgs.MsgType.IMU
            msg = data_msgs.Imu.from_ros_imu(msg, bag_timestamp=bag_timestamp)

        elif topic == self.gnss_uncorrected_topic:
            message_type = data_msgs.MsgType.GNSS_UNCORRECTED
            msg = data_msgs.Gnss.from_rosbags_msg(msg, corrected=False, bag_timestamp=bag_timestamp)
        
        elif topic == self.gnss_corrected_topic:
            message_type = data_msgs.MsgType.GNSS_CORRECTED
            msg = data_msgs.Gnss.from_rosbags_msg(msg, corrected=True, bag_timestamp=bag_timestamp)
        
        self.msg_list.append(msg)
        self.msg_order.append(message_type.value)
        self.timestamp_floats.append(bag_timestamp.to_sec())

class Ros2Bag(Ros1Bag):
    """Data loader for ROS2 bag files.
    
    Extends Ros1Bag to handle ROS2 bag files with their different format and message types.
    Uses rosbags library to read messages and converts them to standardized internal types.
    """

    def __init__(self, file_path, data_parameters: ParametersBagData):
        """Initializes ROS2 bag data loader.

        Args:
            file_path (str): Path to the ROS2 bag file
            data_parameters (ParametersBagData): Configuration parameters for data loading
                including topic names for various message types
        """
        self.typestore = get_typestore(Stores.ROS2_HUMBLE)

        super().__init__(file_path, data_parameters)

    #overrides Ros1Bag
    def rosbag_import(self):
        pass
    
    #overrides Ros1Bag
    def set_message_source(self):
        """
        Set the message source to ROS2BAG
        """
        self.message_source = data_msgs.Source.ROS2BAG    
    
    #overrides Ros1Bag
    @staticmethod
    def bag_timestamp_to_sec(timestamp):
        """
        Convert the rosbag timestamp to seconds
        
        Args:
            time: The timestamp to convert
        """
        return timestamp * 1e-9  # convert from nanoseconds to seconds

    #overrides Ros1Bag
    def open_file(self, file_path):
        """
        Override the open_file method to handle ros2 bag files

        Args:
            file_path: The path to the rosbag file
        """
        self.current_data_file_path = file_path
        bagpath = Path(file_path)

        
        # self.t_start = None

        with AnyReader([bagpath], default_typestore=self.typestore) as reader:
            topics = [self.rgb_topic, self.depth_topic, self.odom_topic, self.orientation_topic, self.gnss_uncorrected_topic, self.gnss_corrected_topic]
            connections = [x for x in reader.connections if x.topic in topics]
            for connection, t, rawdata in reader.messages(connections=connections):
                msg = reader.deserialize(rawdata, connection.msgtype)
                topic = connection.topic
                self.handle_message(topic, msg, t)
        
        self.msg_order = np.array(self.msg_order)
        self.timestamp_floats = np.array(self.timestamp_floats)


class Cached(Base):
    """Data loader for cached JSON data files.
    
    Loads data from previously saved JSON files that contain preprocessed messages.
    Can load associated image files from a separate directory structure.
    """
    
    def __init__(self, file_path: str, data_parameters: ParametersCachedData):
        """Initializes cached data loader.

        Args:
            file_path (str): Path to the cached JSON data file
            data_parameters (ParametersCachedData): Configuration parameters including image directory
        """
        super().__init__()
        # self.timestamps_keys = []

        self.cached_img_directory = data_parameters.cached_image_dir
        self.open_file(file_path)

    def open_file(self, file_path: str):
        """Opens and parses a cached JSON data file.
        
        Loads the JSON file and converts entries to appropriate message types.
        Also handles loading associated images if needed.
        
        Args:
            file_path (str): Path to the cached JSON data file
        """
        logger.info(f"Opening cached data file: {file_path}")

        self.current_data_file_path = file_path

        loaded_data_msgs: List[data_msgs.AnyDataMsg] = json.load(open(file_path))

        for loaded_data_msg in loaded_data_msgs:
            
            data_msg: data_msgs.AnyDataMsg = None

            message_type = data_msgs.MsgType[loaded_data_msg["message_type"]]

            if message_type == data_msgs.MsgType.IMAGE:
                data_msg = data_msgs.Image.from_cached_data(loaded_data_msg)
                data_msg._load_cached_img(self.cached_img_directory)
            elif message_type == data_msgs.MsgType.WHEEL_ODOM or message_type == data_msgs.MsgType.VISUAL_ODOM:
                data_msg = data_msgs.Odom.from_cached_data(loaded_data_msg)
            elif message_type == data_msgs.MsgType.GNSS_CORRECTED or message_type == data_msgs.MsgType.GNSS_UNCORRECTED:
                data_msg = data_msgs.Gnss.from_cached_data(loaded_data_msg)
            elif message_type == data_msgs.MsgType.IMU:
                data_msg = data_msgs.Imu.from_cached_data(loaded_data_msg)
            elif message_type == data_msgs.MsgType.POSE_ESTIMATE:
                data_msg = data_msgs.PoseEstimate.from_cached_data(loaded_data_msg)
                
            if data_msg is not None:
                self.msg_order.append(data_msg.message_type.value)
                self.msg_list.append(data_msg)
                self.timestamp_floats.append(data_msg.bag_timestamp.to_sec())

        self.msg_order = np.array(self.msg_order)
        self.timestamp_floats = np.array(self.timestamp_floats)
