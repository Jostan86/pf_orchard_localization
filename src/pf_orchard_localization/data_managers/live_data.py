from PyQt5.QtCore import QThread, pyqtSignal, pyqtSlot
import numpy as np
from typing import List, Tuple, TYPE_CHECKING

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Bool

from pf_orchard_interfaces.msg import TreeImageData, StampedFloat
from pf_orchard_localization.utils.parameters import ParametersLiveData
from pf_orchard_localization.data_managers import data_msgs

if TYPE_CHECKING:
    ...

import logging
logger = logging.getLogger(__name__)

class Ros2Sub(QThread):
    """ROS2 subscriber for receiving live trunk data.
    
    Connects to various ROS topics to receive trunk data, images, odometry, and GNSS data
    for the live mode of the application.
    """

    def __init__(self, data_parameters: ParametersLiveData):
        """Initialize the ROS2 subscriber.
        
        Args:
            data_parameters (ParametersLiveData): Parameters for the live data version of the app
        """
        super().__init__()
        self.node = None

        self.rgb_image_topic = data_parameters.rgb_image_topic
        self.gnss_uncorrected_topic = data_parameters.gnss_uncorrected_topic
        self.gnss_corrected_topic = data_parameters.gnss_corrected_topic

    def init_ros_node(self):
    # overrides Ros2Service
    def run(self) -> None:
        """Start ROS node and initialize all subscribers.
        
        Overrides QThread's run method to initialize the ROS node, create all required
        subscribers, and begin the ROS event loop.
        """
        rclpy.init()
        self.node = Node('pyqt5_subscriber_node')
        
        self.tree_image_data_subscription = self.node.create_subscription(
            TreeImageData,
            '/tree_image_data',
            self.tree_image_data_callback,
            10
        )
        
        self.rgb_img_subscription = self.node.create_subscription(
            Image,
            self.rgb_image_topic,
            self.rgb_img_msg_callback,
            10
        )
        
        self.odom_data_subscription = self.node.create_subscription(
            StampedFloat,
            '/of_odom',
            self.odom_data_callback,
            10
        )
        self.gnss_uncorrected_subscription = self.node.create_subscription(
            NavSatFix,
            self.gnss_uncorrected_topic,
            self.gnss_uncorrected_callback,
            10)

        self.gnss_corrected_subscription = self.node.create_subscription(
            NavSatFix,
            self.gnss_corrected_topic,
            self.gnss_corrected_callback,
            10)
        
        self.converged_pub = self.node.create_publisher(Bool, 'pf_converged', 10)
        
        self.reset_optical_flow_client = self.node.create_client(Trigger, 'reset_of_odom')
        
        while not self.reset_optical_flow_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Optical flow odom service not available, waiting again...')
        
        self.reset_optical_flow()
            
        while rclpy.ok():
            rclpy.spin_once(self.node)
        
    def convert_to_decimal(self, dmm: float) -> float:
        """Convert degrees-minutes-minutes format to decimal degrees.
        
        Args:
            dmm: GPS coordinate in degrees-minutes format (e.g., 4710.8635)
            
        Returns:
            Decimal degrees (e.g., 47.1811)
        """
        dmm = float(dmm)
        degrees = int(dmm // 100)
        minutes = dmm % 100
        decimal_degrees = degrees + (minutes / 60)
        return decimal_degrees
        

    def gnss_uncorrected_callback(self, msg: NavSatFix) -> None:
        """Process uncorrected GNSS data from ROS topic.
        
        Args:
            msg (NavSatFix): NavSatFix message containing raw GNSS data
        """
        gnss_msg = data_msgs.Gnss.from_rosbags_msg(msg, corrected=False)
        self.uncorrected_gnss_data_signal.emit(gnss_msg)
    
    def gnss_corrected_callback(self, msg: NavSatFix) -> None:
        """Process corrected GNSS data from ROS topic.
        
        Args:
            msg (NavSatFix): NavSatFix message containing corrected GNSS data
        """
        gnss_msg = data_msgs.Gnss.from_rosbags_msg(msg, corrected=True)
        self.corrected_gnss_data_signal.emit(gnss_msg)

    def tree_image_data_callback(self, tree_image_data: TreeImageData) -> None:
        """Process tree image data and emit a signal with the processed data.
        
        Args:
            tree_image_data (TreeImageData): Tree image data from ROS topic
        """
            
        tree_positions, widths, class_estimates, seg_img = self.tree_image_msg_2_trunk_data(tree_image_data)
                
        if self.class_estimates is not None:
            self.class_estimates = self.remap_classes(self.class_estimates)
        
        trunk_data = {"positions": tree_positions, "widths": widths, "classes": class_estimates}
        timestamp = tree_image_data.header.stamp.sec + tree_image_data.header.stamp.nanosec * 1e-9
        tree_image_data = {"timestamp": timestamp, "trunk_data": trunk_data, "seg_img": seg_img}
        self.trunk_data_signal.emit(tree_image_data)
    
    def rgb_img_msg_callback(self, rgb_img_msg: Image) -> None:
        """Process RGB image message and emit signal with the image.
        
        Args:
            rgb_img_msg (Image): RGB image message from ROS topic
        """

        
        rgb_image = self.bridge.imgmsg_to_cv2(rgb_img_msg, desired_encoding="bgr8")
        self.signal_original_image.emit(rgb_image, self.original_image_display_num)
    
    def odom_data_callback(self, odom_data: StampedFloat) -> None:
        """Process optical flow odometry data and emit signal.
        
        Args:
            odom_data (StampedFloat): Odometry data message from ROS topic
        """
        timestamp = odom_data.header.stamp.sec + odom_data.header.stamp.nanosec * 1e-9
        odom_data = {"timestamp": timestamp, "linear_displacment": odom_data.data}
        self.odom_data_signal.emit(odom_data)
    
    def reset_optical_flow(self) -> None:
        """Reset the optical flow odometry by calling ROS service."""
        future = self.reset_optical_flow_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is not None:
            if future.result().success:
                self.signal_print_message.emit("Optical flow odometer reset")
            else:
                self.signal_print_message.emit("Could not reset optical flow odometer")
    
    @pyqtSlot(bool)
    def publish_converged(self, converged: bool) -> None:
        """Publish a ROS message indicating if the particle filter has converged.
        
        Args:
            converged (bool): True if the particle filter has converged
        """
        msg = Bool()
        msg.data = converged
        self.converged_pub.publish(msg)
                
    def stop(self) -> None:
        """Shutdown the ROS node."""
        rclpy.shutdown()
