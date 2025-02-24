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
    """Extends the TrunkDataConnection class to connect to get the trunk data from a ROS subscriber. Used for the live mode of the app"""

    def __init__(self, data_parameters: ParametersLiveData):
        """
        Args:
            data_parameters (ParametersLiveData): The parameters for the live data version of the app
        """
        super().__init__()
        self.node = None

        self.rgb_image_topic = data_parameters.rgb_image_topic
        self.gnss_uncorrected_topic = data_parameters.gnss_uncorrected_topic
        self.gnss_corrected_topic = data_parameters.gnss_corrected_topic

    def init_ros_node(self):
    # overrides Ros2Service
    def run(self):
        """
        Overrides the run method to instead initialize the ROS node and the subscribers and begin waiting for messages
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
        
    def convert_to_decimal(self, dmm):
        """
        Converts the degrees minutes minutes format to decimal degrees
        
        Args:
            dmm (float): The degrees minutes minutes format
            
        Returns:
            float: The decimal degrees
        """
        dmm = float(dmm)
        degrees = int(dmm // 100)
        minutes = dmm % 100
        decimal_degrees = degrees + (minutes / 60)
        return decimal_degrees
        

    def gnss_uncorrected_callback(self, msg: NavSatFix):
        """Callback for the uncorrected GNSS data"""
        gnss_msg = data_msgs.Gnss.from_rosbags_msg(msg, corrected=False)
        self.uncorrected_gnss_data_signal.emit(gnss_msg)
    
    def gnss_corrected_callback(self, msg):
        """Callback for the corrected GNSS data"""
        gnss_msg = data_msgs.Gnss.from_rosbags_msg(msg, corrected=True)
        self.corrected_gnss_data_signal.emit(gnss_msg)

    def tree_image_data_callback(self, tree_image_data):
        """
        Callback for the tree image data, packages the data and emits a signal with the data

        Args:
            tree_image_data (TreeImageData): The tree image data message
        """
            
        tree_positions, widths, class_estimates, seg_img = self.tree_image_msg_2_trunk_data(tree_image_data)
                
        if self.class_estimates is not None:
            self.class_estimates = self.remap_classes(self.class_estimates)
        
        trunk_data = {"positions": tree_positions, "widths": widths, "classes": class_estimates}
        timestamp = tree_image_data.header.stamp.sec + tree_image_data.header.stamp.nanosec * 1e-9
        tree_image_data = {"timestamp": timestamp, "trunk_data": trunk_data, "seg_img": seg_img}
        self.trunk_data_signal.emit(tree_image_data)
    
    def rgb_img_msg_callback(self, rgb_img_msg):
        """
        Callback for the RGB image data, emits a signal with the RGB image data

        Args:
            rgb_img_msg (Image): The RGB image message
        """

        
        rgb_image = self.bridge.imgmsg_to_cv2(rgb_img_msg, desired_encoding="bgr8")
        self.signal_original_image.emit(rgb_image, self.original_image_display_num)
    
    def odom_data_callback(self, odom_data):
        """
        Callback for the optical flow odometry data, emits a signal with the odometry data

        Args:
            odom_data (StampedFloat): The odometry data message
        """
        timestamp = odom_data.header.stamp.sec + odom_data.header.stamp.nanosec * 1e-9
        odom_data = {"timestamp": timestamp, "linear_displacment": odom_data.data}
        self.odom_data_signal.emit(odom_data)
    
    def reset_optical_flow(self):
        """
        Resets the optical flow odometry
        """
        future = self.reset_optical_flow_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is not None:
            if future.result().success:
                self.signal_print_message.emit("Optical flow odometer reset")
            else:
                self.signal_print_message.emit("Could not reset optical flow odometer")
    
    @pyqtSlot(bool)
    def publish_converged(self, converged):
        """
        Slot to publish a message indicating if the particle filter has converged

        Args:
            converged (bool): If True, the particle filter has converged
        """
        msg = Bool()
        msg.data = converged
        self.converged_pub.publish(msg)
                
    def stop(self):
        rclpy.shutdown()
