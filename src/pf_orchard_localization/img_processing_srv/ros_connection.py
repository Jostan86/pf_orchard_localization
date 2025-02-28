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
from pf_orchard_interfaces.srv import TreeImageProcessing

from pf_orchard_localization import img_processing_srv
from pf_orchard_localization.utils.parameters import ParametersLiveData
from pf_orchard_localization.data_managers import data_msgs

if TYPE_CHECKING:
    ...

import logging
logger = logging.getLogger(__name__)

class Ros2Service(img_processing_srv.DirectPkgConnection):
    """Trunk data connection that retrieves trunk detection data from a ROS2 service.
    
    Sends RGB and depth images to a service for processing and receives detected trunk data.
    """
    
    # overrides DirectPkgConnection
    def __init__(self,
                 class_mapping=(1, 2, 0),
                 offset=(0, 0),
                 ):
        """Initializes the ROS2 service trunk data connector.

        Args:
            class_mapping (tuple): Mapping of class values from service to internal representation
            offset (tuple): X,Y position offset to apply to detected positions
        """
        super().__init__(class_mapping=class_mapping, offset=offset)
        
        self.bridge = CvBridge()
    
    # overrides DirectPkgConnection
    def init_trunk_analyzer(self, width_estimation_config_file_path):
        """Overrides trunk analyzer initialization with empty implementation.
        
        Not needed because processing happens in the ROS service.
        
        Args:
            width_estimation_config_file_path (str): Unused config path
        """
        pass
        
    # overrides DirectPkgConnection (but calls super)
    def run(self):
        """Initializes ROS2 node and starts processing thread.
        
        Creates ROS2 service client for trunk data requests and starts the 
        base class processing loop.
        """
        rclpy.init(args=None)
        
        self.client_node = Ros2ServiceCaller()
        
        super().run()
        
    # overrides DirectPkgConnection
    def get_trunk_data(self, current_msg, return_seg_img=False):
        """Gets trunk detection data by calling a ROS2 service.
        
        Converts images to ROS messages, sends them to the service, and processes
        the returned detection results.
        
        Args:
            current_msg (dict): Message containing RGB and depth images
            return_seg_img (bool): Whether to return segmentation visualization image
            
        Returns:
            tuple: Detected positions, widths, and classes (and optionally segmented image)
        """

        
        rgb_image = current_msg['rgb_image']
        depth_image = current_msg['depth_image']
        
        depth_image_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="passthrough")
        rgb_image_msg = self.bridge.cv2_to_imgmsg(rgb_image, encoding="passthrough")
        
        tree_image_msg = self.client_node.send_request(depth_image_msg, rgb_image_msg)
        
        if tree_image_msg is None and not return_seg_img:
            return None, None, None
        elif tree_image_msg is None and return_seg_img:
            return None, None, None, None        

        self.positions, self.widths, self.class_estimates, self.seg_img = self.tree_image_msg_2_trunk_data(tree_image_msg)
        
        if self.seg_img is None:
            self.seg_img = rgb_image

        if self.original_image_display_num != -1:
            self.signal_original_image.emit(current_msg['rgb_image'], self.original_image_display_num)
            
        if self.segmented_image_display_num != -1:
            self.signal_segmented_image.emit(self.seg_img, self.segmented_image_display_num)
        
        if self.class_estimates is not None:
            self.class_estimates = self.remap_classes(self.class_estimates)

        if not return_seg_img:
            return self.positions, self.widths, self.class_estimates
        else:
            return self.positions, self.widths, self.class_estimates, self.seg_img
        
    def tree_image_msg_2_trunk_data(self, tree_image_msg: TreeImageData):
        """Converts ROS TreeImageData message to internal trunk data format.

        Extracts detection information and segmentation visualization from the
        service response message.

        Args:
            tree_image_msg (TreeImageData): Service response with detection data

        Returns:
            tuple: Tuple containing (positions, widths, classifications, segmented_image)
        """
        
        seg_image = self.bridge.imgmsg_to_cv2(tree_image_msg.segmented_image, desired_encoding="passthrough")
        
        if not tree_image_msg.object_seen:
            return None, None, None, None
        
        tree_positions = []
        widths = []
        class_estimates = []
        
        for tree_data in tree_image_msg.trees:
            widths.append(tree_data.width)
            class_estimates.append(tree_data.classification)
            tree_positions.append([tree_data.position.x, tree_data.position.y])       
        
        return np.array(tree_positions), np.array(widths), np.array(class_estimates), seg_image
    
class Ros2ServiceCaller(Node):
    """ROS2 node that calls the trunk width estimation service.
    
    Provides methods to send image data to the trunk width estimation service
    and receive detection results.
    """

    def __init__(self):
        super().__init__('trunk_width_estimation_client')
        
        self.client = self.create_client(TreeImageProcessing, 'trunk_width_estimation')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.request = TreeImageProcessing.Request()
        
    def send_request(self, depth_image_msg, rgb_image_msg):
        """Sends image data to trunk width service and waits for response.

        Args:
            depth_image_msg (Image): ROS depth image message
            rgb_image_msg (Image): ROS RGB image message

        Returns:
            TreeImageData: Detection results or None if service call failed
        """        
        self.request.depth_image = depth_image_msg
        self.request.color_image = rgb_image_msg
        
        future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            return response.tree_image_data
        else:
            return None

class Ros2Sub(Ros2Service):
    """Trunk data connector that uses ROS2 subscriptions for live data.
    
    Instead of calling the trunk width service directly, subscribes to topics
    where trunk detection results are published. Used for live operation mode.
    """
    trunk_data_signal = pyqtSignal(dict)
    odom_data_signal = pyqtSignal(dict)
    corrected_gnss_data_signal = pyqtSignal(data_msgs.Gnss)
    uncorrected_gnss_data_signal = pyqtSignal(data_msgs.Gnss)

    # overrides Ros2Service
    def __init__(self, data_parameters: ParametersLiveData):
        """Initializes ROS2 subscriber for live trunk data.
        
        Args:
            data_parameters (ParametersLiveData): Configuration parameters for live data
                processing including topic names
        """
        super().__init__()
        self.node = None

        self.rgb_image_topic = data_parameters.rgb_image_topic
        self.gnss_uncorrected_topic = data_parameters.gnss_uncorrected_topic
        self.gnss_corrected_topic = data_parameters.gnss_corrected_topic

    # overrides Ros2Service
    def run(self):
        """Initializes ROS2 node and creates topic subscriptions.
        
        Sets up subscriptions to tree detection results, odometry, and GNSS data,
        and starts the ROS2 spin loop to process incoming messages.
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
        """Converts degrees-minutes format to decimal degrees.
        
        Args:
            dmm (float): Coordinate in degrees-minutes format (DDMM.MMMM)
            
        Returns:
            float: Coordinate in decimal degrees format
        """
        dmm = float(dmm)
        degrees = int(dmm // 100)
        minutes = dmm % 100
        decimal_degrees = degrees + (minutes / 60)
        return decimal_degrees
        

    def gnss_uncorrected_callback(self, msg: NavSatFix):
        """Processes incoming uncorrected GNSS data and emits signal.
        
        Args:
            msg (NavSatFix): Raw GNSS data message
        """
        gnss_msg = data_msgs.Gnss.from_rosbags_msg(msg, corrected=False)
        self.uncorrected_gnss_data_signal.emit(gnss_msg)
    
    def gnss_corrected_callback(self, msg):
        """Processes incoming corrected GNSS data and emits signal.
        
        Args:
            msg (NavSatFix): Corrected GNSS data message
        """
        gnss_msg = data_msgs.Gnss.from_rosbags_msg(msg, corrected=True)
        self.corrected_gnss_data_signal.emit(gnss_msg)

    def tree_image_data_callback(self, tree_image_data):
        """Processes incoming trunk detection data and emits signal.
        
        Converts ROS tree detection message to internal format and emits signal
        with detection results and timestamp.
        
        Args:
            tree_image_data (TreeImageData): Tree detection results message
        """
            
        tree_positions, widths, class_estimates, seg_img = self.tree_image_msg_2_trunk_data(tree_image_data)
                
        if self.class_estimates is not None:
            self.class_estimates = self.remap_classes(self.class_estimates)
        
        trunk_data = {"positions": tree_positions, "widths": widths, "classes": class_estimates}
        timestamp = tree_image_data.header.stamp.sec + tree_image_data.header.stamp.nanosec * 1e-9
        tree_image_data = {"timestamp": timestamp, "trunk_data": trunk_data, "seg_img": seg_img}
        self.trunk_data_signal.emit(tree_image_data)
    
    def rgb_img_msg_callback(self, rgb_img_msg):
        """Processes incoming RGB images and emits display signal.
        
        Converts ROS image message to OpenCV format and emits display signal.
        
        Args:
            rgb_img_msg (Image): ROS RGB image message
        """

        
        rgb_image = self.bridge.imgmsg_to_cv2(rgb_img_msg, desired_encoding="bgr8")
        self.signal_original_image.emit(rgb_image, self.original_image_display_num)
    
    def odom_data_callback(self, odom_data):
        """Processes incoming odometry data and emits signal.
        
        Extracts timestamp and displacement from optical flow odometry message
        and emits signal with the data.
        
        Args:
            odom_data (StampedFloat): Optical flow odometry message
        """
        timestamp = odom_data.header.stamp.sec + odom_data.header.stamp.nanosec * 1e-9
        odom_data = {"timestamp": timestamp, "linear_displacment": odom_data.data}
        self.odom_data_signal.emit(odom_data)
    
    def reset_optical_flow(self):
        """Calls service to reset the optical flow odometry.
        
        Sends request to optical flow node to reset accumulated displacement
        and reports success or failure to the user.
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
        """Publishes convergence status of the particle filter to ROS.
        
        Makes the particle filter convergence status available to other ROS nodes.
        
        Args:
            converged (bool): Whether the particle filter has converged
        """
        msg = Bool()
        msg.data = converged
        self.converged_pub.publish(msg)
                
    def stop(self):
        """Shuts down the ROS2 node.
        
        Properly terminates ROS2 communications when module is stopped.
        """
        rclpy.shutdown()
    
    # override some methods to make sure they aren't being called, mostly for debugging
    def handle_request(self, request):
        raise NotImplementedError("This shouldn't be being called1")
    
    def get_trunk_data(self, current_msg, return_seg_img=False):
        raise NotImplementedError("This shouldn't be being called2")
        
    def get_results(self, current_msg, results_dict, results):
        raise NotImplementedError("This shouldn't be being called3")
    



        
