import sys
from PyQt5.QtCore import QThread, pyqtSignal, pyqtSlot
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pf_orchard_interfaces.msg import TreeImageData, TreeInfo, TreePosition, StampedFloat
from pf_orchard_interfaces.srv import TreeImageProcessing
from .trunk_data_connection import TrunkDataConnection
import numpy as np
import time 
from std_srvs.srv import Trigger
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool
import utm


class TrunkDataConnectionRosService(TrunkDataConnection):
    """
    Extends the TrunkDataConnection class to connect to get the trunk data from a ROS service
    """
    
    def __init__(self,
                 class_mapping=(1, 2, 0),
                 offset=(0, 0),
                 ):
        """
        Args:
            class_mapping (tuple, optional): The mapping of classes from the trunk width estimation package to this one. Defaults to (1, 2, 0).
            offset (tuple, optional): The offset to apply to the positions. Defaults to (0, 0).
        """
        super().__init__(class_mapping=class_mapping, offset=offset)
        
        self.bridge = CvBridge()
        
    def run(self):
        """
        Extends the run method to first initialize the ROS node and the service client
        """
        rclpy.init(args=None)
        
        self.client_node = Ros2ServiceCaller()
        
        super().run()
        
    
    def get_trunk_data(self, current_msg, return_seg_img=False):
        """
        Overrides the get_trunk_data method to instead call the ros service to get the trunk data
        
        Args:
            current_msg (dict): The current message
            return_seg_img (bool, optional): If True, the segmented image will be returned. Defaults to False.

        Returns:
            tuple: The positions, widths, and class estimates of the trunks
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
        """
        Extracts the positions, widths, and class estimates of the trunks from the TreeImageData message

        Args:
            tree_image_msg (TreeImageData): The TreeImageData message

        Returns:
            tuple: The positions, widths, and class estimates of the trunks
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
    """
    A class to call the ROS2 service to get the trunk data
    """

    def __init__(self):
        super().__init__('trunk_width_estimation_client')
        
        self.client = self.create_client(TreeImageProcessing, 'trunk_width_estimation')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.request = TreeImageProcessing.Request()
        
    def send_request(self, depth_image_msg, rgb_image_msg):
        """
        Sends a request to the service to get the trunk data then waits for the response

        Args:
            depth_image_msg (Image): The depth image message
            rgb_image_msg (Image): The color image message

        Returns:
            TreeImageData: The TreeImageData message
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

class TrunkDataConnectionRosSub(TrunkDataConnectionRosService):
    """
    Extends the TrunkDataConnection class to connect to get the trunk data from a ROS subscriber. Used for the live mode of the app
    """
    trunk_data_signal = pyqtSignal(dict)
    odom_data_signal = pyqtSignal(dict)
    gnss_data_signal = pyqtSignal(dict)

    
    def __init__(self, rgb_image_topic, gnss_topic):
        """
        Args:
            rgb_image_topic (str): The topic to subscribe to for the RGB images
            gnss_topic (str): The topic to subscribe to for the GNSS data
        """
        super().__init__()
        self.node = None

        self.rgb_image_topic = rgb_image_topic
        self.gnss_topic = gnss_topic

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
        self.gnss_subscription = self.node.create_subscription(
            NavSatFix,
            self.gnss_topic,
            self.gnss_callback,
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
        

    def gnss_callback(self, msg):
        """
        Callback for the GNSS data

        Args:
            msg (NavSatFix): The GNSS data message
        """
        lat = msg.latitude
        lon = msg.longitude
        
        if np.isclose(lat, 0) or np.isclose(lon, 0):
            return
        
        utm_coords = utm.from_latlon(lat, lon)        
        # TODO: these should be set automatically somehow
        easting = utm_coords[0] - 293414.01
        northing = utm_coords[1] - 5128452.65
    
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # TODO: these should be set automatically somehow
        if easting > 100 or easting < -25 or northing > 180 or northing < -25:
            self.signal_print_message.emit("GPS data appears out of range")
            print("GPS data appears out of range")
            return

        gnss_data = {"timestamp": timestamp, "easting": easting, "northing": northing}
        self.gnss_data_signal.emit(gnss_data)        

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
        odom_data = {"timestamp": timestamp, "x_odom": odom_data.data}
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
    
    # override some methods to make sure they aren't being called, mostly for debugging
    def handle_request(self, request):
        raise NotImplementedError("This shouldn't be being called1")
    
    def get_trunk_data(self, current_msg, return_seg_img=False):
        raise NotImplementedError("This shouldn't be being called2")
        
    def get_results(self, current_msg, results_dict, results):
        raise NotImplementedError("This shouldn't be being called3")
    



        
