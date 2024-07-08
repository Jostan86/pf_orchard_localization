import sys
from PyQt6.QtCore import QThread, pyqtSignal
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pf_orchard_interfaces.msg import TreeImageData, TreeInfo, TreePosition
from pf_orchard_interfaces.srv import TreeImageProcessing
from .trunk_data_connection import TrunkDataConnection
import numpy as np
import time 

class TrunkDataConnectionRos2(TrunkDataConnection):
    
    def __init__(self,
                 class_mapping=(1, 2, 0),
                 offset=(0, 0),
                 ):
        super().__init__(class_mapping=class_mapping, offset=offset)
        
        rclpy.init(args=None)
        
        self.client_node = Ros2ServiceCaller()
    
    def get_trunk_data(self, current_msg, return_seg_img=False):
        
        rgb_image = current_msg['rgb_image']
        depth_image = current_msg['depth_image']
        
        self.positions, self.widths, self.class_estimates, self.seg_img = self.client_node.send_request(depth_image, rgb_image)
        
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
        
        
class Ros2ServiceCaller(Node):
    def __init__(self):
        super().__init__('trunk_width_estimation_client')
        self.client = self.create_client(TreeImageProcessing, 'trunk_width_estimation')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.request = TreeImageProcessing.Request()
        
        self.bridge = CvBridge()

    def send_request(self, depth_image, rgb_image):
        
        depth_image_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="passthrough")
        rgb_image_msg = self.bridge.cv2_to_imgmsg(rgb_image, encoding="bgr8")
        
        self.request.depth_image = depth_image_msg
        self.request.color_image = rgb_image_msg
        
        future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
        else:
            return None, None, None, None
        
        tree_positions = []
        widths = []
        class_estimates = []
        
        for tree_data in response.tree_image_data.trees:
            widths.append(tree_data.width)
            class_estimates.append(tree_data.classification)
            tree_positions.append([tree_data.position.x, tree_data.position.y])
            
        
        seg_image = self.bridge.imgmsg_to_cv2(response.tree_image_data.segmented_image, desired_encoding="passthrough")
        
            
        return np.array(tree_positions), np.array(widths), np.array(class_estimates), seg_image