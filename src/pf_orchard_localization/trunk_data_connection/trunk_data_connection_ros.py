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

class TrunkDataConnectionRosService(TrunkDataConnection):
    
    def __init__(self,
                 class_mapping=(1, 2, 0),
                 offset=(0, 0),
                 ):
        super().__init__(class_mapping=class_mapping, offset=offset)
        
        self.bridge = CvBridge()
        
    def run(self):
        rclpy.init(args=None)
        
        self.client_node = Ros2ServiceCaller()
        
        super().run()
        
    
    def get_trunk_data(self, current_msg, return_seg_img=False):
        
        rgb_image = current_msg['rgb_image']
        depth_image = current_msg['depth_image']
        
        depth_image_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="passthrough")
        rgb_image_msg = self.bridge.cv2_to_imgmsg(rgb_image, encoding="bgr8")
        
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
    def __init__(self):
        super().__init__('trunk_width_estimation_client')
        
        self.client = self.create_client(TreeImageProcessing, 'trunk_width_estimation')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.request = TreeImageProcessing.Request()
        
    def send_request(self, depth_image_msg, rgb_image_msg):
        
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
    trunk_data_signal = pyqtSignal(dict)
    odom_data_signal = pyqtSignal(dict)
    
    def __init__(self):
        super().__init__()
        self.node = None
        
        # self.original_image_display_num = 0
        # self.segmented_image_display_num = 1

    def run(self):
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
            '/registered/rgb/image',
            self.rgb_img_msg_callback,
            10
        )
        
        self.odom_data_subscription = self.node.create_subscription(
            StampedFloat,
            '/of_odom',
            self.odom_data_callback,
            10
        )
        
        self.reset_optical_flow_client = self.node.create_client(Trigger, 'reset_of_odom')
        while not self.reset_optical_flow_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Optical flow odom service not available, waiting again...')
        
        self.reset_optical_flow()
            
        while rclpy.ok():
            rclpy.spin_once(self.node)

    def tree_image_data_callback(self, tree_image_data):
            
        tree_positions, widths, class_estimates, seg_img = self.tree_image_msg_2_trunk_data(tree_image_data)
                
        if self.class_estimates is not None:
            self.class_estimates = self.remap_classes(self.class_estimates)
        
            # self.signal_segmented_image.emit(seg_img, self.segmented_image_display_num)
        
        trunk_data = {"positions": tree_positions, "widths": widths, "classes": class_estimates}
        timestamp = tree_image_data.header.stamp.sec + tree_image_data.header.stamp.nanosec * 1e-9
        tree_image_data = {"timestamp": timestamp, "trunk_data": trunk_data, "seg_img": seg_img}
        self.trunk_data_signal.emit(tree_image_data)
    
    def rgb_img_msg_callback(self, rgb_img_msg):
        rgb_image = self.bridge.imgmsg_to_cv2(rgb_img_msg, desired_encoding="passthrough")
        self.signal_original_image.emit(rgb_image, self.original_image_display_num)
    
    def odom_data_callback(self, odom_data):
        timestamp = odom_data.header.stamp.sec + odom_data.header.stamp.nanosec * 1e-9
        odom_data = {"timestamp": timestamp, "x_odom": odom_data.data}
        self.odom_data_signal.emit(odom_data)
    
    def reset_optical_flow(self):
        future = self.reset_optical_flow_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is not None:
            if future.result().success:
                self.signal_print_message.emit("Optical flow odometer reset")
            else:
                self.signal_print_message.emit("Could not reset optical flow odometer")
                
    def stop(self):
        rclpy.shutdown()
    
    def handle_request(self, request):
        raise NotImplementedError("This shouldn't be being called1")
    
    def get_trunk_data(self, current_msg, return_seg_img=False):
        raise NotImplementedError("This shouldn't be being called2")
        
    def get_results(self, current_msg, results_dict, results):
        raise NotImplementedError("This shouldn't be being called3")


        
