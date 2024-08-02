import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from collections import deque
import message_filters
# from .optical_flow_odom import OpticalFlowOdometer
from cv_bridge import CvBridge
from pf_orchard_interfaces.msg import StampedFloat
import cv2
import numpy as np
from scipy.ndimage import median_filter
from std_srvs.srv import Trigger
import os

class RealSenseProcessor(Node):

    def __init__(self):
        super().__init__('of_odom_processor')
        
        self.depth_queue = deque()
        self.rgb_queue = deque()
        
        depth_topic = os.environ.get('DEPTH_IMAGE_TOPIC')
        rgb_topic = os.environ.get('RGB_IMAGE_TOPIC')
        self.depth_sub = message_filters.Subscriber(self, Image, depth_topic)
        self.rgb_sub = message_filters.Subscriber(self, Image, rgb_topic)
        
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.depth_sub, self.rgb_sub], 
            queue_size=20, 
            slop=0.01
        )
        self.ts.registerCallback(self.callback)
        
        self.publisher = self.create_publisher(StampedFloat, 'of_odom', 10)
        self.timer = self.create_timer(0.001, self.process_images)
        
        self.reset_optical_flow_service = self.create_service(Trigger, 'reset_of_odom', self.reset_optical_flow_callback)
        self.processing_images = False
        
        self.cv_bridge = CvBridge()
        
        self.optical_flow_estimator = OpticalFlowOdometer()
        
    def reset_optical_flow_callback(self, req=None, res=None):
        self.optical_flow_estimator = OpticalFlowOdometer()
                
        return Trigger.Response(success=True, message='Optical flow odometer reset')

    def callback(self, depth_msg, rgb_msg):
        self.depth_queue.append(depth_msg)
        self.rgb_queue.append(rgb_msg)


    def process_images(self):
        if self.depth_queue and self.rgb_queue:
            if self.processing_images:
                self.get_logger().warn('Images are still being processed. Skipping this iteration. Also, wtf 14636')
                return
            
            self.processing_images = True
            
            depth_msg = self.depth_queue.popleft()
            rgb_msg = self.rgb_queue.popleft()
            
            if len(self.depth_queue) > 20:
                self.get_logger().warn('Queue too large. Reducing queue size.')
                self.depth_queue.popleft()
                self.rgb_queue.popleft()

            if depth_msg.header.stamp == rgb_msg.header.stamp:
                self.get_logger().info(f'Processing images with timestamp: {depth_msg.header.stamp}')
                self.do_work(depth_msg, rgb_msg)
            else:
                self.get_logger().warn('Timestamps do not match. WTF Why 2356432')
                # self.find_matching_images(depth_msg, rgb_msg)

            self.processing_images = False
    # def find_matching_images(self, depth_msg, rgb_msg):
    #     if len(self.depth_queue) > 1 and len(self.rgb_queue) > 1:
    #         next_depth_msg = self.depth_queue[0]
    #         next_rgb_msg = self.rgb_queue[0]

    #         if next_depth_msg.header.stamp == next_rgb_msg.header.stamp:
    #             self.depth_queue.popleft()
    #             self.rgb_queue.popleft()
    #             self.do_work(next_depth_msg, next_rgb_msg)
    #         else:
    #             self.get_logger().warn('Could not find matching timestamps. Dropping images.')
    #             self.depth_queue.popleft()
    #             self.rgb_queue.popleft()

    def do_work(self, depth_msg, rgb_msg):
        try:
            depth_img = self.cv_bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            rgb_img = self.cv_bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            result = self.optical_flow_estimator.get_odom_estimate(rgb_img, depth_img)
            
            if result is None:
                self.get_logger().warn('Could not get optical flow estimate')
                return
            
            msg = StampedFloat(header=depth_msg.header, data=result)
            self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error processing images: {e}')
            self.reset_optical_flow_callback()
            
        
class OpticalFlowOdometer:
    def __init__(self, intrinsic_matrix=None, scale=0.5):
        self.sift = cv2.SIFT_create()

        if intrinsic_matrix is None:
            # d435 camera intrinsics
            intrinsic_matrix = np.array([[608.389 * scale, 0.0, 428.622 * scale], [0.0, 607.683 * scale, 246.877 * scale], [0, 0, 1]])

            # azure kinect camera intrinsics (full size)
            # intrinsic_matrix = np.array([[902.98, 0.0, 956.55], [0.0, 902.77, 547.68], [0, 0, 1]])
        
        self.scale = scale
        
        self.intrinsic_matrix = intrinsic_matrix
        self.keypoints_previous = None
        self.descriptors_previous = None
        self.depth_image_previous = None

    def filter_depth_image(self, depth_img):
        return median_filter(depth_img, size=5)

    def get_matched_keypoints(self, rgb_img, depth_img):
        # get keypoints and descriptors
        keypoints_2D, descriptors = self.sift.detectAndCompute(rgb_img, None)
        
        if self.keypoints_previous is None or self.descriptors_previous is None or self.depth_image_previous is None:
            self.depth_image_previous = self.filter_depth_image(depth_img)
            self.keypoints_previous = keypoints_2D
            self.descriptors_previous = descriptors
            return None, None
        
        # match descriptors between the two images
        bf = cv2.BFMatcher()
        matches = bf.knnMatch(self.descriptors_previous, descriptors, k=2)

        # Apply Lowe's ratio test to find good matches
        good_matches = []
        for m, n in matches:
            if m.distance < 0.75 * n.distance:
                good_matches.append(m)

        # Convert 2D keypoints to 3D points using depth data
        points_previous_img_3d = []
        points_current_img_2d = []

        for m in good_matches:
            x1, y_matched_keypoint_previous = int(self.keypoints_previous[m.queryIdx].pt[0]), int(self.keypoints_previous[m.queryIdx].pt[1])
            depth_matched_keypoint_previous = self.depth_image_previous[y_matched_keypoint_previous, x1]

            if np.isclose(depth_matched_keypoint_previous, 0):
                continue

            X = (x1 - self.intrinsic_matrix[0, 2]) * depth_matched_keypoint_previous / self.intrinsic_matrix[0, 0]
            Y = (y_matched_keypoint_previous - self.intrinsic_matrix[1, 2]) * depth_matched_keypoint_previous / self.intrinsic_matrix[1, 1]
            Z = depth_matched_keypoint_previous
            points_previous_img_3d.append((X, Y, Z))

            x2, y2 = int(keypoints_2D[m.trainIdx].pt[0]), int(keypoints_2D[m.trainIdx].pt[1])
            points_current_img_2d.append((x2, y2))


        self.depth_image_previous = self.filter_depth_image(depth_img)
        self.keypoints_previous = keypoints_2D
        self.descriptors_previous = descriptors

        points_previous_img_3d = np.ascontiguousarray(points_previous_img_3d, dtype=np.float32)
        points_current_img_2d = np.ascontiguousarray(points_current_img_2d, dtype=np.float32)


        return points_previous_img_3d, points_current_img_2d

    def get_movement_estimate(self, rgb_img, depth_img):
        rgb_img = cv2.resize(rgb_img, (int(rgb_img.shape[1] * self.scale), int(rgb_img.shape[0] * self.scale)))
        depth_img = cv2.resize(depth_img, (int(depth_img.shape[1] * self.scale), int(depth_img.shape[0] * self.scale)))

        points_previous_img_3d, points_current_img_2d = self.get_matched_keypoints(rgb_img, depth_img)
        
        if points_previous_img_3d is None or points_current_img_2d is None:
            print("Need another image to estimate movement")
            return None, None

        # SolvePnP to estimate camera movement
        _, rotation_vector, translation_vector, inliers = cv2.solvePnPRansac(points_previous_img_3d,
                                                                             points_current_img_2d,
                                                                             self.intrinsic_matrix,
                                                                             None,
                                                                             iterationsCount=1000,
                                                                             reprojectionError=4.0,
                                                                             confidence=0.99,)

        if inliers is None:
            # print("No inliers found")
            return None, None

        num_outliers = len(points_previous_img_3d) - len(inliers)
        # print("Number of outliers: {}".format(num_outliers))
        # Convert rotation vector to a rotation matrix
        # rotation_matrix, _ = cv2.Rodrigues(rotation_vector)

        return translation_vector, rotation_vector
    
    def get_odom_estimate(self, rgb_img: np.ndarray, depth_img: np.ndarray):
        translation_vector, rotation_vector = self.get_movement_estimate(rgb_img, depth_img)
        
        if translation_vector is None:
            return None
        
        return translation_vector[0, 0]/1000

def main(args=None):
    rclpy.init(args=args)
    realsense_processor = RealSenseProcessor()
    rclpy.spin(realsense_processor)
    realsense_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
