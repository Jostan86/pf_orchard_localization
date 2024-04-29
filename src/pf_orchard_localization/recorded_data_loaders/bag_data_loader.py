#!/usr/bin/env python3
from cv_bridge import CvBridge, CvBridgeError
import rosbag
from pf_orchard_localization.recorded_data_loaders import BaseDataLoader

class BagDataLoader(BaseDataLoader):
    def __init__(self, file_path, depth_topic, rgb_topic, odom_topic):

        super().__init__()

        self.rosbag = rosbag
        self.bridge = CvBridge()
        self.depth_topic = depth_topic
        self.rgb_topic = rgb_topic
        self.odom_topic = odom_topic

        self.open_file(file_path)

    def pair_messages(self, d_msg, img_msg):

        if d_msg is not None and img_msg is not None and d_msg.header.stamp == img_msg.header.stamp:
            try:
                depth_image = self.bridge.imgmsg_to_cv2(d_msg, "passthrough")
                color_img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
                time_stamp_img = d_msg.header.stamp.to_sec()

                return depth_image, color_img, time_stamp_img
            except CvBridgeError as e:
                print(e)

        return None, None, None

    def open_file(self, file_path):

        self.current_data_file_path = file_path

        bag_data = self.rosbag.Bag(file_path)

        depth_msg = None
        color_msg = None

        t_start = None
        for topic, msg, t in bag_data.read_messages(topics=[self.rgb_topic, self.depth_topic, self.odom_topic]):
            if t_start is None:
                t_start = t.to_sec()
            if topic == self.depth_topic:
                depth_msg = msg
                depth_img, color_img, time_stamp_img = self.pair_messages(depth_msg, color_msg)
                if depth_img is not None:
                    self.time_stamps.append(t.to_sec() - t_start)
                    msg = {'topic': 'image', 'rgb_image': color_img, 'depth_image': depth_img,
                           'timestamp': time_stamp_img}
                    self.msg_list.append(msg)
                    self.msg_order.append(1)

            elif topic == self.rgb_topic:
                color_msg = msg
                depth_img, color_img, time_stamp_img = self.pair_messages(depth_msg, color_msg)
                if depth_img is not None:
                    self.time_stamps.append(t.to_sec() - t_start)
                    msg = {'topic': 'image', 'rgb_image': color_img, 'depth_image': depth_img,
                           'timestamp': time_stamp_img}
                    self.msg_list.append(msg)
                    self.msg_order.append(1)

            elif topic == self.odom_topic:
                self.time_stamps.append(t.to_sec() - t_start)
                msg = {'topic': 'odom', 'data': msg}
                self.msg_list.append(msg)
                self.msg_order.append(0)