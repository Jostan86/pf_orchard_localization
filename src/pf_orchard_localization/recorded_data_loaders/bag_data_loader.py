#!/usr/bin/env python3
from cv_bridge import CvBridge, CvBridgeError

from pf_orchard_localization.recorded_data_loaders import BaseDataLoader

from pathlib import Path
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

class BagDataLoader(BaseDataLoader):
    def __init__(self, file_path, depth_topic, rgb_topic, odom_topic):

        super().__init__()
       
        self.rosbag_import()
        self.bridge = CvBridge()
        self.depth_topic = depth_topic
        self.rgb_topic = rgb_topic
        self.odom_topic = odom_topic

        self.depth_msg = None
        self.color_msg = None
        self.t_start = None

        self.open_file(file_path)
    
    def rosbag_import(self):
        import rosbag
        self.rosbag = rosbag

    @staticmethod
    def bag_timestamp_to_sec(time):
        return time.to_sec()

    @staticmethod
    def header_timestamp_to_sec(time):
        return time.to_sec()


    def pair_messages(self, d_msg, img_msg):

        if d_msg is not None and img_msg is not None and d_msg.header.stamp == img_msg.header.stamp:
            try:
                depth_image = self.bridge.imgmsg_to_cv2(d_msg, "passthrough")
                color_img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
                time_stamp_img = self.header_timestamp_to_sec(d_msg.header.stamp)

                return depth_image, color_img, time_stamp_img
            except CvBridgeError as e:
                print(e)

        return None, None, None

    def open_file(self, file_path):

        self.current_data_file_path = file_path

        bag_data = self.rosbag.Bag(file_path)

        self.depth_msg = None
        self.color_msg = None

        self.t_start = None
        for topic, msg, t in bag_data.read_messages(topics=[self.rgb_topic, self.depth_topic, self.odom_topic]):
            self.handle_message(topic, msg, t)


    def handle_message(self, topic, msg, t):
            if self.t_start is None:
                self.t_start = self.bag_timestamp_to_sec(t)
            if topic == self.depth_topic:
                self.depth_msg = msg
                depth_img, color_img, time_stamp_img = self.pair_messages(self.depth_msg, self.color_msg)
                if depth_img is not None:
                    self.time_stamps.append(self.bag_timestamp_to_sec(t) - self.t_start)
                    msg = {'topic': 'image', 'rgb_image': color_img, 'depth_image': depth_img,
                           'timestamp': time_stamp_img}
                    self.msg_list.append(msg)
                    self.msg_order.append(1)

            elif topic == self.rgb_topic:
                self.color_msg = msg
                depth_img, color_img, time_stamp_img = self.pair_messages(self.depth_msg, self.color_msg)
                if depth_img is not None:
                    self.time_stamps.append(self.bag_timestamp_to_sec(t) - self.t_start)
                    msg = {'topic': 'image', 'rgb_image': color_img, 'depth_image': depth_img,
                           'timestamp': time_stamp_img}
                    self.msg_list.append(msg)
                    self.msg_order.append(1)

            elif topic == self.odom_topic:
                self.time_stamps.append(self.bag_timestamp_to_sec(t) - self.t_start)
                msg = {'topic': 'odom', 'data': msg}
                self.msg_list.append(msg)
                self.msg_order.append(0)

class Bag2DataLoader(BagDataLoader):
    def __init__(self, file_path, depth_topic, rgb_topic, odom_topic):
        self.typestore = get_typestore(Stores.ROS2_HUMBLE)

        super().__init__(file_path, depth_topic, rgb_topic, odom_topic)
        
    def rosbag_import(self):
        ...
        
    @staticmethod
    def bag_timestamp_to_sec(time):
        return time * 1e-9  # convert from nanoseconds to seconds

    @staticmethod
    def header_timestamp_to_sec(time):
        return time.sec + time.nanosec * 1e-9

    def open_file(self, file_path):

        self.current_data_file_path = file_path
        bagpath = Path(file_path)

        self.depth_msg = None
        self.color_msg = None
        self.t_start = None

        with AnyReader([bagpath], default_typestore=self.typestore) as reader:
            topics = [self.rgb_topic, self.depth_topic, self.odom_topic]
            connections = [x for x in reader.connections if x.topic in topics]
            for connection, t, rawdata in reader.messages(connections=connections):
                msg = reader.deserialize(rawdata, connection.msgtype)
                topic = connection.topic
                self.handle_message(topic, msg, t)

if __name__ == '__main__':
    bag_path = "/media/jostan/portabits/pcl_mod_ros2/envy-trunks-02_6_converted_synced_pcl-mod"
    bag_loader = Bag2DataLoader(bag_path, '/registered/depth/image', '/registered/rgb/image', '/odometry/filtered')

    # bag_path = "/media/jostan/portabits/pcl_mod/envy-trunks-02_6_converted_synced_pcl-mod.bag"
    # bag_loader = BagDataLoader(bag_path, '/registered/depth/image', '/registered/rgb/image', '/odometry/filtered')

    print("Num img msgs: ", bag_loader.num_img_msgs)
    print("Num odom msgs: ", bag_loader.num_odom_msgs)
