#!/usr/bin/env python3
import bisect

class BaseDataLoader:
    """
    Base class for data loaders. The data loader is responsible for loading the data from the recorded data files and
    providing the data to the rest of the system. The data loader should be able to provide the data in the order it was
    recorded, and be able to skip to a specific time stamp in the data.
    """

    def __init__(self):

        self.time_stamps = []
        self.cur_data_pos = 0
        self.msg_list = []
        self.time_stamps_img = []
        self.msg_order = []

        self.current_data_file_path = None

        self.reached_end_of_data = False
        self.reached_start_of_data = True

    @property
    def num_odom_msgs(self):
        """
        Returns the number of odometry messages in the data
        """
        return len(self.msg_list) - sum(self.msg_order)

    @property
    def num_img_msgs(self):
        """
        Returns the number of image messages in the data
        """
        return sum(self.msg_order)

    @property
    def at_end_of_data(self):
        """
        Returns True if the data loader is at the last message in the data
        """
        return self.cur_data_pos >= len(self.msg_list)

    @property
    def at_start_of_data(self):
        """
        Returns True if the data loader is at the first message in the data
        """
        return self.cur_data_pos <= 0

    @property
    def at_img_msg(self):
        """
        Returns True if the current message is an image message
        """
        return self.current_msg['topic'] == 'image'

    @property
    def at_odom_msg(self):
        """
        Returns True if the current message is an odometry message
        """
        return self.current_msg['topic'] == 'odom'

    @property
    def current_data_file_time_stamp(self):
        """
        Returns the time stamp of the current data file
        """
        return self.time_stamps[self.cur_data_pos]

    @property
    def current_img_position(self):
        """
        Returns the position of the current image message relative to the other image messages
        """
        return self.msg_order[:self.cur_data_pos].count(1)

    @property
    def current_odom_position(self):
        """
        Returns the position of the current odometry message relative to the other odometry messages
        """
        return self.msg_order[:self.cur_data_pos].count(0)

    @property
    def current_msg(self):
        """
        Returns the current message
        """
        return self.msg_list[self.cur_data_pos]

    @property
    def current_data_file_name(self):
        """
        Returns the name of the current loaded data file
        """
        return self.current_data_file_path.split('/')[-1]

    def close(self):
        """
        Close a data file
        """
        self.time_stamps = []
        self.cur_data_pos = 0
        self.msg_list = []
        self.time_stamps_img = []
        self.msg_order = []

        self.current_data_file_path = None

        self.reached_end_of_data = False
        self.reached_start_of_data = True

    def get_next_msg(self):
        """
        Get the next message in the data
        
        Returns:
            The next message in the data
        """
        self.cur_data_pos += 1
        if self.at_end_of_data:
            self.cur_data_pos = len(self.msg_list) - 1
            self.reached_end_of_data = True
            return None
        else:
            self.reached_start_of_data = False
            return self.current_msg

    def get_next_img_msg(self):
        """
        Get the next image message in the data, skipping odometry messages

        Returns:
            The next image message in the data
        """
        self.cur_data_pos += 1

        while True:
            if self.at_end_of_data:
                self.reached_end_of_data = True
                self.cur_data_pos = len(self.msg_list) - 1
                return None
            if self.at_img_msg:
                self.reached_start_of_data = False
                return self.current_msg
            else:
                self.cur_data_pos += 1

    def get_prev_img_msg(self):
        """
        Get the previous image message in the data, skipping odometry messages

        Returns:
            The previous image message in the data
        """
        self.cur_data_pos -= 1

        while True:
            if self.at_start_of_data:
                self.reached_start_of_data = True
                self.cur_data_pos = 0
                return None
            if self.at_img_msg:
                self.reached_end_of_data = False
                return self.current_msg
            else:
                self.cur_data_pos -= 1

    def set_time_stamp(self, time_stamp):
        """
        Set the current position in the data to a specific time stamp

        Args:
            time_stamp: The time stamp to set the current position to

        Returns:
            tuple: A message indicating if the time stamp was set successfully, and the image message at the time stamp
        """
        previous_pos = self.cur_data_pos

        # Find the position of the time stamp in the list of time stamps
        time_stamp_pos = bisect.bisect_left(self.time_stamps, time_stamp)

        # Check if the position is valid
        if time_stamp_pos >= len(self.time_stamps):
            return "Time stamp is too large", None

        # Set the current position to the position of the time stamp
        self.cur_data_pos = time_stamp_pos

        img_msg = self.get_next_img_msg()

        if img_msg is None:
            self.cur_data_pos = previous_pos
            return "Time stamp is too large", None
        else:
            self.reached_end_of_data = False
            return "Time stamp set", img_msg

