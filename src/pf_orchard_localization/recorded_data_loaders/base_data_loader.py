#!/usr/bin/env python3
import bisect

class BaseDataLoader:
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
        return len(self.msg_list) - sum(self.msg_order)

    @property
    def num_img_msgs(self):
        return sum(self.msg_order)

    @property
    def at_end_of_data(self):
        return self.cur_data_pos >= len(self.msg_list)

    @property
    def at_start_of_data(self):
        return self.cur_data_pos <= 0

    @property
    def at_img_msg(self):
        return self.current_msg['topic'] == 'image'

    @property
    def at_odom_msg(self):
        return self.current_msg['topic'] == 'odom'

    @property
    def current_data_file_time_stamp(self):
        return self.time_stamps[self.cur_data_pos]

    @property
    def current_img_position(self):
        return self.msg_order[:self.cur_data_pos].count(1)

    @property
    def current_odom_position(self):
        return self.msg_order[:self.cur_data_pos].count(0)

    @property
    def current_msg(self):
        return self.msg_list[self.cur_data_pos]

    @property
    def current_data_file_name(self):
        return self.current_data_file_path.split('/')[-1]

    def close(self):
        self.time_stamps = []
        self.cur_data_pos = 0
        self.msg_list = []
        self.time_stamps_img = []
        self.msg_order = []

        self.current_data_file_path = None

        self.reached_end_of_data = False
        self.reached_start_of_data = True

    def get_next_msg(self):
        self.cur_data_pos += 1
        if self.at_end_of_data:
            self.cur_data_pos = len(self.msg_list) - 1
            self.reached_end_of_data = True
            return None
        else:
            self.reached_start_of_data = False
            return self.current_msg

    def get_next_img_msg(self):
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

