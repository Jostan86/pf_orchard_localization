#!/usr/bin/env python3
import json
from pf_orchard_localization.recorded_data_loaders import BaseDataLoader

class CachedDataLoader(BaseDataLoader):
    def __init__(self, file_path):
        super().__init__()
        # self.time_stamps_keys = []

        self.open_file(file_path)

    def open_file(self, file_path):
        self.current_data_file_path = file_path

        t_start = None

        loaded_data = json.load(open(file_path))
        # get all the time stamps from the data, which are the keys
        time_stamps_keys = list(loaded_data.keys())
        time_stamps_keys.sort()

        # Find the last None value in loaded_data and remove it, all data after it, and all odom data between it and
        # the previous image
        last_img = 0
        for i, time_stamp_key in enumerate(time_stamps_keys):
            if loaded_data[time_stamp_key] is None:
                continue
            data_keys = list(loaded_data[time_stamp_key].keys())
            if 'tree_data' in data_keys:
                last_img = i

        for i, time_stamp_key in enumerate(time_stamps_keys):
            if i > last_img:
                break
            if t_start is None:
                t_start = float(time_stamp_key)/1000.0
            self.time_stamps.append(float(time_stamp_key)/1000.0 - t_start)
            if loaded_data[time_stamp_key] is None:
                self.msg_order.append(1)
                msg = {'topic': 'image', 'data': loaded_data[time_stamp_key], 'timestamp': float(time_stamp_key)/1000.0}
                self.msg_list.append(msg)
                continue
            data_keys = list(loaded_data[time_stamp_key].keys())
            if 'x_odom' in data_keys:
                msg = {'topic': 'odom', 'data': loaded_data[time_stamp_key], 'timestamp': float(time_stamp_key)/1000.0}
                self.msg_list.append(msg)
                self.msg_order.append(0)
            else:
                self.msg_order.append(1)
                msg = {'topic': 'image', 'data': loaded_data[time_stamp_key], 'timestamp': float(time_stamp_key)/1000.0}
                self.msg_list.append(msg)