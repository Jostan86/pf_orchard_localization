import numpy as np
import cv2
from typing import Callable, Optional, List
import time

def import_trunk_analyzer(width_estimation_config_file_path):
    from width_estimation import TrunkAnalyzer, TrunkSegmenter
    return TrunkAnalyzer(width_estimation_config_file_path, combine_segmenter=False), TrunkSegmenter(width_estimation_config_file_path)

# class for trunk data connection
class TrunkDataConnection:
    def __init__(self,
                 width_estimation_config_file_path: str = None,
                 seg_image_display_func: Callable[[Optional[np.ndarray]], None] = None,
                 pre_filtered_segmentation_display_func: Callable[[Optional[np.ndarray]], None] = None,
                 original_image_display_func: Callable[[Optional[np.ndarray]], None] = None,
                 class_mapping=(1, 2, 0),
                 offset=(0, 0),
                 message_printer: Callable[[List[str]], None] = None):

        self.init_trunk_analyzer(width_estimation_config_file_path)

        self.class_mapping = class_mapping
        self.seg_image_display_func = seg_image_display_func
        self.pre_filtered_segmentation_display_func = pre_filtered_segmentation_display_func
        self.original_image_display_func = original_image_display_func
        self.offset = offset
        self.message_printer = message_printer

    def init_trunk_analyzer(self, width_estimation_config_file_path):
        if width_estimation_config_file_path is not None:
            self.trunk_analyzer, self.trunk_segmenter = import_trunk_analyzer(width_estimation_config_file_path)

    def get_trunk_data(self, current_msg, return_seg_img=False):
        results_dict, results = self.trunk_segmenter.get_results(current_msg['rgb_image'])

        if self.pre_filtered_segmentation_display_func is not None:
            seg_img_og = results.plot()
        else:
            seg_img_og = None

        positions, widths, class_estimates, seg_img = self.get_results(current_msg, results_dict, results)

        if seg_img is None:
            seg_img = current_msg['rgb_image']

        if self.original_image_display_func is not None:
            self.original_image_display_func(current_msg['rgb_image'])

        if self.pre_filtered_segmentation_display_func is not None:
            self.pre_filtered_segmentation_display_func(seg_img_og)

        if self.seg_image_display_func is not None:
            self.seg_image_display_func(seg_img)

        if not return_seg_img:
            return positions, widths, class_estimates
        else:
            return positions, widths, class_estimates, seg_img

    def get_results(self, current_msg, results_dict, results):

        positions, widths, class_estimates, x_positions_in_image, results_kept = (
            self.trunk_analyzer.pf_helper(current_msg['depth_image'], results_dict=results_dict))

        if results_kept is not None:
            seg_img = results[results_kept].plot()
        else:
            seg_img = current_msg['rgb_image']

        if class_estimates is not None:
            class_estimates = self.remap_classes(class_estimates)

        return positions, widths, class_estimates, seg_img

    def remap_classes(self, class_estimates):
        for i, mapped_class in enumerate(self.class_mapping):
            class_estimates[class_estimates == i] = mapped_class

        return class_estimates

    def print_messages(self, positions, widths):
        messages = []

        msg_str = "Widths: "
        for width in widths:
            width *= 100
            msg_str += str(round(width, 2)) + "cm,  "
        messages.append(msg_str)
        msg_str = "Positions: "
        for position in positions:
            msg_str += "(" + str(round(position[0], 3)) + ", " + str(round(position[1], 3)) + ") "
        messages.append(msg_str)
        messages.append("---")

        if self.message_printer is not None:
            self.message_printer(messages)


class TrunkDataConnectionCachedData(TrunkDataConnection):
    def __init__(self,
                 cached_img_directory,
                 seg_image_display_func: Callable[[Optional[np.ndarray]], None],
                 class_mapping=(1, 2, 0),
                 offset=(0, 0),
                 message_printer: Callable[[List[str]], None] = None,
                 actual_position_plot_func: Callable[[np.ndarray], None] = None):

        super().__init__(seg_image_display_func=seg_image_display_func,
                         class_mapping=class_mapping,
                         offset=offset,
                         message_printer=message_printer)

        self.cached_img_directory = cached_img_directory
        self.actual_position_plot_func = actual_position_plot_func

    def get_trunk_data(self, current_msg):

        if current_msg['data'] is None:
            return None, None, None

        msg_data = current_msg['data']['tree_data']
        positions = np.array(msg_data['positions'])
        widths = np.array(msg_data['widths'])
        class_estimates = np.array(msg_data['classes'], dtype=np.int32)

        if self.actual_position_plot_func is not None:
            actual_position = (current_msg['data']['location_estimate'])
            actual_position_np = np.array([actual_position['x'], actual_position['y']])
            self.actual_position_plot_func(actual_position_np)

        seg_img = self.load_cached_img(current_msg['timestamp'])


        if seg_img is not None and self.seg_image_display_func is not None:
            self.seg_image_display_func(seg_img)

        if class_estimates is not None:
            class_estimates = self.remap_classes(class_estimates)
            self.print_messages(positions, widths)

        return positions, widths, class_estimates

    def load_cached_img(self, time_stamp):
        time_stamp = str(int(1000*time_stamp))
        file_path = self.cached_img_directory + "/" + time_stamp + ".png"
        img = cv2.imread(file_path)
        return img