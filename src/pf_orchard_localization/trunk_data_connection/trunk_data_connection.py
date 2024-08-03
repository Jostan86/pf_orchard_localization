import numpy as np
import cv2
from typing import Callable, Optional, List
import time
from PyQt5.QtCore import QThread, pyqtSignal, pyqtSlot, QObject, QMutex, QWaitCondition
from PyQt5.QtWidgets import QApplication
import socket
import struct
import pickle
import copy

# Function to only import these if they're needed
def import_trunk_analyzer(width_estimation_config_file_path):
    from trunk_width_estimation import TrunkAnalyzer, TrunkSegmenter, PackagePaths
    # TODO: make the config file path an argument that works
    config_file = "width_estimation_config_apple.yaml"
    return TrunkAnalyzer(PackagePaths(config_file), combine_segmenter=False), TrunkSegmenter(PackagePaths(config_file))

class TrunkDataConnection(QThread):
    """
    Thread to get the trunk data from the trunk width estimation package
    """
    
    signal_save_calibration_data = pyqtSignal(dict)
    signal_request_processed = pyqtSignal(object)
    signal_segmented_image = pyqtSignal(object, int)
    signal_unfiltered_image = pyqtSignal(object, int)
    signal_original_image = pyqtSignal(object, int)
    signal_print_message = pyqtSignal(str)
    
    def __init__(self,
                 width_estimation_config_file_path: str = None,
                 class_mapping=(1, 2, 0),
                 offset=(0, 0),
                 ):
        """
        Args:
            width_estimation_config_file_path (str, optional): The path to the width estimation config file. Defaults to None.
            class_mapping (tuple, optional): The mapping of classes for the trunk data. Defaults to (1, 2, 0).
            offset (tuple, optional): The offset to apply to the positions. Defaults to (0, 0).
        """
        super().__init__()

        self.wait_condition = QWaitCondition()
        self.mutex = QMutex()

        self.init_trunk_analyzer(width_estimation_config_file_path)

        self.class_mapping = class_mapping
        self.offset = offset
        
        # Start as -1 so they don't display, they are set externally in the main thread using a signal
        self.original_image_display_num = -1
        self.unfiltered_image_display_num = -1
        self.segmented_image_display_num = -1
        
        self.emitting_save_calibration_data = False
        
        self.positions = None
        self.widths = None
        self.class_estimates = None
        self.seg_img = None
        self.x_positions_in_image = None
        self.results_kept = None
        
        self.current_msg = None        

    def init_trunk_analyzer(self, width_estimation_config_file_path):
        """
        Initialize the trunk analyzer and segmenter
        
        Args:
            width_estimation_config_file_path (str): The path to the width estimation config file"""
        self.mutex.lock()
        self.trunk_analyzer, self.trunk_segmenter = import_trunk_analyzer(width_estimation_config_file_path)
        self.mutex.unlock()

    def run(self):
        """
        The main loop of the thread, waits for trunk data requests and processes them
        """
        while True:
            self.mutex.lock()
            self.wait_condition.wait(self.mutex)
            if self.current_msg is not None:
                result = self.get_trunk_data(self.current_msg, return_seg_img=True)
                
                if not self.for_display_only:
                    self.signal_request_processed.emit(result)
                    
                self.current_msg = None
            self.mutex.unlock()
    
    @pyqtSlot(dict)
    def handle_request(self, request_data):
        """ 
        Receive a request for trunk data and saves the request data
        """
        self.mutex.lock()
        self.current_msg = request_data["current_msg"]
        
        if "for_display_only" in request_data:
            self.for_display_only = request_data["for_display_only"]
        else:
            self.for_display_only = False
        
        self.wait_condition.wakeAll()
        self.mutex.unlock()

    def get_trunk_data(self, current_msg, return_seg_img=False):
        """
        Get the trunk data from the trunk width estimation package

        Args:
            current_msg (dict): The current message
            return_seg_img (bool, optional): Whether to return the segmented image. Defaults to False.

        Returns:
            tuple: The positions, widths, class estimates, and segmented image if return_seg_img is True
        """

        results_dict, results = self.trunk_segmenter.get_results(current_msg['rgb_image'])

        if self.unfiltered_image_display_num != -1:
            seg_img_og = results.plot()
        else:
            seg_img_og = None

        self.get_results(current_msg, results_dict, results)

        if self.seg_img is None:
            self.seg_img = current_msg['rgb_image']

        if self.original_image_display_num != -1:
            self.signal_original_image.emit(current_msg['rgb_image'], self.original_image_display_num)

        if self.unfiltered_image_display_num != -1:
            self.signal_unfiltered_image.emit(seg_img_og, self.unfiltered_image_display_num)
            
        if self.segmented_image_display_num != -1:
            self.signal_segmented_image.emit(self.seg_img, self.segmented_image_display_num)

        if not return_seg_img:
            return self.positions, self.widths, self.class_estimates
        else:
            return self.positions, self.widths, self.class_estimates, self.seg_img

    def get_results(self, current_msg, results_dict, results):
        """
        Get the results from the trunk segmenter and analyzer

        Args:
            current_msg (dict): The current message
            results_dict (dict): The results dictionary from the trunk segmenter
            results (list): The results from the trunk segmenter
        """

        self.positions, self.widths, self.class_estimates, self.x_positions_in_image, self.results_kept = (
            self.trunk_analyzer.get_width_estimation_pf(current_msg['depth_image'], results_dict=results_dict))

        if self.emitting_save_calibration_data:
            calibration_data = copy.deepcopy(current_msg)
            calibration_data['x_positions_in_image'] = copy.deepcopy(self.x_positions_in_image)
            self.signal_save_calibration_data.emit(calibration_data)
                                            
        if self.results_kept is not None:
            self.seg_img = results[self.results_kept].plot()
        else:
            self.seg_img = current_msg['rgb_image']

        if self.class_estimates is not None:
            self.class_estimates = self.remap_classes(self.class_estimates)
    
    def remap_classes(self, class_estimates):
        """
        Remap the classes from the trunk segmenter to the classes used in the localization package

        Args:
            class_estimates (np.array): The class estimates from the trunk segmenter

        Returns:
            np.array: The class estimates with the classes remapped
        """
        class_estimates_copy = class_estimates.copy()
        for i, class_num in enumerate(self.class_mapping):
            class_estimates_copy[class_estimates == i] = class_num

        return class_estimates_copy

    def print_messages(self, positions, widths):
        """
        Print the messages for the positions and widths

        Args:
            positions (np.array): The positions of the trunks
            widths (np.array): The widths of the trunks
        """
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
        
        message = "\n".join(messages)
        self.signal_print_message.emit(message)
            
    
    def set_emitting_save_calibration_data(self, emitting_save_calibration_data):
        """
        Set whether to emit the save calibration data signal

        Args:
            emitting_save_calibration_data (bool): Whether to emit the save calibration data signal
        """
        self.emitting_save_calibration_data = emitting_save_calibration_data

    @pyqtSlot(int)
    def set_segmented_image_display_num(self, segmented_image_display_num):
        """
        Slot to set the display position for the segmented image

        Args:
            segmented_image_display_num (int): The segmented image display number
        """
        self.segmented_image_display_num = segmented_image_display_num
    
    @pyqtSlot(int)
    def set_unfiltered_image_display_num(self, unfiltered_image_display_num):
        """
        Slot to set the display position for the unfiltered image

        Args:
            unfiltered_image_display_num (int): The unfiltered image display number
        """
        self.unfiltered_image_display_num = unfiltered_image_display_num
    
    @pyqtSlot(int)
    def set_original_image_display_num(self, original_image_display_num):
        """
        Slot to set the display position for the original image

        Args:
            original_image_display_num (int): The original image display number
        """
        self.original_image_display_num = original_image_display_num

    


class TrunkDataConnectionCachedData(TrunkDataConnection):
    """
    Extension of the TrunkDataConnection class that uses cached images instead of the bag images
    """

    def __init__(self,
                 cached_img_directory,
                 class_mapping=(1, 2, 0),
                 offset=(0, 0),
                ):
        """
        Args:
            cached_img_directory (str): The directory containing the cached images
            class_mapping (tuple, optional): The mapping of classes for the trunk data. Defaults to (1, 2, 0).
            offset (tuple, optional): The offset to apply to the positions. Defaults to (0, 0).
        """
        super().__init__(class_mapping=class_mapping, offset=offset)

        self.cached_img_directory = cached_img_directory
    
    def init_trunk_analyzer(self, width_estimation_config_file_path):
        """
        Bypass the init_trunk_analyzer method because the trunk analyzer and segmenter are not needed
        """
        pass
    
    def get_trunk_data(self, current_msg, return_seg_img=False):
        """
        Override the get_trunk_data method to get the trunk data from use the cached data files

        Args:
            current_msg (dict): The current message
            return_seg_img (bool, optional): Whether to return the segmented image. Defaults to False.
    
        Returns:
            tuple: The positions, widths, class estimates, and segmented image if return_seg_img is True
        """
        if current_msg['data'] is None:
            return None, None, None

        msg_data = current_msg['data']['tree_data']
        self.positions = np.array(msg_data['positions'])
        self.widths = np.array(msg_data['widths'])
        self.class_estimates = np.array(msg_data['classes'], dtype=np.int32)

        self.seg_img = self.load_cached_img(current_msg['timestamp'])

        if self.segmented_image_display_num != -1:
            self.signal_segmented_image.emit(self.seg_img, self.segmented_image_display_num)

        self.class_estimates = self.remap_classes(self.class_estimates)
        self.print_messages(self.positions, self.widths)

        return self.positions, self.widths, self.class_estimates

    def load_cached_img(self, time_stamp):
        """
        Load the cached image from the cached image directory
        
        Args:
            time_stamp (int): The time stamp of the image

        Returns:
            np.ndarray: The image
        """
        time_stamp = str(int(1000*time_stamp))
        file_path = self.cached_img_directory + "/" + time_stamp + ".png"
        img = cv2.imread(file_path)
        return img


