from PyQt5.QtCore import QThread, pyqtSignal, pyqtSlot, QObject, QMutex, QWaitCondition
import numpy as np
import cv2
from typing import Callable, Optional, List, TYPE_CHECKING
import time
import copy

# from pf_orchard_localization.recorded_data_loaders import MessageType, Timestamp, ImageData
from pf_orchard_localization.data_managers import data_msgs

if TYPE_CHECKING:
    from trunk_width_estimation import TrunkAnalyzer, PackagePaths, TrunkAnalyzerData

import logging
logger = logging.getLogger(__name__)

# Function to only import these if they're needed
def import_trunk_analyzer(config_file):
    from trunk_width_estimation import TrunkAnalyzer, PackagePaths
    TrunkAnalyzer = TrunkAnalyzer
    PackagePaths = PackagePaths
    return TrunkAnalyzer(PackagePaths(config_file), combine_segmenter=True)

def make_trunk_analyzer_data_from_images(rgb_image, depth_image) -> 'TrunkAnalyzerData':
    from trunk_width_estimation import TrunkAnalyzerData
    return TrunkAnalyzerData.from_images(rgb_image, depth_image)

class DirectPkgConnection(QThread):
    """
    Thread to get the trunk data from the trunk width estimation package
    """

    signal_save_calibration_data = pyqtSignal(dict)
    signal_request_processed = pyqtSignal(object)
    signal_print_message = pyqtSignal(str)
    
    def __init__(self,
                 using_cached_data: bool = False,
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

        self.using_cached_data = using_cached_data

        self.trunk_analyzer: TrunkAnalyzer = None
        if not self.using_cached_data:
            self.init_trunk_analyzer(width_estimation_config_file_path)

        self.class_mapping = class_mapping
        self.offset = offset
                
        self.emitting_save_calibration_data = False
        
        self.images_to_include: dict = {'unfiltered': False, 'depth': False}
        
        self.unprocessed_image_data_msg: data_msgs.Image = None        

    def init_trunk_analyzer(self, width_estimation_config_file_path: str):
        """
        Initialize the trunk analyzer and segmenter
        
        Args:
            width_estimation_config_file_path (str): The path to the width estimation config file"""
        self.mutex.lock()
        
        # TODO: should use the actual config file i'd imagine
        config_file = "width_estimation_config_apple.yaml"
        self.trunk_analyzer = import_trunk_analyzer(config_file)

        self.mutex.unlock()

    @pyqtSlot(data_msgs.Image)
    def handle_request(self, image_data_msg: data_msgs.Image):
        """ 
        Receive a request for trunk data and saves the request data
        """
        self.mutex.lock()
        self.unprocessed_image_data_msg = image_data_msg
        
        self.wait_condition.wakeAll()
        self.mutex.unlock()
        
    def run(self):
        """
        The main loop of the thread, waits for trunk data requests and processes them
        """
        while True:
            self.mutex.lock()
            self.wait_condition.wait(self.mutex)

            if not self.using_cached_data:
                processed_img_data_msg = self.get_trunk_data(self.unprocessed_image_data_msg)
            else:
                processed_img_data_msg = self.unprocessed_image_data_msg

            self.signal_request_processed.emit(processed_img_data_msg)
                    
            self.unprocessed_image_data_msg = None

            self.mutex.unlock()

    def get_trunk_data(self, img_data_msg: data_msgs.Image) -> data_msgs.Image:
        """
        Get the trunk data from the trunk width estimation package

        Args:
            img_data_msg (data_msgs.Image): The image data message

        Returns:
            data_msgs.Image: The image data message with the trunk data added
        """
        logger.debug(f"Getting trunk data for image with timestamp: {img_data_msg.bag_timestamp}")

        if self.images_to_include is None:
            logger.error("Images to include not set")
            return None
        
        trunk_analyzer_data = make_trunk_analyzer_data_from_images(img_data_msg.rgb_image, img_data_msg.depth_image)

        trunk_analyzer_data = self.trunk_analyzer.get_width_estimation_pf(trunk_analyzer_data, save_unfiltered_segmentation=self.images_to_include['unfiltered']) 

        img_data_msg.add_trunk_data(trunk_analyzer_data)
        
        img_data_msg.segmented_image = trunk_analyzer_data.visualize_segmentation()
        
        if self.images_to_include['unfiltered']:
            img_data_msg.unfiltered_segmented_image = trunk_analyzer_data.saved_seg
        if self.images_to_include['depth']:
            img_data_msg.visualized_depth_image = trunk_analyzer_data.visualize_depth_image()

        # TODO gotta fix this if i want to do something with the calibration data, i'm not sure what data i 
        if self.emitting_save_calibration_data:
            calibration_data = copy.deepcopy(img_data_msg)
            calibration_data['x_positions_in_image'] = copy.deepcopy(trunk_analyzer_data.x_positions_in_image)
            self.signal_save_calibration_data.emit(calibration_data)

        return img_data_msg

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

    @pyqtSlot(dict)
    def set_images_to_include(self, images_to_include):
        """
        Slot to set the display position for the segmented image

        Args:
            images_to_include (dict): The images to include
        """
        if 'unfiltered' not in images_to_include:
            images_to_include['unfiltered'] = False
        if 'depth' not in images_to_include:
            images_to_include['depth'] = False

        self.images_to_include['unfiltered'] = images_to_include['unfiltered']
        self.images_to_include['depth'] = images_to_include['depth']

        
