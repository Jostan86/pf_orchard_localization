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
    """Thread for processing trunk width estimation requests.
    
    Processes images to detect trees and estimate trunk widths using the trunk_width_estimation package.
    Runs in a separate thread to prevent blocking the GUI during image processing.
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
        """Initializes trunk width estimation thread.
        
        Args:
            using_cached_data (bool): Whether using cached data rather than processing images
            width_estimation_config_file_path (str): Path to configuration file for trunk width estimation
            class_mapping (tuple): Mapping of object classification values between systems
            offset (tuple): X,Y position offset to apply to detected objects
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
        """Initializes the trunk width analyzer and segmenter.
        
        Args:
            width_estimation_config_file_path (str): Path to configuration file for trunk analyzer
        """
        self.mutex.lock()
        
        # TODO: should use the actual config file i'd imagine
        config_file = "width_estimation_config_apple.yaml"
        self.trunk_analyzer = import_trunk_analyzer(config_file)

        self.mutex.unlock()

    @pyqtSlot(data_msgs.Image)
    def handle_request(self, image_data_msg: data_msgs.Image):
        """Queues incoming image data for processing.
        
        Stores the image data and signals the worker thread to start processing.
        
        Args:
            image_data_msg (data_msgs.Image): Image message to process
        """
        self.mutex.lock()
        self.unprocessed_image_data_msg = image_data_msg
        
        self.wait_condition.wakeAll()
        self.mutex.unlock()
        
    def run(self):
        """Main processing loop for the thread.
        
        Waits for image requests, processes them for trunk detection,
        and emits signals with the processed results.
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
        """Processes images to extract trunk detection and width information.

        Uses the trunk_width_estimation package to detect trees and measure trunk widths.
        Enriches the image message with detections and optionally visualization images.

        Args:
            img_data_msg (data_msgs.Image): Input image message containing RGB and depth images

        Returns:
            data_msgs.Image: Enhanced image message with trunk detection data
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
        """Formats and emits detection results as human-readable messages.

        Args:
            positions (np.array): Array of detected trunk positions
            widths (np.array): Array of detected trunk widths in meters
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
        """Controls whether to emit signals for calibration data recording.

        Args:
            emitting_save_calibration_data (bool): Whether to emit calibration data signals
        """
        self.emitting_save_calibration_data = emitting_save_calibration_data

    @pyqtSlot(dict)
    def set_images_to_include(self, images_to_include):
        """Controls which visualization image types to include in results.

        Args:
            images_to_include (dict): Dictionary with boolean flags for image types
                to include (e.g., {'unfiltered': True, 'depth': False})
        """
        if 'unfiltered' not in images_to_include:
            images_to_include['unfiltered'] = False
        if 'depth' not in images_to_include:
            images_to_include['depth'] = False

        self.images_to_include['unfiltered'] = images_to_include['unfiltered']
        self.images_to_include['depth'] = images_to_include['depth']

        
