from PyQt5.QtCore import QThread, pyqtSignal, pyqtSlot, QMutex, QWaitCondition
import numpy as np

from pf_orchard_localization.pf_engine import PfEngine
from pf_orchard_localization.visual_odom import OpticalFlowOdometerThread
from pf_orchard_localization.data_managers import data_msgs
from pf_orchard_localization.utils.parameters import ParametersCachedData, ParametersBagData

from typing import Union, TYPE_CHECKING
import time

if TYPE_CHECKING:
    from pf_orchard_localization import data_managers
    from pf_orchard_localization import img_processing_srv

import logging
logger = logging.getLogger(__name__)

class RecordedData(QThread):
    """Thread for running particle filter with pre-recorded data.
    
    Processes data from ROS bag files or cached data sources to update 
    the particle filter state.
    """
    
    load_next_data_file = pyqtSignal(bool)
    pf_run_message = pyqtSignal(str)
    set_time_line = pyqtSignal(float)
    cache_msg = pyqtSignal(object)
    set_img_number_label = pyqtSignal(int, int)
    plot_best_guess = pyqtSignal(np.ndarray)
    plot_particles = pyqtSignal(np.ndarray)
    corrected_gnss_data_signal = pyqtSignal(data_msgs.Gnss)
    uncorrected_gnss_data_signal = pyqtSignal(data_msgs.Gnss)
    
    
    def __init__(self, 
                 pf_engine: PfEngine, 
                 data_manager: Union['data_managers.Ros2Bag', 'data_managers.Cached'],
                 trunk_data_thread: Union['img_processing_srv.DirectPkgConnection', 'img_processing_srv.Ros2Service'],
                 data_parameters: Union[ParametersBagData, ParametersCachedData],
                 stop_when_converged: bool,
                 only_single_image: bool,
                 time_delay_multiplier: float,
                 cache_data_enabled: bool=False,
                 using_cached_data: bool=False):
        """
        Args:
            pf_engine (PfEngine): The particle filter engine
            data_manager (recorded_data_loaders.Ros2Bag or .Cached): The data manager that loads and manages the data from the bag file
            trunk_data_thread (trunk_data_connection.DirectPkgConnection or .Ros2Service): The thread that handles getting the trunk data
            data_parameters (ParametersBagData or ParametersCachedData): The parameters for the data
            stop_when_converged (bool): If True, the thread will stop when the particle filter converges
            only_single_image (bool): If True, the thread will only process a single image then exit
            time_delay_multiplier (float): The amount of added time to wait between processing images
            cache_data_enabled (bool, optional): If True, the thread will cache the data. Defaults to False.
            using_cached_data (bool, optional): If True, the thread will use cached data. Defaults to False.
        """
        
        super().__init__()
        
        self.pf_engine = pf_engine
        self.data_manager = data_manager
        self.cache_data_enabled = cache_data_enabled
        self.trunk_data_thread = trunk_data_thread
        self.data_parameters = data_parameters
        self.stop_when_converged = stop_when_converged
        self.only_single_image = only_single_image
        self.time_delay_multiplier = time_delay_multiplier
        self.using_cached_data = using_cached_data

        self.trunk_data_thread.signal_request_processed.connect(self.on_trunk_request_processed)
        
        self.processed_img_data_msg: data_msgs.Image = None
        self.processed_odom_data_msg: data_msgs.Odom = None
        self.image_idx = 0
        
        if self.data_parameters.use_visual_odom and not self.using_cached_data:
            self.visual_odom_thread = OpticalFlowOdometerThread()
            self.visual_odom_thread.start()
            self.visual_odom_thread.signal_request_processed.connect(self.on_visual_odom_request_processed)
            self.odom_mutex = QMutex()
            self.odom_condition = QWaitCondition()
        
        self.img_data_request_mutex = QMutex()
        self.img_data_request_condition = QWaitCondition()
        
        self.data_manager_mutex = QMutex()
        self.data_manager_condition = QWaitCondition()
        
        self.pf_active = False
        self.converged = False
        self.load_data_success = False
        
        self.prev_img_msg_timestamp: data_msgs.Timestamp = None
        self.prev_actual_time = time.time()
        
    def get_new_data_manager(self) -> None:
        """Request a new data manager from the main thread.
        
        Signals the main thread to load the next data file and waits 
        for the new data manager to be received.
        """

        load_first_image = False
        self.load_next_data_file.emit(load_first_image)

        self.data_manager_mutex.lock()
        timeout = 120 * 1000  # Timeout in milliseconds # TODO: Maybe make this a parameter
        if not self.data_manager_condition.wait(self.data_manager_mutex, timeout):
            self.pf_run_message.emit("Timeout waiting for data manager. Stopping particle filter.")
            self.pf_active = False
            self.data_manager_mutex.unlock()
            return

        if not self.load_data_success:
            self.pf_run_message.emit("Failed to load next data file. Stopping particle filter.")
            self.pf_active = False
        
        self.data_manager_mutex.unlock()
        
    @pyqtSlot(bool, object)
    def data_manager_receiver(self, success: bool, data_manager) -> None:
        """Receive the data manager from the main thread.
        
        Args:
            success (bool): Whether the data manager was successfully loaded
            data_manager: The new data manager instance
        """

        self.data_manager_mutex.lock()
        self.data_manager = data_manager
        self.load_data_success = success
        self.data_manager_condition.wakeAll()
        self.data_manager_mutex.unlock()        
            
    def run(self) -> None:
        """Main execution loop that processes data until stopped.
        
        Continuously processes messages from the data manager until 
        stopped by calling stop_pf().
        """
        
        self.pf_active = True
        
        while self.pf_active:
            self.send_next_msg()
            
            if self.pf_active and self.data_manager.at_img_msg and self.only_single_image:
                self.pf_active = False
    
    def send_next_msg(self) -> None:
        """Get and process the next message from the data manager.
        
        Retrieves the next message, handles it according to its type, 
        and updates the particle filter state.
        """

        current_msg = self.data_manager.get_next_msg()

        # The data manager returns None if it is at the end of the data
        if current_msg is None:

            if not self.data_parameters.auto_load_next_file:
                self.pf_run_message.emit("Reached end of data. Stopping particle filter.")
                self.pf_active = False
                return

            self.get_new_data_manager()

            if not self.pf_active:
                return
            
            current_msg = self.data_manager.get_next_msg()

        self.set_time_line.emit(self.data_manager.get_time_relative_to_start())

        if current_msg.message_type == data_msgs.MsgType.WHEEL_ODOM and not self.data_parameters.use_visual_odom:
            logger.debug(f"Sending wheel odom message to pf_engine. msg timestamp: {current_msg.msg_timestamp}, bag timestamp: {current_msg.bag_timestamp}")
            self.pf_engine.motion_update(current_msg)
        elif current_msg.message_type == data_msgs.MsgType.VISUAL_ODOM and self.data_parameters.use_visual_odom:
            logger.debug(f"Sending visual odom message to pf_engine. msg timestamp: {current_msg.msg_timestamp}, bag timestamp: {current_msg.bag_timestamp}")
            self.pf_engine.motion_update(current_msg)
        elif current_msg.message_type == data_msgs.MsgType.IMAGE:
            logger.debug(f"Parsing image message. msg timestamp: {current_msg.msg_timestamp}, bag timestamp: {current_msg.bag_timestamp}")
            self.handle_image_msg(current_msg)
            self.handle_delay(current_msg)
        elif current_msg.message_type == data_msgs.MsgType.IMU:
            logger.debug(f"Sending IMU message to pf_engine. msg timestamp: {current_msg.msg_timestamp}, bag timestamp: {current_msg.bag_timestamp}")
            self.pf_engine.orientation_update(current_msg)
        elif current_msg.message_type == data_msgs.MsgType.GNSS_CORRECTED:
            logger.debug(f"Parsing corrected GNSS message. msg timestamp: {current_msg.msg_timestamp}, bag timestamp: {current_msg.bag_timestamp}")
            self.corrected_gnss_data_signal.emit(current_msg)
        elif current_msg.message_type == data_msgs.MsgType.GNSS_UNCORRECTED:
            logger.debug(f"Parsing uncorrected GNSS message. msg timestamp: {current_msg.msg_timestamp}, bag timestamp: {current_msg.bag_timestamp}")
            self.uncorrected_gnss_data_signal.emit(current_msg)

        if self.cache_data_enabled:
            self.cache_msg.emit(current_msg)
            # Need to cache a pose message at some point, so doing it at each image
            if current_msg.message_type == data_msgs.MsgType.IMAGE:
                pose_estimate_msg = data_msgs.PoseEstimate.from_particle_pose(self.pf_engine.best_particle, msg_timestamp=current_msg.msg_timestamp, bag_timestamp=current_msg.bag_timestamp)
                self.cache_msg.emit(pose_estimate_msg)

    def wait_for_response(self) -> None:
        """Wait for trunk data and visual odometry data to be received.
        
        Blocks until both the processed image data and visual odometry 
        data (if enabled) are available.
        """

        self.img_data_request_mutex.lock()
        if self.processed_img_data_msg is None:
            self.img_data_request_condition.wait(self.img_data_request_mutex)
        self.img_data_request_mutex.unlock()
        
        if self.data_parameters.use_visual_odom and not self.using_cached_data:
            # Trunk data is in, now wait for odom data if it is not already in
            self.odom_mutex.lock()
            if self.processed_odom_data_msg is None:
                self.odom_condition.wait(self.odom_mutex)
            self.odom_mutex.unlock()
    
    @pyqtSlot(object)
    def on_trunk_request_processed(self, img_data_msg: data_msgs.Image) -> None:
        """Receive processed trunk data from the trunk data thread.
        
        Args:
            img_data_msg (data_msgs.Image): Image data containing trunk detection information
        """
        self.img_data_request_mutex.lock()
        self.processed_img_data_msg = img_data_msg
        self.img_data_request_condition.wakeAll()
        self.img_data_request_mutex.unlock()
    
    @pyqtSlot(object)
    def on_visual_odom_request_processed(self, odom_msg_data: data_msgs.Odom) -> None:
        """Receive visual odometry data from the optical flow thread.

        Args:
            odom_msg_data (data_msgs.Odom): Odometry data containing estimated motion
        """
        self.odom_mutex.lock()
        self.processed_odom_data_msg = odom_msg_data
        self.odom_condition.wakeAll()
        self.odom_mutex.unlock()

    def handle_image_msg(self, img_data_msg: data_msgs.Image) -> None: 
        """Process image data to get trunk detections and visual odometry.

        Sends the image to the trunk detection thread and visual odometry thread,
        updates the particle filter with the results, and plots the current state.

        Args:
            img_data_msg (data_msgs.Image): Image data message to process
        """
        process_visual_odom = self.data_parameters.use_visual_odom and img_data_msg.source != data_msgs.Source.CACHED

        self.processed_img_data_msg = None
        self.processed_odom_data_msg = None

        self.trunk_data_thread.handle_request(img_data_msg)
        
        if process_visual_odom:
            self.visual_odom_thread.handle_request(img_data_msg)        
        
        self.wait_for_response()
        
        if process_visual_odom:
            # If the odom data is None, the visual odometer failed to process the image
            if self.processed_odom_data_msg.linear_displacement is not None:
                self.pf_engine.motion_update(self.processed_odom_data_msg)
                if self.cache_data_enabled:
                    self.cache_msg.emit(self.processed_odom_data_msg)
        
        self.set_img_number_label.emit(self.data_manager.current_img_position, self.data_manager.num_img_msgs)

        self.pf_engine.sensor_update(self.processed_img_data_msg)

        self.plot_best_guess.emit(self.pf_engine.best_particle)
        self.plot_particles.emit(self.pf_engine.downsample_particles())       
        
        if self.processed_img_data_msg.object_locations is not None:
            self.check_convergence()
    
    def handle_delay(self, img_data_msg: data_msgs.Image) -> None:
        """Insert appropriate delay between image processing.
        
        Calculates and applies delay based on the time difference between 
        consecutive images and the configured time multiplier.
        
        Args:
            img_data_msg (data_msgs.Image): Current image data message
        """
        
        if self.prev_img_msg_timestamp is not None:
            actual_time_taken = time.time() - self.prev_actual_time
            time_stamp_difference = img_data_msg.msg_timestamp - self.prev_img_msg_timestamp
            time_to_wait = time_stamp_difference.to_sec() * self.time_delay_multiplier - actual_time_taken
            if time_to_wait > 0:
                self.msleep(int(time_to_wait * 1000))
        
        self.prev_img_msg_timestamp = img_data_msg.msg_timestamp
        self.prev_actual_time = time.time()
        
           
    def check_convergence(self) -> None:
        """Check if the particle filter has converged.
        
        Updates the converged flag and stops processing if configured
        to stop when convergence is detected.
        """

        self.converged = self.pf_engine.check_convergence()
        
        if self.converged and self.stop_when_converged:
            self.pf_active = False
            
    @pyqtSlot()
    def stop_pf(self) -> None:
        """Stop the particle filter thread."""
        
        self.pf_active = False



