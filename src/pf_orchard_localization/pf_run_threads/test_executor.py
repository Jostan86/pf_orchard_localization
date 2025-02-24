from PyQt5.QtCore import QThread, pyqtSignal, pyqtSlot, QMutex, QWaitCondition

import numpy as np
import time
import os
from functools import wraps
from typing import TYPE_CHECKING

from pf_orchard_localization.pf_engine import PfEngine
from pf_orchard_localization.utils.pf_evaluation import PfTestRegimen, PfTest
from pf_orchard_localization.utils.parameters import ParametersPf, ParametersCachedData
from pf_orchard_localization.data_managers import data_msgs
from pf_orchard_localization import data_managers
from pf_orchard_localization import img_processing_srv

import logging
logger = logging.getLogger(__name__)

def check_tests_aborted(func):
    @wraps(func)
    def wrapper(self, *args, **kwargs):
        if self.tests_aborted:
            return
        return func(self, *args, **kwargs)
    return wrapper

class TestExecutorQt(QThread):
    """
    A class for running the particle filter tests in the qt app on a separate thread
    """
    pf_run_message = pyqtSignal(str)
    set_img_number_label = pyqtSignal(int, int)
    plot_best_guess = pyqtSignal(np.ndarray)
    plot_particles = pyqtSignal(np.ndarray)
    corrected_gnss_data_signal = pyqtSignal(data_msgs.Gnss)
    uncorrected_gnss_data_signal = pyqtSignal(data_msgs.Gnss)
    set_time_line = pyqtSignal(float)
    
    update_test_number = pyqtSignal(int)
    update_trial_number = pyqtSignal(int)
    plot_gt_position = pyqtSignal(np.ndarray)
    update_trial_info = pyqtSignal(float, int)
    update_ui_with_trial_results = pyqtSignal(float, float, float)
    # reset_pf_app = pyqtSignal(bool)
       
    
    def __init__(self,
                 pf_engine: PfEngine,
                 parameters_pf: ParametersPf,
                 parameters_data: ParametersCachedData,
                 trunk_data_thread: img_processing_srv.DirectPkgConnection,
                 num_trials,
                 use_visual_odom: bool = False,
                 convergence_threshold=0.5,
                 test_index=None,
                 load_data_only=False,
                 ):
        """
        Args:
            pf_engine (PfEngine): The particle filter engine
            parameters_pf (ParametersPf): The parameters for the particle filter
            parameters_data (ParametersCachedData): The parameters for the cached data
            trunk_data_thread (img_processing_srv.DirectPkgConnection): The thread for getting the trunk data
            num_trials (int): The number of trials to run for each test
            use_visual_odom (bool, optional): Whether to use visual odometry. Defaults to False.
            save_path (str, optional): The path to save the results. Defaults to None.
            convergence_threshold (float, optional): The distance error threshold to be considered a correct convergence. Defaults to 0.5.
            test_index (int, optional): The index of the test to run. If None, all tests will be run. Defaults to None.
            load_data_only (bool, optional): If True, the data will be loaded but the tests will not be run. Defaults to False.
        """
        
        super().__init__()

        self.pf_engine = pf_engine
        self.parameters_pf = parameters_pf
        self.parameters_data = parameters_data
        self.trunk_data_thread = trunk_data_thread
        self.num_trials = num_trials
        
        self.convergence_threshold = convergence_threshold
        self.test_index = test_index
        self.load_data_only = load_data_only
        self.use_visual_odom = use_visual_odom

        self.save_path = parameters_data.test_results_save_path
        if self.save_path is None:
            logger.warning("No save path for the results is given in config file. Results will not be saved.")


        self.trunk_data_thread.signal_request_processed.connect(self.on_trunk_request_processed)


        self.test_regimen = PfTestRegimen(self.parameters_data.test_start_info_path, self.print_message)
        if not self.load_data_only:
            self.test_regimen.initialize_save_files(self.save_path)

        self.pf_active = False
        self.tests_aborted = False
        self.data_manager: data_managers.Cached = None
        self.position_estimate: np.ndarray = None
        self.position_gt: np.ndarray = None
        self.start_position: np.ndarray = None

        self.img_data_request_mutex = QMutex()
        self.img_data_request_condition = QWaitCondition()

        self.processed_img_data_msg: data_msgs.Image = None

        self.msg_handlers = {
            data_msgs.MsgType.WHEEL_ODOM: self.handle_wheel_odom,
            data_msgs.MsgType.VISUAL_ODOM: self.handle_visual_odom,
            data_msgs.MsgType.IMAGE: self.handle_image_msg,
            data_msgs.MsgType.IMU: self.handle_imu,
            data_msgs.MsgType.GNSS_CORRECTED: self.handle_gnss_corrected,
            data_msgs.MsgType.GNSS_UNCORRECTED: self.handle_gnss_uncorrected,
            data_msgs.MsgType.POSE_ESTIMATE: self.handle_pose_estimate,
        }
    
    def print_message(self, message):
        """Override the print_message function to emit a signal to the app"""
        self.pf_run_message.emit(message)
    
    def run(self):
        """The main loop of the thread, stopped by calling stop_pf()."""
        
        if self.test_index is not None:
            if self.load_data_only:
                self.reset_for_test(self.test_regimen.pf_tests[self.test_index])
            else:
                self.run_selected_test(self.test_index)
        elif self.test_index is None:
            self.run_all_tests()

    
    def stop_pf(self):
        """Stop the particle filter gracefully"""
        self.pf_active = False
        self.tests_aborted = True
    
    def run_all_tests(self):
        """Run all tests in the test regimen"""
        self.tests_aborted = False

        self.print_message("Running all tests")

        for test_info in self.test_regimen.pf_tests:
            
            self.run_test(test_info)
            
            if not self.tests_aborted:
                test_info.test_completed = True
            
        if self.save_path is not None and not self.tests_aborted:
            self.test_regimen.process_results()

    def run_selected_test(self, test_index):
        """Run a single test from the test regimen
        
        Args:
            test_index (int): The index of the test to run
        """
        self.tests_aborted = False
        
        test_info = self.test_regimen.pf_tests[test_index]

        self.run_test(test_info)

    def reset_for_trial(self, test_info: PfTest):
        """Reset the particle filter for a new trial of the test

        Args:
            test_info (PfTest): The test info for the test to reset for
        """
        self.start_position = None
        self.position_gt = None
        
        self.data_manager.set_file_time_relative_to_start(test_info.start_time)

        img_data_msg = self.data_manager.get_next_img_msg()

        self.trunk_data_thread.handle_request(img_data_msg)  

        self.wait_for_response()

        self.signal_update_image_number()
        
        self.reset_pf()

    @check_tests_aborted
    def run_test(self, test_info: PfTest):
        """Run a test
        
        Args:
            test_info (PfTest): The test info for the test to run
        """
        
        self.signal_update_test_number(test_info.test_name)

        self.print_message("Running test: " + test_info.test_name)

        self.reset_for_test(test_info)

        for trial_num in range(self.num_trials):
            self.print_message("Starting trial " + str(trial_num + 1))
            self.signal_update_trial_number(trial_num + 1)
            self.run_trial(test_info)
            
            self.signal_update_ui_with_trial_results(test_info)

    def reset_for_test(self, test_info: PfTest):
        """
        Reset the particle filter for a new test

        Args:
            test_info (PfTest): The test info for the test to reset for
        """
        data_file_path = os.path.join(self.parameters_data.data_file_dir, test_info.data_file_name + ".json")
        self.data_manager = data_managers.Cached(data_file_path, self.parameters_data)


        self.parameters_pf.start_pose_center_x = test_info.start_x
        self.parameters_pf.start_pose_center_y = test_info.start_y
        self.parameters_pf.start_width = test_info.start_width
        self.parameters_pf.start_height = test_info.start_length
        self.parameters_pf.start_rotation = test_info.start_rotation
        self.parameters_pf.start_orientation_center = test_info.orientation_center
        self.parameters_pf.start_orientation_range = test_info.orientation_range

        self.reset_for_trial(test_info)

    def reset_pf(self):
        """
        Reset the particle filter
        """
        self.pf_engine.reset_pf(self.parameters_pf)
    
    @check_tests_aborted
    def run_trial(self, test_info: PfTest):
        """
        Run a single trial of the test

        Args:
            test_info (PfTest): The test info for the test to run
        """

        self.reset_for_trial(test_info)

        self.trial_start_time = time.time()
        
        self.pf_active = True

        while self.pf_active:
            self.send_next_msg()

        trial_time = time.time() - self.trial_start_time
        
        correct_convergence, location_error, distance_traveled = self.check_converged_location()

        test_info.add_results(trial_time, correct_convergence, location_error, distance_traveled)

    @check_tests_aborted
    def send_next_msg(self):
        """Send the next message to the particle filter"""

        current_msg = self.data_manager.get_next_msg()

        if current_msg is None:
            self.pf_active = False
            return

        self.signal_set_time_line()

        handler = self.msg_handlers.get(current_msg.message_type)
        if handler:
            handler(current_msg)
        else:
            logger.warning(f"Unhandled message type: {current_msg.message_type}")

        self.signal_update_trial_info()
    
    def handle_wheel_odom(self, msg: data_msgs.Odom):
        if not self.use_visual_odom:
            logger.debug(f"Sending wheel odom message to pf_engine. msg timestamp: {msg.msg_timestamp}, bag timestamp: {msg.bag_timestamp}")
            self.pf_engine.motion_update(msg)

    def handle_visual_odom(self, msg: data_msgs.Odom):
        if self.use_visual_odom:
            logger.debug(f"Sending visual odom message to pf_engine. msg timestamp: {msg.msg_timestamp}, bag timestamp: {msg.bag_timestamp}")
            self.pf_engine.motion_update(msg)

    def handle_imu(self, msg: data_msgs.Imu):
        logger.debug(f"Sending IMU message to pf_engine. msg timestamp: {msg.msg_timestamp}, bag timestamp: {msg.bag_timestamp}")
        self.pf_engine.orientation_update(msg)

    def handle_gnss_corrected(self, msg: data_msgs.Gnss):
        logger.debug(f"Parsing corrected GNSS message. msg timestamp: {msg.msg_timestamp}, bag timestamp: {msg.bag_timestamp}")
        self.corrected_gnss_data_signal.emit(msg)

    def handle_gnss_uncorrected(self, msg: data_msgs.Gnss):
        logger.debug(f"Parsing uncorrected GNSS message. msg timestamp: {msg.msg_timestamp}, bag timestamp: {msg.bag_timestamp}")
        self.uncorrected_gnss_data_signal.emit(msg)

    def handle_pose_estimate(self, msg: data_msgs.PoseEstimate):
        logger.debug(f"Parsing pose estimate message. msg timestamp: {msg.msg_timestamp}, bag timestamp: {msg.bag_timestamp}")
        self.position_gt = msg.to_numpy()
        if self.start_position is None:
            self.start_position = self.position_gt
        self.signal_plot_gt_position()

    def handle_image_msg(self, img_data_msg: data_msgs.Image): 
        """Get the trunk data and odom data from the image message

        Args:
            img_data_msg (data_msgs.Image): The image data message
        """

        self.processed_img_data_msg = None

        self.trunk_data_thread.handle_request(img_data_msg)       
        
        self.wait_for_response()

        self.signal_update_image_number()
        
        self.pf_engine.sensor_update(self.processed_img_data_msg)

        self.signal_plot_particles()
        self.signal_plot_best_guess()

        self.plot_best_guess.emit(self.pf_engine.best_particle)
        self.plot_particles.emit(self.pf_engine.downsample_particles())       
        
        if self.processed_img_data_msg.object_locations is not None:
            self.check_convergence()

    def wait_for_response(self):
        """Wait for trunk data and odom data to be received""" 

        self.img_data_request_mutex.lock()
        if self.processed_img_data_msg is None:
            self.img_data_request_condition.wait(self.img_data_request_mutex)
        self.img_data_request_mutex.unlock()
    
    @pyqtSlot(object)
    def on_trunk_request_processed(self, img_data_msg: data_msgs.Image):
        """Slot to receive the trunk data from the trunk data thread
        
        Args:
            img_msg_data (data_msgs.Image): The image data message
        """
        self.img_data_request_mutex.lock()
        self.processed_img_data_msg = img_data_msg
        self.img_data_request_condition.wakeAll()
        self.img_data_request_mutex.unlock()
    
    def check_convergence(self):
        """Check if the particle filter has converged, and set the converged flag"""

        self.converged = self.pf_engine.check_convergence()
        
        if self.converged:
            self.pf_active = False
   
    # TODO redo this to use GPS
    def check_converged_location(self):
        """
        Check if the particle filter has converged to the correct location

        Returns:
            correct_convergence (bool): Whether the particle filter converged to the correct location
            location_error (float): The error in the estimated location
            distance_treaveled (float): The distance the particle filter traveled before converging
        """
        position_estimate = self.pf_engine.best_particle[0:2]

        actual_position = self.position_gt[0:2]

        location_error = np.linalg.norm(position_estimate - actual_position)

        distance_traveled = np.linalg.norm(self.start_position[0:2] - actual_position)

        if location_error < self.convergence_threshold:
            return True, location_error, distance_traveled
        else:
            return False, location_error, distance_traveled
        
            
    # def signal_running_all_tests(self):
    #     self.running_all_tests.emit()
    
    # def signal_done_running_all_tests(self):
    #     self.done_running_all_tests.emit()
    
    # def signal_running_selected_test(self):
    #     self.running_selected_test.emit()
    
    # def signal_done_running_selected_test(self):
    #     self.done_running_selected_test.emit()
    
    def signal_update_test_number(self, test_name):
        """Send signal to update the test number in the app
        
        Args:
            test_name (int): The test number"""
        self.update_test_number.emit(test_name)
    
    def signal_set_time_line(self):
        """Send signal to set the time line in the app

        Args:
            current_time (float): The current time
        """
        self.set_time_line.emit(self.data_manager.get_time_relative_to_start())
       
    def signal_update_trial_number(self, trial_num):
        """Send signal to update the trial number in the app

        Args:
            trial_num (int): The trial number
        """
        self.update_trial_number.emit(trial_num)
    
    def signal_plot_gt_position(self):
        """Send signal to plot the ground truth position in the app"""
        if self.position_gt is None:
            self.print_message("No ground truth position data available")
        else:
            self.plot_gt_position.emit(self.position_gt)
    
    def signal_plot_best_guess(self):
        """Send signal to plot the best guess in the app"""
        self.plot_best_guess.emit(self.pf_engine.best_particle)
    
    def signal_plot_particles(self):
        """Send signal to plot the particles in the app"""
        particles = self.pf_engine.downsample_particles()
        self.plot_particles.emit(particles)

    def signal_update_image_number(self):
        """Send signal to update the image number in the app"""
        current_image_position = self.data_manager.current_img_position
        num_img_msgs = self.data_manager.num_img_msgs
        self.set_img_number_label.emit(current_image_position, num_img_msgs)
    
    def signal_update_trial_info(self):
        """Send signal to update the trial info in the app"""
        particle_count = self.pf_engine.particles.shape[0]
        current_time = time.time() - self.trial_start_time
        self.update_trial_info.emit(current_time, particle_count)
    
    @check_tests_aborted
    def signal_update_ui_with_trial_results(self, test_info: PfTest):
        """Send signal to update the ui with the trial results

        Args:
            test_info (PfTest): The test information
        """
        pass
        # trial_convergence_rate, trial_avg_time_all, trial_avg_time_converged, _, _ = test_info.get_results()
        # self.main_app_manager.pf_test_controls.update_convergence_rate(trial_convergence_rate, 0)
        # self.main_app_manager.pf_test_controls.update_test_time(trial_avg_time_all, 0)
        # self.update_ui_with_trial_results.emit(trial_convergence_rate, trial_avg_time_all, trial_avg_time_converged)


    
