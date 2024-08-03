from ..utils.pf_evaluation import PfTestExecutor
from PyQt5.QtCore import QThread, pyqtSignal, pyqtSlot, QMutex, QWaitCondition
import numpy as np
import time
from ..utils.parameters import ParametersPf
from ..pf_engine import PfEngine
import inspect
from ..visual_odom import OpticalFlowOdometerThread
import os

class PfBagThread(QThread):
    """
    Thread to run the particle filter algorithm using data from a bag file
    """
    
    load_next_data_file = pyqtSignal(bool)
    pf_run_message = pyqtSignal(str)
    set_time_line = pyqtSignal(float)
    cache_msg = pyqtSignal(dict)
    set_img_number_label = pyqtSignal(int, int)
    plot_best_guess = pyqtSignal(np.ndarray)
    plot_particles = pyqtSignal(np.ndarray)
    
    
    def __init__(self, 
                 pf_engine: PfEngine, 
                 data_manager, 
                 trunk_data_thread, 
                 stop_when_converged, 
                 only_single_image, 
                 added_delay, 
                 image_fps,
                 use_visual_odom=False,
                 cache_data_enabled=False):
        """
        Args:
            pf_engine (PfEngine): The particle filter engine
            data_manager (Bag2DataLoader): The data manager that loads and manages the data from the bag file
            trunk_data_thread (TrunkDataConnection or TrunkDataConnectionRosService): The thread that handles getting the trunk data
            stop_when_converged (bool): If True, the thread will stop when the particle filter converges
            only_single_image (bool): If True, the thread will only process a single image then exit
            added_delay (float): The amount of added time to wait between processing images
            image_fps (int): The frames per second of the images
            use_visual_odom (bool, optional): If True, the thread will use visual odometry. Defaults to False.
            cache_data_enabled (bool, optional): If True, the thread will cache the data. Defaults to False.
        """
        
        super().__init__()
        
        self.pf_engine = pf_engine
        self.data_manager = data_manager
        self.cache_data_enabled = cache_data_enabled
        self.trunk_data_thread = trunk_data_thread
        self.stop_when_converged = stop_when_converged
        self.only_single_image = only_single_image
        self.added_delay = added_delay
        self.use_visual_odom = use_visual_odom

        self.fps = image_fps
        
        self.dt = 1 / self.fps
        
        self.trunk_data_thread.signal_request_processed.connect(self.on_trunk_request_processed)
        
        self.trunk_data = None
        
        if self.use_visual_odom:
            self.visual_odom_thread = OpticalFlowOdometerThread()
            self.visual_odom_thread.start()
            self.visual_odom_thread.signal_request_processed.connect(self.on_visual_odom_request_processed)
            self.odom_mutex = QMutex()
            self.odom_condition = QWaitCondition()
        
        self.trunk_mutex = QMutex()
        self.trunk_condition = QWaitCondition()
        
        self.data_manager_mutex = QMutex()
        self.data_manager_condition = QWaitCondition()
        
        self.pf_active = False
        self.converged = False
        self.load_data_success = False
        
    def get_new_data_manager(self):
        """
        Get a new data manager from the main thread. Tells the main thread to load the next data file then waits for the data manager to be received.
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
    def data_manager_receiver(self, success, data_manager):
        """
        Slot to receive the data manager from the main thread
        """
        self.data_manager_mutex.lock()
        self.data_manager = data_manager
        self.load_data_success = success
        self.data_manager_condition.wakeAll()
        self.data_manager_mutex.unlock()
        
            
    def run(self):
        """
        The main loop of the thread, stopped by calling stop_pf(). 
        """
        
        self.pf_active = True
        
        while self.pf_active:
            self.send_next_msg()
            self.msleep(int(self.added_delay * 1000))
            
            if self.pf_active and self.data_manager.at_img_msg and self.only_single_image:
                self.pf_active = False
    
    def send_next_msg(self):
        """
        Get the next message from the data manager and process it
        """
        current_msg = self.data_manager.get_next_msg()
        
        # The data manager returns None if it is at the end of the data
        if current_msg is None:
            self.get_new_data_manager()
            if not self.pf_active:
                return
            current_msg = self.data_manager.get_next_msg()
        
        self.set_time_line.emit(self.data_manager.current_data_file_time_stamp)

        # Using visual odom so this is commented out, TODO: make visual odom a setting somewhere, also, cacheing for visual odom currently doesn't work
        if current_msg['topic'] == 'odom' and not self.use_visual_odom:
            x_odom, theta_odom, time_stamp_odom = self.get_wheel_odom_data(current_msg)
            self.pf_engine.handle_odom(x_odom, theta_odom, time_stamp_odom)
            
        elif current_msg['topic'] == 'image':
            
            self.get_data_from_image_msg(current_msg)

    def get_data_from_image_msg(self, current_msg): 
        """
        Get the trunk data and odom data from the image message

        Args:
            current_msg (dict): The current message from the data manager
        """
        
        # Send off the request for processing the trunk data and odom data then wait for the response
        request = {"current_msg": current_msg}

        self.trunk_data_thread.handle_request(request)
        
        if self.use_visual_odom:
            self.visual_odom_thread.handle_request(request)
        
        self.trunk_data = None
        self.x_odom = None
        
        self.wait_for_response()
        
        if self.use_visual_odom:
            # If the odom data is None, the visual odometer failed to process the image
            if self.x_odom != "returned_none":
                u = np.array([[self.x_odom/self.dt], [0]])
                num_readings = int(self.dt * 60)
                self.pf_engine.motion_update(u, self.dt, num_readings=num_readings)
            else:
                return
        

        positions, widths, class_estimates, seg_img = self.trunk_data

       # Exit if there is no trunk data
        if positions is None:
            if self.cache_data_enabled:
                data = {'topic': 'image', 'positions': None, 'widths': None, 'class_estimates': None, 'location_estimate': None,
                        'time_stamp': current_msg['timestamp'], 'image': None}
                self.cache_msg.emit(data)
            return

        self.set_img_number_label.emit(self.data_manager.current_img_position, self.data_manager.num_img_msgs)

        tree_data = {'positions': positions, 'widths': widths, 'classes': class_estimates}
        self.pf_engine.scan_update(tree_data)

        self.plot_best_guess.emit(self.pf_engine.best_particle)
        self.plot_particles.emit(self.pf_engine.downsample_particles())

        if self.cache_data_enabled:
            data = {'topic': 'image', 'positions': positions, 'widths': widths, 'class_estimates': class_estimates, 'location_estimate': self.pf_engine.best_particle,
                    'time_stamp': current_msg['timestamp'], 'image': seg_img}
            self.cache_msg.emit(data)
                    
        self.check_convergence()
        
    def wait_for_response(self):
        """
        Wait for trunk data and odom data to be received
        """ 
        self.trunk_mutex.lock()
        if self.trunk_data is None:
            self.trunk_condition.wait(self.trunk_mutex)
        self.trunk_mutex.unlock()
        
        if self.use_visual_odom:
            # Trunk data is in, now wait for odom data if it is not already in
            self.odom_mutex.lock()
            if self.x_odom is None:
                self.odom_condition.wait(self.odom_mutex)
            self.odom_mutex.unlock()
        
    
    @pyqtSlot(object)
    def on_trunk_request_processed(self, trunk_data):
        """
        Slot to receive the trunk data from the trunk data thread
        
        Args:
            trunk_data (tuple): The trunk data
        """
        self.trunk_mutex.lock()
        self.trunk_data = trunk_data
        self.trunk_condition.wakeAll()
        self.trunk_mutex.unlock()
    
    @pyqtSlot(object)
    def on_visual_odom_request_processed(self, x_odom):
        """
        Slot to receive the odom data from the visual odometer thread

        Args:
            x_odom (float): The estimated x movement in mm from the visual odometer
        """
        self.odom_mutex.lock()
        if x_odom is None:
            self.x_odom = "returned_none"
        else:
            self.x_odom = x_odom
        self.odom_condition.wakeAll()
        self.odom_mutex.unlock()
        
    def get_wheel_odom_data(self, current_msg):
        """
        Get the odometry data from the current message

        Args:
            current_msg (dict): The current message
        
        Returns:
            tuple: The x movement, theta movement, and time stamp of the odometry data
        """
        odom_data = current_msg['data']
        x_odom = odom_data.twist.twist.linear.x
        theta_odom = odom_data.twist.twist.angular.z
        time_stamp_odom = odom_data.header.stamp.sec + odom_data.header.stamp.nanosec * 1e-9

        if self.cache_data_enabled:
            data = {'topic': 'odom', 'x_odom': x_odom, 'theta_odom': theta_odom, 'time_stamp': time_stamp_odom}
            self.cache_msg.emit(data)

        return x_odom, theta_odom, time_stamp_odom
    
    def check_convergence(self):
        """
        Check if the particle filter has converged, and set the converged flag
        """
        self.converged = self.pf_engine.check_convergence()
        
        if self.converged and self.stop_when_converged:
            self.pf_active = False
            
    @pyqtSlot()
    def stop_pf(self):
        """
        Stop the particle filter
        """
        self.pf_active = False
    
class PfCachedThread(PfBagThread):
    """
    Thread to run the particle filter algorithm using cached data, extending PfBagThread
    """
    def get_new_data_manager(self):
        """
        Override the base class method to just exit if the end of a data file is reached, as cached data does not have multiple files
        """
        self.pf_active = False       
    
    def get_wheel_odom_data(self, current_msg):
        """
        Override the base class method to get the odometry data from the cached data manager

        Args:
            current_msg (dict): The current message

        Returns:
            tuple: The x movement, theta movement, and time stamp of the odometry data
        """
        odom_data = current_msg['data']
        x_odom = odom_data['x_odom']
        theta_odom = odom_data['theta_odom']
        time_stamp_odom = current_msg['timestamp']

        return x_odom, theta_odom, time_stamp_odom 
    
    def get_data_from_image_msg(self, current_msg):
        """
        Override the base class method to get the trunk data and odom data from the cached data manager

        Args:
            current_msg (dict): The current message from the data manager
        """
        
        request = {"current_msg": current_msg}
        
        self.trunk_data_thread.handle_request(request)

        if self.use_visual_odom:
            self.visual_odom_thread.handle_request(request)
        
        self.trunk_data = None
        
        self.wait_for_response()

        positions, widths, class_estimates = self.trunk_data

        if positions is None:
            return

        tree_data = {'positions': positions, 'widths': widths, 'classes': class_estimates}
        self.pf_engine.scan_update(tree_data)

        actual_position = (current_msg['data']['location_estimate'])
        self.position_gt = np.array([actual_position['x'], actual_position['y']])
        self.plot_best_guess.emit(self.position_gt)

        self.position_estimate = self.pf_engine.best_particle
        self.plot_particles.emit(self.pf_engine.downsample_particles())

        self.set_img_number_label.emit(self.data_manager.current_img_position, self.data_manager.num_img_msgs)

        self.check_convergence()


class PfTestExecutorQt(PfTestExecutor, QThread):
    """
    A class for running the particle filter tests in the qt app on a separate thread
    """
    update_test_number = pyqtSignal(int)
    set_time_line = pyqtSignal(float)
    update_trial_number = pyqtSignal(int)
    plot_gt_position = pyqtSignal(np.ndarray)
    plot_particles = pyqtSignal(np.ndarray)
    update_image_number = pyqtSignal(int, int)
    update_trial_info = pyqtSignal(float, int)
    update_ui_with_trial_results = pyqtSignal(float, float, float)
    reset_pf_app = pyqtSignal(bool)
    print_message = pyqtSignal(str)
    
    
    def __init__(self,
                 pf_engine: PfEngine,
                 parameters_pf: ParametersPf,
                 test_info_path: str,
                 cached_data_files_dir: str,
                 num_trials,
                 get_trunk_data_func,
                 save_path=None,
                 convergence_threshold=0.5,
                 test_index=None,
                 load_data_only=False
                 ):
        """
        Args:
            pf_engine (PfEngine): The particle filter engine
            parameters_pf (ParametersPf): The parameters for the particle filter
            test_info_path (str): The path to the csv file containing the test information
            cached_data_files_dir (str): The directory containing the cached data files
            num_trials (int): The number of trials to run for each test
            get_trunk_data_func (function): A function that returns the trunk data from an image message
            save_path (str, optional): The path to save the results. Defaults to None.
            convergence_threshold (float, optional): The distance error threshold to be considered a correct convergence. Defaults to 0.5.
            test_index (int, optional): The index of the test to run. If None, all tests will be run. Defaults to None.
            load_data_only (bool, optional): If True, the data will be loaded but the tests will not be run. Defaults to False.
        """
        
        # Initialize the PfTestExecutor class, which contains the main logic for running the tests
        PfTestExecutor.__init__(self,
                                pf_engine=pf_engine, 
                                parameters_pf=parameters_pf,
                                test_info_path=test_info_path,
                                cached_data_files_dir=cached_data_files_dir,
                                num_trials=num_trials,
                                save_path=save_path,
                                print_message_func=self.signal_print_message, 
                                convergence_threshold=convergence_threshold)
        
        self.get_trunk_data_func = get_trunk_data_func
        self.test_index = test_index
        self.load_data_only = load_data_only
        
        QThread.__init__(self)
    
    def signal_print_message(self, message):
        """
        Override the print_message function to emit a signal to the app
        """
        self.print_message.emit(message)
    
    def run(self):
        """
        The main loop of the thread, stopped by calling stop_pf().
        """

        
        if self.test_index is not None:
            if self.load_data_only:
                self.reset_for_test(self.test_regimen.pf_tests[self.test_index])
            else:
                self.run_selected_test(self.test_index)
        elif self.test_index is None:
            self.run_all_tests()
            
    # def signal_running_all_tests(self):
    #     self.running_all_tests.emit()
    
    # def signal_done_running_all_tests(self):
    #     self.done_running_all_tests.emit()
    
    # def signal_running_selected_test(self):
    #     self.running_selected_test.emit()
    
    # def signal_done_running_selected_test(self):
    #     self.done_running_selected_test.emit()
    
    def signal_update_test_number(self, test_name):
        """
        Send signal to update the test number in the app
        
        Args:
            test_name (int): The test number"""
        self.update_test_number.emit(test_name)
    
    def signal_set_time_line(self, current_time):
        """
        Send signal to set the time line in the app

        Args:
            current_time (float): The current time
        """
        self.set_time_line.emit(current_time)
    
    def signal_current_msg(self, current_msg):
        """
        Send signal to get the trunk data from the current message

        Args:
            current_msg (dict): The current message data
        """
        self.get_trunk_data(current_msg)
    
    def signal_update_trial_number(self, trial_num):
        """
        Send signal to update the trial number in the app

        Args:
            trial_num (int): The trial number
        """
        self.update_trial_number.emit(trial_num)
        
    def signal_plot_gt_position(self):
        """
        Send signal to plot the ground truth position in the app
        """
        self.plot_gt_position.emit(self.position_gt)
    
    def signal_plot_particles(self):
        """
        Send signal to plot the particles in the app
        """
        particles = self.pf_engine.downsample_particles()
        self.plot_particles.emit(particles)
        
    def signal_update_image_number(self):
        """
        Send signal to update the image number in the app
        """
        current_image_position = self.data_manager.current_img_position
        num_img_msgs = self.data_manager.num_img_msgs
        self.update_image_number.emit(current_image_position, num_img_msgs)
    
    def signal_update_trial_info(self):
        """
        Send signal to update the trial info in the app
        """
        particle_count = self.pf_engine.particles.shape[0]
        current_time = time.time() - self.trial_start_time
        self.update_trial_info.emit(current_time, particle_count)
    
    def signal_update_ui_with_trial_results(self, test_info):
        """
        Send signal to update the ui with the trial results

        Args:
            test_info (PfTest): The test information
        """
        trial_convergence_rate, trial_avg_time_all, trial_avg_time_converged = test_info.get_results()
        # self.main_app_manager.pf_test_controls.update_convergence_rate(trial_convergence_rate, 0)
        # self.main_app_manager.pf_test_controls.update_test_time(trial_avg_time_all, 0)
        self.update_ui_with_trial_results.emit(trial_convergence_rate, trial_avg_time_all, trial_avg_time_converged)

    def get_trunk_data(self, current_msg):
        """
        Get the trunk data from the current message using the get_trunk_data_func
        """
        return self.get_trunk_data_func(current_msg)
    
    def reset_pf(self):
        """
        Reset the particle filter
        """
        self.reset_pf_app.emit(False)

        # wait for the reset to complete, otherwise things break
        # TODO, probably would be best to have a response signal setup instead of waiting
        time.sleep(0.4)
        
        # self.pf_engine.reset_pf(self.parameters_pf)
    
    @pyqtSlot()
    def stop_pf(self):
        """
        Stop the particle filter
        """
        self.pf_active = False
        self.tests_aborted = True