from ..utils.pf_evaluation import PfTestExecutor
from PyQt6.QtCore import QThread, pyqtSignal, pyqtSlot, QMutex, QWaitCondition
import numpy as np
import time
from ..utils.parameters import ParametersPf
from ..pf_engine import PFEngine
import inspect
from ..visual_odom import OpticalFlowOdometerThread
class PfBagThread(QThread):
    
    load_next_data_file = pyqtSignal(bool)
    pf_run_message = pyqtSignal(str)
    set_time_line = pyqtSignal(float)
    cache_msg = pyqtSignal(dict)
    set_img_number_label = pyqtSignal(int, int)
    plot_best_guess = pyqtSignal(np.ndarray)
    plot_particles = pyqtSignal(np.ndarray)
    
    
    def __init__(self, 
                 pf_engine: PFEngine, 
                 data_manager, 
                 trunk_data_thread, 
                 stop_when_converged, 
                 only_single_image, 
                 added_delay, 
                 fps=10,
                 cache_data_enabled=False):
        
        super().__init__()
        
        self.pf_engine = pf_engine
        self.data_manager = data_manager
        self.cache_data_enabled = cache_data_enabled
        self.trunk_data_thread = trunk_data_thread
        self.stop_when_converged = stop_when_converged
        self.only_single_image = only_single_image
        self.added_delay = added_delay
        self.fps = fps
        
        self.dt = 1 / self.fps
        
        self.trunk_data_thread.signal_request_processed.connect(self.on_trunk_request_processed)
        
        self.trunk_data = None
        
        self.visual_odom_thread = OpticalFlowOdometerThread()
        self.visual_odom_thread.start()
        self.visual_odom_thread.signal_request_processed.connect(self.on_visual_odom_request_processed)
        
        self.trunk_mutex = QMutex()
        self.trunk_condition = QWaitCondition()
        
        self.odom_mutex = QMutex()
        self.odom_condition = QWaitCondition()
        
        self.is_processing = False
        self.pf_active = False
        self.converged = False
        self.load_data_success = False
        self.recieved_data_manager = False
        
        
        
    def get_new_data_manager(self):
        self.recieved_data_manager = False
        
        load_first_image = False
        self.load_next_data_file.emit(load_first_image)
        
        self.load_data_success = False
        
        time_start = time.time()
        while not self.recieved_data_manager:
            
            time.sleep(0.2)
            
            if time.time() - time_start > 120:
                self.pf_run_message.emit("Timeout waiting for data manager. Stopping particle filter. To increase timeout, \
                                         edit line " + str(inspect.currentframe().f_back.f_lineno - 1) + " in pf_mode.py")
                self.pf_active = False
                return 
            
            
        if not self.load_data_success:
            self.pf_run_message.emit("Failed to load next data file. Stopping particle filter.")
            self.pf_active = False
            return 
        
    
    @pyqtSlot(bool, object)
    def data_manager_receiver(self, success, data_manager):
        self.load_data_success = success
        self.data_manager = data_manager
        self.recieved_data_manager = True
        
            
    def run(self):
        
        self.pf_active = True
        
        while self.pf_active:
            self.is_processing = True 
            self.send_next_msg()
            self.is_processing = False
            self.msleep(int(self.added_delay * 1000))
            
            
            if self.pf_active:
                if self.data_manager.at_img_msg and self.only_single_image:
                    self.pf_active = False
    
    def send_next_msg(self):

        current_msg = self.data_manager.get_next_msg()
        
        if current_msg is None:
            self.get_new_data_manager()
            if not self.pf_active:
                return
            current_msg = self.data_manager.get_next_msg()
            

        self.set_time_line.emit(self.data_manager.current_data_file_time_stamp)

        if current_msg['topic'] == 'odom':
            # x_odom, theta_odom, time_stamp_odom = self.get_odom_data(current_msg)
            # self.pf_engine.handle_odom(x_odom, theta_odom, time_stamp_odom)
            pass

        elif current_msg['topic'] == 'image':
            
            start_time = time.time()
            self.get_data_from_image_msg(current_msg)

        self.is_processing = False

    def get_data_from_image_msg(self, current_msg):        
        
        request = {"current_msg": current_msg}
        self.trunk_data_thread.handle_request(request)
        self.visual_odom_thread.handle_request(request)
        
        self.trunk_data = None
        self.x_odom = None
        
        self.wait_for_response()
        
        if self.x_odom != "returned_none":
            u = np.array([[self.x_odom/self.dt], [0]])
            num_readings = int(self.dt * 60)
            self.pf_engine.motion_update(u, self.dt, num_readings=num_readings)
        else:
            return
        
        positions, widths, class_estimates, seg_img = self.trunk_data

       
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
        self.trunk_mutex.lock()
        if self.trunk_data is None:
            self.trunk_condition.wait(self.trunk_mutex)
        self.trunk_mutex.unlock()
        
        self.odom_mutex.lock()
        if self.x_odom is None:
            self.odom_condition.wait(self.odom_mutex)
        self.odom_mutex.unlock()
        
    
    @pyqtSlot(object)
    def on_trunk_request_processed(self, trunk_data):
        self.trunk_mutex.lock()
        self.trunk_data = trunk_data
        self.trunk_condition.wakeAll()
        self.trunk_mutex.unlock()
    
    @pyqtSlot(object)
    def on_visual_odom_request_processed(self, x_odom):
        self.odom_mutex.lock()
        if x_odom is None:
            self.x_odom = "returned_none"
        else:
            self.x_odom = x_odom
        self.odom_condition.wakeAll()
        self.odom_mutex.unlock()
        
    def get_odom_data(self, current_msg):
        odom_data = current_msg['data']
        x_odom = odom_data.twist.twist.linear.x
        theta_odom = odom_data.twist.twist.angular.z
        time_stamp_odom = odom_data.header.stamp.sec + odom_data.header.stamp.nanosec * 1e-9

        if self.cache_data_enabled:
            data = {'topic': 'odom', 'x_odom': x_odom, 'theta_odom': theta_odom, 'time_stamp': time_stamp_odom}
            self.cache_msg.emit(data)

        return x_odom, theta_odom, time_stamp_odom
    
    def check_convergence(self):
        self.converged = self.pf_engine.check_convergence()
        
        if self.converged and self.stop_when_converged:
            self.pf_active = False
            
    @pyqtSlot()
    def stop_pf(self):
        self.pf_active = False
        self.is_processing = False
    
class PfCachedThread(PfBagThread):
    
    def get_new_data_manager(self):
        self.pf_active = False       
    
    def get_odom_data(self, current_msg):
        odom_data = current_msg['data']
        x_odom = odom_data['x_odom']
        theta_odom = odom_data['theta_odom']
        time_stamp_odom = current_msg['timestamp']

        return x_odom, theta_odom, time_stamp_odom 
    
    def get_data_from_image_msg(self, current_msg):
        
        request = {"current_msg": current_msg}
        
        self.trunk_data_thread.handle_request(request)
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
                 pf_engine: PFEngine,
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
        self.print_message.emit(message)
    
    def run(self):
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
        self.update_test_number.emit(test_name)
    
    def signal_set_time_line(self, current_time):
        self.set_time_line.emit(current_time)
    
    def signal_current_msg(self, current_msg):
        self.get_trunk_data(current_msg)
    
    def signal_update_trial_number(self, trial_num):
        self.update_trial_number.emit(trial_num)
        
    def signal_plot_gt_position(self):
        self.plot_gt_position.emit(self.position_gt)
    
    def signal_plot_particles(self):
        particles = self.pf_engine.downsample_particles()
        self.plot_particles.emit(particles)
        
    def signal_update_image_number(self):
        current_image_position = self.data_manager.current_img_position
        num_img_msgs = self.data_manager.num_img_msgs
        self.update_image_number.emit(current_image_position, num_img_msgs)
    
    def signal_update_trial_info(self):
        particle_count = self.pf_engine.particles.shape[0]
        current_time = time.time() - self.trial_start_time
        self.update_trial_info.emit(current_time, particle_count)
    
    def signal_update_ui_with_trial_results(self, test_info):
        trial_convergence_rate, trial_avg_time_all, trial_avg_time_converged = test_info.get_results()
        # self.main_app_manager.pf_test_controls.update_convergence_rate(trial_convergence_rate, 0)
        # self.main_app_manager.pf_test_controls.update_test_time(trial_avg_time_all, 0)
        self.update_ui_with_trial_results.emit(trial_convergence_rate, trial_avg_time_all, trial_avg_time_converged)

    def get_trunk_data(self, current_msg):
        return self.get_trunk_data_func(current_msg)
    
    def reset_pf(self):
        self.reset_pf_app.emit(False)
        
        # wait for the reset to complete
        time.sleep(0.4)
        
        # self.pf_engine.reset_pf(self.parameters_pf)
    
    @pyqtSlot()
    def stop_pf(self):
        self.pf_active = False
        self.tests_aborted = True
    