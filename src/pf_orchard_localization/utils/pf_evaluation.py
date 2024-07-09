from ..recorded_data_loaders import CachedDataLoader
from ..pf_engine import PFEngine
from map_data_tools import MapData
from .parameters import ParametersPf
import numpy as np
import csv
import os
import time
from PyQt5.QtCore import QThread, pyqtSignal, pyqtSlot
import copy

class PfTest:
    def __init__(self, test_name, start_x, start_y, start_width, start_length, start_rotation, orientation_center,
                 orientation_range, data_file_name, start_time):
        self.test_name = test_name
        self.start_x = start_x
        self.start_y = start_y
        self.start_width = start_width
        self.start_length = start_length
        self.start_rotation = start_rotation
        self.orientation_center = orientation_center
        self.orientation_range = orientation_range
        self.data_file_name = data_file_name
        self.start_time = start_time

        self.results_distances = []
        self.results_convergence_accuracy = []
        self.results_run_times = []

        self.test_completed = False

    def __repr__(self):
        return f"Test Name: {self.test_name}, Start X: {self.start_x}, Start Y: {self.start_y}, Start Width: {self.start_width}, Start Length: {self.start_length}, Start Rotation: {self.start_rotation}, Orientation Center: {self.orientation_center}, Orientation Range: {self.orientation_range}, Data File Name: {self.data_file_name}, Start Time: {self.start_time}"

    def reset_results(self):
        self.test_completed = False
        self.results_distances = []
        self.results_convergence_accuracy = []
        self.results_run_times = []

    def add_results(self, run_time, correct_convergence, distance_to_converge):
        self.results_distances.append(distance_to_converge)
        self.results_convergence_accuracy.append(correct_convergence)
        self.results_run_times.append(run_time)

    def get_results(self):
        convergences = np.array(self.results_convergence_accuracy)
        run_times = np.array(self.results_run_times)

        # Calculate the convergence rate
        convergence_rate = np.sum(convergences) / len(convergences)

        # Calculate the average time to convergence
        avg_time_all = np.sum(run_times) / len(run_times)

        # Calculate the average time for trials that converged
        avg_time_converged = np.sum(run_times * convergences) / np.sum(convergences)

        return convergence_rate, avg_time_all, avg_time_converged

    def set_completed(self):
        self.test_completed = True
        
class PfTestRegimen:
    def __init__(self, test_info_file_path, print_message_func=print):

        self.print_message_func = print_message_func
        
        self.test_names = []
        self.pf_tests = []

        with open(test_info_file_path, newline='') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                self.test_names.append(row['test_name'])
                pf_test = PfTest(row['test_name'],
                                 float(row['start_x']),
                                 float(row['start_y']),
                                 float(row['start_width']),
                                 float(row['start_length']),
                                 float(row['start_rotation']),
                                 float(row['orientation_center']),
                                 float(row['orientation_range']),
                                 row['data_file_name'],
                                 float(row['start_time']))

                self.pf_tests.append(pf_test)

        self.num_tests = len(self.test_names)
        self.current_test = 0

        self.num_trials_per_location = 1

    def process_results(self, save_path=None, overwrite=True, print_results=True):
        
        
        completed_tests = []
        for pf_test in self.pf_tests:
            if pf_test.test_completed:
                completed_tests.append(pf_test)

        avg_convergence_rates = np.zeros(len(completed_tests))
        avg_times_all = np.zeros(len(completed_tests))
        avg_times_converged = np.zeros(len(completed_tests))

        for i in range(len(completed_tests)):
            convergence_rate, avg_time_all, avg_time_converged = completed_tests[i].get_results()
            avg_convergence_rates[i] = convergence_rate
            avg_times_all[i] = avg_time_all
            avg_times_converged[i] = avg_time_converged

        # Calculate the overall average time for trials that converged, removing nan values
        overall_avg_time_converged = np.nanmean(avg_times_converged)

        # Calculate the overall average convergence rate
        overall_avg_convergence_rate = np.mean(avg_convergence_rates)

    
        # Save the results to a csv file
        if save_path is not None:
            if not save_path.endswith(".csv"):
                save_path = save_path + ".csv"
            
            if os.path.exists(save_path) and not overwrite:
                self.print_message_func("File already exists")
                return
            
            with open(save_path, "w") as f:
                writer = csv.writer(f)
                writer.writerow(["Start Location", "Convergence Rate", "Average Time", "Average Time Converged"])
                for i in range(len(completed_tests)):
                    writer.writerow([i, avg_convergence_rates[i], avg_times_all[i], avg_times_converged[i]])

                writer.writerow(["Overall Average Time Converged", overall_avg_time_converged])
                writer.writerow(["Overall Average Convergence Rate", overall_avg_convergence_rate])
                writer.writerow(["Number of Trials per Test", len(completed_tests[0].results_distances)])

        if print_results:
            
            self.print_message_func("Results:")
            for i in range(len(completed_tests)):
                self.print_message_func("Start Location: {}   Convergence Rate: {}   Average Time: {}   Average Time Converged: {}".format(
                    i, avg_convergence_rates[i], avg_times_all[i], avg_times_converged[i]
                ))
            self.print_message_func("Overall Average Time Converged: {}".format(overall_avg_time_converged))
            self.print_message_func("Overall Average Convergence Rate: {}".format(overall_avg_convergence_rate))

    def reset_tests(self):
        for pf_test in self.pf_tests:
            pf_test.reset_results()

    def reset_test(self, test_num):
        self.pf_tests[test_num].reset_results()        

class PfTestExecutor:
    def __init__(self,
                 pf_engine: PFEngine,
                 parameters_pf: ParametersPf,
                 test_info_path: str,
                 cached_data_files_dir: str,
                 num_trials,
                 class_mapping=(1, 2, 0),
                 save_path=None,
                 convergence_threshold=0.5,
                 print_message_func=print):

        self.convergence_threshold = convergence_threshold
        self.print_message_func = print_message_func
        
        self.test_regimen = PfTestRegimen(test_info_path, print_message_func)
        self.cached_data_files_dir = cached_data_files_dir
        self.pf_active = False
        self.pf_engine = pf_engine
        self.parameters_pf = parameters_pf
        self.num_trials = num_trials
        self.class_mapping = class_mapping
        
        self.save_path = save_path
        
        self.data_manager = None
        
        self.position_estimate = None
        self.position_gt = None
       
           
    def signal_update_test_number(self, test_name):
        pass
    
    def signal_set_time_line(self, current_time):
        pass
    
    def signal_current_msg(self, current_msg):
        pass
    
    def signal_update_trial_number(self, trial_num):
        pass
        
    def signal_plot_gt_position(self):
        pass
    
    def signal_plot_particles(self):
        # particles = self.pf_engine.downsample_particles()
        pass
    
    def signal_update_image_number(self):
        # current_image_position = self.data_manager.current_img_position
        # num_img_msgs = self.data_manager.num_img_msgs
        pass
    
    def signal_update_trial_info(self):
        # particle_count = self.main_app_manager.pf_engine.particles.shape[0]
        
        # self.main_app_manager.pf_test_controls.update_trial_info(current_time, particle_count)
        pass
    
    def signal_update_ui_with_trial_results(self, test_info):
        # trial_convergence_rate, trial_avg_time_all, trial_avg_time_converged = test_info.get_results()
        # self.main_app_manager.pf_test_controls.update_convergence_rate(trial_convergence_rate, 0)
        # self.main_app_manager.pf_test_controls.update_test_time(trial_avg_time_all, 0)
        pass
    
    def process_results(self, save_path):
        self.test_regimen.process_results(save_path=save_path)
    
    def stop_pf(self):
        self.pf_active = False
        self.tests_aborted = True
        
    def run_all_tests(self):
        self.tests_aborted = False

        self.print_message_func("Running all tests")

        for test_info in self.test_regimen.pf_tests:
            
            self.run_test(test_info)
            
            if self.tests_aborted:
                break
            
            test_info.test_completed = True
            
        if self.save_path is not None and not self.tests_aborted:
            self.process_results(self.save_path)

    def run_selected_test(self, test_index):
        self.tests_aborted = False
        
        test_info = self.test_regimen.pf_tests[test_index]

        self.run_test(test_info)

            
    def run_test(self, test_info):
        
        self.signal_update_test_number(test_info.test_name)

        self.print_message_func("Running test: " + test_info.test_name)

        self.reset_for_test(test_info)
        
        if self.tests_aborted:
                return

        for trial_num in range(self.num_trials):
            self.print_message_func("Starting trial " + str(trial_num + 1))
            self.signal_update_trial_number(trial_num + 1)
            self.run_trial(test_info)
            
            if self.tests_aborted:
                return
            
            self.signal_update_ui_with_trial_results(test_info)
            
    def reset_for_test(self, test_info):
        
        data_file_path = os.path.join(self.cached_data_files_dir, test_info.data_file_name + ".json")
        self.data_manager = CachedDataLoader(data_file_path)

        self.parameters_pf.start_pose_center_x = test_info.start_x
        self.parameters_pf.start_pose_center_y = test_info.start_y
        self.parameters_pf.start_width = test_info.start_width
        self.parameters_pf.start_height = test_info.start_length
        self.parameters_pf.start_rotation = test_info.start_rotation
        self.parameters_pf.start_orientation_center = test_info.orientation_center
        self.parameters_pf.start_orientation_range = test_info.orientation_range

        self.reset_for_trial(test_info)

    def reset_for_trial(self, test_info):
        
        message, current_msg = self.data_manager.set_time_stamp(test_info.start_time)
        self.signal_current_msg(current_msg)
        
        self.reset_pf()
    
    def reset_pf(self):
        
        self.pf_engine.reset_pf(self.parameters_pf)
        
        # self.main_app_manager.data_file_time_line.set_time_line(test_info.start_time)
        # self.main_app_manager.data_file_time_line_edited()
        # self.main_app_manager.reset_pf(use_ui_parameters=False)

    def run_trial(self, test_info):

        self.reset_for_trial(test_info)

        self.trial_start_time = time.time()
        
        self.pf_active = True

        while self.pf_active:
            self.send_next_msg()

        if self.tests_aborted:
            self.print_message_func("Test aborted")
            return

        trial_time = time.time() - self.trial_start_time
        
        correct_convergence, distance = self.check_converged_location()

        test_info.add_results(trial_time, correct_convergence, distance)

    def send_next_msg(self):
        current_msg = self.data_manager.get_next_msg()

        if current_msg is None:
            self.pf_active = False
            return

        self.signal_set_time_line(self.data_manager.current_data_file_time_stamp)

        if current_msg['topic'] == 'odom':
            x_odom, theta_odom, time_stamp_odom = self.get_odom_data(current_msg)
            self.pf_engine.handle_odom(x_odom, theta_odom, time_stamp_odom)

        elif current_msg['topic'] == 'image':

            self.get_data_from_image_msg(current_msg)

        self.converged = self.pf_engine.check_convergence()
        
        if self.converged:
            self.pf_active = False
            
        self.signal_update_trial_info()
        
    def get_odom_data(self, current_msg):
        odom_data = current_msg['data']
        x_odom = odom_data['x_odom']
        theta_odom = odom_data['theta_odom']
        time_stamp_odom = current_msg['timestamp']

        return x_odom, theta_odom, time_stamp_odom 

    def get_data_from_image_msg(self, current_msg):

        positions, widths, class_estimates = self.get_trunk_data(current_msg)

        if positions is None:
            return

        tree_data = {'positions': positions, 'widths': widths, 'classes': class_estimates}
        self.pf_engine.scan_update(tree_data)

        actual_position = current_msg['data']['location_estimate']
        self.position_gt = np.array([actual_position['x'], actual_position['y']])
        
        self.signal_plot_gt_position()
        self.signal_plot_particles()
        self.signal_update_image_number()
        
    def get_trunk_data(self, current_msg):

        if current_msg['data'] is None:
            return None, None, None

        msg_data = current_msg['data']['tree_data']
        self.positions = np.array(msg_data['positions'])
        self.widths = np.array(msg_data['widths'])
        self.class_estimates = np.array(msg_data['classes'], dtype=np.int32)

        self.class_estimates = self.remap_classes(self.class_estimates)

        return self.positions, self.widths, self.class_estimates
    
    def remap_classes(self, class_estimates):
        class_estimates_copy = class_estimates.copy()
        for i, class_num in enumerate(self.class_mapping):
            class_estimates_copy[class_estimates == i] = class_num

        return class_estimates_copy
    
    def check_converged_location(self):
        position_estimate = self.pf_engine.best_particle[0:2]

        actual_position = self.position_gt[0:2]

        distance = np.linalg.norm(position_estimate - actual_position)
        if distance < self.convergence_threshold:
            return True, distance
        else:
            return False, distance
        
    
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
    






