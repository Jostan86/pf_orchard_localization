from ..recorded_data_loaders import CachedDataLoader
from ..pf_engine import PfEngine
from .parameters import ParametersPf
import numpy as np
import csv
import os
import time

class PfTest:
    """
    Class to store and process information for a single test
    """
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
        """
        Returns a string representation of the test data
        """
        return f"Test Name: {self.test_name}, Start X: {self.start_x}, Start Y: {self.start_y}, Start Width: {self.start_width}, Start Length: {self.start_length}, Start Rotation: {self.start_rotation}, Orientation Center: {self.orientation_center}, Orientation Range: {self.orientation_range}, Data File Name: {self.data_file_name}, Start Time: {self.start_time}"

    def reset_results(self):
        """
        Reset the results for the test
        """
        self.test_completed = False
        self.results_distances = []
        self.results_convergence_accuracy = []
        self.results_run_times = []

    def add_results(self, run_time, correct_convergence, distance_to_converge):
        """
        Add the results of a trial to the test info

        Args:
            run_time: The time it took for the trial to run
            correct_convergence: Whether the trial converged to the correct location
            distance_to_converge: The distance traveled before converging
        """
        self.results_distances.append(distance_to_converge)
        self.results_convergence_accuracy.append(correct_convergence)
        self.results_run_times.append(run_time)

    def get_results(self):
        """
        Calculate the results for the test

        Returns:
            convergence_rate (float): The average convergence rate for the test
            avg_time_all (float): The average time to convergence for all trials
            avg_time_converged (float): The average time to convergence for trials that converged
        """
        convergences = np.array(self.results_convergence_accuracy)
        run_times = np.array(self.results_run_times)

        convergence_rate = np.sum(convergences) / len(convergences)

        avg_time_all = np.sum(run_times) / len(run_times)

        avg_time_converged = np.sum(run_times * convergences) / np.sum(convergences)

        return convergence_rate, avg_time_all, avg_time_converged

    def set_completed(self):
        """
        Set the test as completed
        """
        self.test_completed = True
        
class PfTestRegimen:
    """
    Class to store and process information for a set of tests
    """

    def __init__(self, test_info_file_path, print_message_func=print):
        """        
        Args:
            test_info_file_path (str): The path to the csv file containing the test information
            print_message_func (function): The function to use for printing messages
        """

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
        """
        Process the results of the tests
        
        Args:
            save_path (str): The path to save the results to
            overwrite (bool): Whether to overwrite the file if it already exists
            print_results (bool): Whether to print the results to the console
        """  
        
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
        """
        Reset the results for all tests
        """
        for pf_test in self.pf_tests:
            pf_test.reset_results()

    def reset_test(self, test_num):
        """
        Reset the results for a single test
        """
        self.pf_tests[test_num].reset_results()        

class PfTestExecutor:
    """
    Class to execute a set of tests using a particle filter, at this point this is a simplified headless version of what the app does.
    """
    #TODO: explore somehow having a single set of code for running the particle filter, instead of here and in the app. This seems 
    # difficult, so another option may just be to setup a way to ensure changes to one are reflected in the other.
    def __init__(self,
                 pf_engine: PfEngine,
                 parameters_pf: ParametersPf,
                 test_info_path: str,
                 cached_data_files_dir: str,
                 num_trials,
                 class_mapping=(1, 2, 0),
                 save_path=None,
                 convergence_threshold=0.5,
                 print_message_func=print):
        """
        Args:
            pf_engine (PfEngine): The particle filter engine
            parameters_pf (ParametersPf): The parameters for the particle filter
            test_info_path (str): The path to the csv file containing the test information
            cached_data_files_dir (str): The directory containing the cached data files
            num_trials (int): The number of trials to run for each test
            class_mapping (tuple, optional): The mapping of classes from the trunk width estimation package to this one. Defaults to (1, 2, 0).
            save_path (str, optional): The path to save the results to. Defaults to None.
            convergence_threshold (float, optional): The distance at which the particle filter is considered to have converged. Defaults to 0.5.
            print_message_func (function, optional): The function to use for printing messages. Defaults to print.
        """

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
    
    ### ---------
    # A bunch of functions that can be overridden to display the test status in a UI or other way
    ### --------- 
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
        pass
    
    def signal_update_image_number(self):
        pass
    
    def signal_update_trial_info(self):
        pass
    
    def signal_update_ui_with_trial_results(self, test_info):
        pass
    
    def process_results(self, save_path):
        self.test_regimen.process_results(save_path=save_path)
    
    def stop_pf(self):
        """
        Stop the particle filter gracefully
        """
        self.pf_active = False
        self.tests_aborted = True
        
    def run_all_tests(self):
        """
        Run all tests in the test regimen
        """
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
        """
        Run a single test from the test regimen
        
        Args:
            test_index (int): The index of the test to run
        """
        self.tests_aborted = False
        
        test_info = self.test_regimen.pf_tests[test_index]

        self.run_test(test_info)

            
    def run_test(self, test_info):
        """
        Run a single test
        
        Args:
            test_info (PfTest): The test info for the test to run
        """
        
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
        """
        Reset the particle filter for a new test

        Args:
            test_info (PfTest): The test info for the test to reset for
        """
        
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
        """
        Reset the particle filter for a new trial of the test

        Args:
            test_info (PfTest): The test info for the test to reset for
        """
        
        message, current_msg = self.data_manager.set_time_stamp(test_info.start_time)
        self.signal_current_msg(current_msg)
        
        self.reset_pf()
    
    def reset_pf(self):
        """
        Reset the particle filter
        """
        self.pf_engine.reset_pf(self.parameters_pf)

    def run_trial(self, test_info):
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

        if self.tests_aborted:
            self.print_message_func("Test aborted")
            return

        trial_time = time.time() - self.trial_start_time
        
        correct_convergence, distance = self.check_converged_location()

        test_info.add_results(trial_time, correct_convergence, distance)

    def send_next_msg(self):
        """
        Send the next message to the particle filter
        """
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
        """
        Get the odometry data from the current message
        
        Args:
            current_msg (dict): The current message
            
        Returns:
            x_odom (float): The x odometry value
            theta_odom (float): The theta odometry value
            time_stamp_odom (float): The odometry time stamp
        """
        odom_data = current_msg['data']
        x_odom = odom_data['x_odom']
        theta_odom = odom_data['theta_odom']
        time_stamp_odom = current_msg['timestamp']

        return x_odom, theta_odom, time_stamp_odom 

    def get_data_from_image_msg(self, current_msg):
        """
        Get the data from the image message

        Args:
            current_msg (dict): The current message
        """
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
        """
        Get the trunk data for the current message

        Args:
            current_msg (dict): The current message

        Returns:
            positions (np.array): The trunk positions
            widths (np.array): The trunk widths
            class_estimates (np.array): The class estimates for the trunks
        """
        if current_msg['data'] is None:
            return None, None, None

        msg_data = current_msg['data']['tree_data']
        self.positions = np.array(msg_data['positions'])
        self.widths = np.array(msg_data['widths'])
        self.class_estimates = np.array(msg_data['classes'], dtype=np.int32)

        self.class_estimates = self.remap_classes(self.class_estimates)

        return self.positions, self.widths, self.class_estimates
    
    def remap_classes(self, class_estimates):
        """
        Remap the classes from the trunk width estimation package to this one

        Args:
            class_estimates (np.array): The class estimates

        Returns:
            np.array: The remapped class estimates
        """
        class_estimates_copy = class_estimates.copy()
        for i, class_num in enumerate(self.class_mapping):
            class_estimates_copy[class_estimates == i] = class_num

        return class_estimates_copy
    
    def check_converged_location(self):
        """
        Check if the particle filter has converged to the correct location

        Returns:
            correct_convergence (bool): Whether the particle filter converged to the correct location
            distance (float): The distance the particle filter traveled before converging
        """
        position_estimate = self.pf_engine.best_particle[0:2]

        actual_position = self.position_gt[0:2]

        distance = np.linalg.norm(position_estimate - actual_position)
        if distance < self.convergence_threshold:
            return True, distance
        else:
            return False, distance
        
    

    






