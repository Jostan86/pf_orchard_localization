from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QHBoxLayout, QVBoxLayout, QApplication

import time
import numpy as np

class PfMode:
    def __init__(self, main_app_manager):
        self.main_app_manager = main_app_manager

        self.pf_continuous_active = False
        self.converged = False
        self.is_processing = False
        self.timer = None

        self.mode_active = False

        self.mode_name = "PF - Recorded Data"

    def ensure_pf_stopped(self):
        if self.pf_continuous_active:
            self.stop_pf_continuous()

    def send_next_msg(self):
        if self.is_processing:
            return

        self.is_processing = True

        current_msg = self.main_app_manager.data_manager.get_next_msg()

        if current_msg is None:
            success = self.main_app_manager.load_next_data_file(load_first_image=False)
            if not success:
                self.stop_pf_continuous()
                self.main_app_manager.print_message("Failed to load next data file. Stopping particle filter.")
                return

            current_msg = self.main_app_manager.data_manager.get_next_msg()

        self.main_app_manager.data_file_time_line.set_time_line(self.main_app_manager.data_manager.current_data_file_time_stamp)

        if current_msg['topic'] == 'odom':
            x_odom, theta_odom, time_stamp_odom = self.get_odom_data(current_msg)
            self.main_app_manager.pf_engine.handle_odom(x_odom, theta_odom, time_stamp_odom)

        elif current_msg['topic'] == 'image':

            self.get_data_from_image_msg(current_msg)

            if not self.is_processing:
                return

        self.converged = self.main_app_manager.pf_engine.check_convergence()
        if self.converged and self.main_app_manager.parameters_pf.stop_when_converged:
            self.stop_pf_continuous()

        self.is_processing = False

    def get_data_from_image_msg(self, current_msg):

        positions, widths, class_estimates, seg_img = self.main_app_manager.trunk_data_connection.get_trunk_data(
                                                                current_msg, return_seg_img=True)

        if positions is None:
            if self.main_app_manager.cached_data_creator.cache_data_enabled:
                self.main_app_manager.cached_data_creator.cache_tree_data(None, None, None, None,
                                                                          current_msg['timestamp'])
            self.is_processing = False
            return

        self.main_app_manager.image_number_label.set_img_number_label(
            self.main_app_manager.data_manager.current_img_position,
            self.main_app_manager.data_manager.num_img_msgs)

        tree_data = {'positions': positions, 'widths': widths, 'classes': class_estimates}
        self.main_app_manager.pf_engine.scan_update(tree_data)

        best_guess = self.main_app_manager.pf_engine.best_particle
        self.main_app_manager.plotter.update_particles(self.main_app_manager.pf_engine.downsample_particles())
        self.main_app_manager.plotter.update_position_estimate(best_guess)

        if self.main_app_manager.cached_data_creator.cache_data_enabled:
            self.main_app_manager.cached_data_creator.cache_tree_data(positions, widths, class_estimates, best_guess,
                                                                      current_msg['timestamp'])
            self.main_app_manager.cached_data_creator.save_image(seg_img, current_msg['timestamp'])

    def get_odom_data(self, current_msg):
        odom_data = current_msg['data']
        x_odom = odom_data.twist.twist.linear.x
        theta_odom = odom_data.twist.twist.angular.z
        time_stamp_odom = odom_data.header.stamp.to_sec()

        if self.main_app_manager.cached_data_creator.cache_data_enabled:
            self.main_app_manager.cached_data_creator.cache_odom_data(x_odom, theta_odom, time_stamp_odom)

        return x_odom, theta_odom, time_stamp_odom

    def cont_button_clicked(self):
        self.send_next_msg()
        while not self.main_app_manager.data_manager.at_img_msg:
            self.send_next_msg()

    def start_pf_continuous(self):
        self.main_app_manager.control_buttons.set_stop()
        self.main_app_manager.change_parameters_button.disable()
        self.main_app_manager.data_file_controls.data_file_open_button.setEnabled(False)
        self.main_app_manager.start_location_controls.disable()
        self.pf_continuous_active = True
        self.timer = QTimer()
        self.timer.timeout.connect(self.send_next_msg)
        self.timer.start(self.main_app_manager.image_delay_slider.get_delay_ms())

    def stop_pf_continuous(self):
        self.timer.stop()
        self.pf_continuous_active = False
        self.is_processing = False
        self.main_app_manager.control_buttons.set_start()
        self.main_app_manager.change_parameters_button.enable()
        self.main_app_manager.data_file_controls.data_file_open_button.setEnabled(True)
        self.main_app_manager.start_location_controls.enable()

    def activate_mode(self):
        self.mode_active = True

        self.setup_gui()
        self.connect_gui()

        self.main_app_manager.reset_pf()

    def setup_gui(self):
        mode_change_button_layout = QHBoxLayout()
        mode_change_button_layout.addWidget(self.main_app_manager.mode_selector)
        mode_change_button_layout.addWidget(self.main_app_manager.change_parameters_button)

        img_delay_time_line_layout = QHBoxLayout()
        img_delay_time_line_layout.addWidget(self.main_app_manager.data_file_time_line)
        img_delay_time_line_layout.addWidget(self.main_app_manager.image_delay_slider)

        self.main_app_manager.ui_layout.addLayout(mode_change_button_layout)
        self.main_app_manager.ui_layout.addWidget(self.main_app_manager.checkboxes)
        self.main_app_manager.ui_layout.addWidget(self.main_app_manager.start_location_controls)
        self.main_app_manager.ui_layout.addWidget(self.main_app_manager.control_buttons)
        self.main_app_manager.ui_layout.addWidget(self.main_app_manager.image_display)
        self.main_app_manager.ui_layout.addWidget(self.main_app_manager.image_number_label)
        self.main_app_manager.ui_layout.addLayout(img_delay_time_line_layout)
        self.main_app_manager.ui_layout.addWidget(self.main_app_manager.data_file_controls)
        self.main_app_manager.ui_layout.addWidget(self.main_app_manager.cached_data_creator)
        self.main_app_manager.ui_layout.addWidget(self.main_app_manager.console)

        self.main_app_manager.main_layout.addLayout(self.main_app_manager.ui_layout)
        self.main_app_manager.main_layout.addWidget(self.main_app_manager.plotter)

    def connect_gui(self):
        self.main_app_manager.control_buttons.startButtonClicked.connect(self.start_pf_continuous)
        self.main_app_manager.control_buttons.stopButtonClicked.connect(self.stop_pf_continuous)
        self.main_app_manager.control_buttons.single_step_button.clicked.connect(self.cont_button_clicked)

    def deactivate_mode(self):

        self.disconnect_gui()

        self.mode_active = False

        self.ensure_pf_stopped()

    def disconnect_gui(self):
        self.main_app_manager.control_buttons.startButtonClicked.disconnect(self.start_pf_continuous)
        self.main_app_manager.control_buttons.stopButtonClicked.disconnect(self.stop_pf_continuous)
        self.main_app_manager.control_buttons.single_step_button.clicked.disconnect(self.cont_button_clicked)
        self.main_app_manager.cached_data_creator.enable_checkbox.setChecked(False)

    def shutdown_hook(self):
        self.ensure_pf_stopped()


class PfModeCached(PfMode):
    def __init__(self, main_app_manager):
        super().__init__(main_app_manager)
        self.position_estimate = None
        self.position_gt = None

    def get_data_from_image_msg(self, current_msg):

        positions, widths, class_estimates = self.main_app_manager.trunk_data_connection.get_trunk_data(current_msg)

        if positions is None:
            self.is_processing = False
            return

        tree_data = {'positions': positions, 'widths': widths, 'classes': class_estimates}
        self.main_app_manager.pf_engine.scan_update(tree_data)

        actual_position = (current_msg['data']['location_estimate'])
        self.position_gt = np.array([actual_position['x'], actual_position['y']])
        self.main_app_manager.plotter.update_actual_position(self.position_gt)

        self.position_estimate = self.main_app_manager.pf_engine.best_particle
        self.main_app_manager.plotter.update_particles(self.main_app_manager.pf_engine.downsample_particles())
        self.main_app_manager.plotter.update_position_estimate(self.position_estimate)

        self.main_app_manager.image_number_label.set_img_number_label(
            self.main_app_manager.data_manager.current_img_position,
            self.main_app_manager.data_manager.num_img_msgs)

    def get_odom_data(self, current_msg):
        odom_data = current_msg['data']
        x_odom = odom_data['x_odom']
        theta_odom = odom_data['theta_odom']
        time_stamp_odom = current_msg['timestamp']

        return x_odom, theta_odom, time_stamp_odom

    def setup_gui(self):

        mode_change_button_layout = QHBoxLayout()
        mode_change_button_layout.addWidget(self.main_app_manager.mode_selector)
        mode_change_button_layout.addWidget(self.main_app_manager.change_parameters_button)

        img_delay_time_line_layout = QHBoxLayout()
        img_delay_time_line_layout.addWidget(self.main_app_manager.data_file_time_line)
        img_delay_time_line_layout.addWidget(self.main_app_manager.image_delay_slider)

        self.main_app_manager.ui_layout.addWidget(self.main_app_manager.mode_selector)
        self.main_app_manager.ui_layout.addWidget(self.main_app_manager.checkboxes)
        self.main_app_manager.ui_layout.addWidget(self.main_app_manager.start_location_controls)
        self.main_app_manager.ui_layout.addWidget(self.main_app_manager.control_buttons)
        self.main_app_manager.ui_layout.addWidget(self.main_app_manager.image_display)
        self.main_app_manager.ui_layout.addWidget(self.main_app_manager.image_number_label)
        self.main_app_manager.ui_layout.addLayout(img_delay_time_line_layout)
        self.main_app_manager.ui_layout.addWidget(self.main_app_manager.data_file_controls)
        self.main_app_manager.ui_layout.addWidget(self.main_app_manager.console)

        self.main_app_manager.main_layout.addLayout(self.main_app_manager.ui_layout)
        self.main_app_manager.main_layout.addWidget(self.main_app_manager.plotter)

    def connect_gui(self):
        self.main_app_manager.control_buttons.startButtonClicked.connect(self.start_pf_continuous)
        self.main_app_manager.control_buttons.stopButtonClicked.connect(self.stop_pf_continuous)
        self.main_app_manager.control_buttons.single_step_button.clicked.connect(self.cont_button_clicked)


    def disconnect_gui(self):

        self.main_app_manager.control_buttons.startButtonClicked.disconnect(self.start_pf_continuous)
        self.main_app_manager.control_buttons.stopButtonClicked.disconnect(self.stop_pf_continuous)
        self.main_app_manager.control_buttons.single_step_button.disconnect(self.cont_button_clicked)


class PfModeCachedTests(PfModeCached):
    def __init__(self, main_app_manager):
        super().__init__(main_app_manager)
        self.convergence_threshold = 0.5
        self.mode_name = "PF - Recorded Data Tests"

    def run_all_tests(self):
        self.tests_aborted = False
        self.main_app_manager.pf_test_controls.set_running_all_tests(True)

        self.main_app_manager.print_message("Running all tests")

        for test_info in self.main_app_manager.pf_test_controls.test_regimen.pf_tests:
            self.run_test(test_info)
            if self.tests_aborted:
                break
            test_info.test_completed = True

        self.main_app_manager.pf_test_controls.set_running_all_tests(False)

    def abort_all_tests(self):
        self.pf_active = False
        self.tests_aborted = True

    def run_selected_test(self):
        self.tests_aborted = False
        self.main_app_manager.pf_test_controls.set_running_selected_test(True)

        current_selection = self.main_app_manager.pf_test_controls.get_selected_test()
        test_info = self.main_app_manager.pf_test_controls.test_regimen.pf_tests[current_selection]

        self.run_test(test_info)

        if self.tests_aborted:
            test_info.reset_results()



        self.main_app_manager.pf_test_controls.set_running_selected_test(False)

    def run_test(self, test_info):
        self.main_app_manager.pf_test_controls.update_test_number(test_info.test_name)

        self.main_app_manager.print_message("Running test: " + test_info.test_name)

        self.load_test_data(test_info=test_info)

        num_trials = self.main_app_manager.pf_test_controls.get_num_trials_per_location()

        test_info.reset_results()

        data_file_name = test_info.data_file_name + ".json"

        # TODO: Does this auto change the file combo box?
        self.main_app_manager.open_data_file(data_file_name)

        for trial_num in range(num_trials):
            self.main_app_manager.print_message("Starting trial " + str(trial_num + 1))
            self.main_app_manager.pf_test_controls.update_trial_number(trial_num + 1)
            self.run_trial(test_info)
            if self.tests_aborted:
                return
            trial_convergence_rate, trial_avg_time_all, trial_avg_time_converged = test_info.get_results()
            self.main_app_manager.pf_test_controls.update_convergence_rate(trial_convergence_rate, 0)
            self.main_app_manager.pf_test_controls.update_test_time(trial_avg_time_all, 0)


        # self.main_app_manager.change_parameters_button.enable()

    def load_test_data(self, test_index=None, test_info=None):

        if test_index is not None:
            test_info = self.main_app_manager.pf_test_controls.test_regimen.pf_tests[test_index]

        # self.main_app_manager.change_parameters_button.disable()

        self.main_app_manager.parameters_pf.start_pose_center_x = test_info.start_x
        self.main_app_manager.parameters_pf.start_pose_center_y = test_info.start_y
        self.main_app_manager.parameters_pf.start_width = test_info.start_width
        self.main_app_manager.parameters_pf.start_height = test_info.start_length
        self.main_app_manager.parameters_pf.start_rotation = test_info.start_rotation
        self.main_app_manager.parameters_pf.start_orientation_center = test_info.orientation_center
        self.main_app_manager.parameters_pf.start_orientation_range = test_info.orientation_range

        self.reset_for_trial(test_info)

    def reset_for_trial(self, test_info):
        self.main_app_manager.data_file_time_line.set_time_line(test_info.start_time)
        self.main_app_manager.data_file_time_line_edited()
        self.main_app_manager.reset_pf(use_ui_parameters=False)

    def run_trial(self, test_info):

        self.reset_for_trial(test_info)

        self.trial_start_time = time.time()
        self.pf_active = True

        while self.pf_active:
            self.send_next_msg()

        if self.tests_aborted:
            self.main_app_manager.print_message("Test aborted")
            return

        trial_time = time.time() - self.trial_start_time
        correct_convergence, distance = self.check_converged_location()

        test_info.add_results(trial_time, correct_convergence, distance)

    def send_next_msg(self):
        current_msg = self.main_app_manager.data_manager.get_next_msg()

        if current_msg is None:
            self.pf_active = False
            return

        self.main_app_manager.data_file_time_line.set_time_line(self.main_app_manager.data_manager.current_data_file_time_stamp)

        if current_msg['topic'] == 'odom':
            x_odom, theta_odom, time_stamp_odom = self.get_odom_data(current_msg)
            self.main_app_manager.pf_engine.handle_odom(x_odom, theta_odom, time_stamp_odom)

        elif current_msg['topic'] == 'image':

            self.get_data_from_image_msg(current_msg)

        self.converged = self.main_app_manager.pf_engine.check_convergence()
        if self.converged:
            self.pf_active = False

        current_time = time.time() - self.trial_start_time
        particle_count = self.main_app_manager.pf_engine.particles.shape[0]
        self.main_app_manager.pf_test_controls.update_trial_info(current_time, particle_count)

    def check_converged_location(self):
        position_estimate = self.position_estimate[0:2]

        actual_position = self.position_gt[0:2]

        distance = np.linalg.norm(position_estimate - actual_position)
        if distance < self.convergence_threshold:
            return True, distance
        else:
            return False, distance

    def setup_gui(self):

        mode_change_button_layout = QHBoxLayout()
        mode_change_button_layout.addWidget(self.main_app_manager.mode_selector)
        mode_change_button_layout.addWidget(self.main_app_manager.change_parameters_button)

        self.main_app_manager.ui_layout.addWidget(self.main_app_manager.mode_selector)
        self.main_app_manager.ui_layout.addWidget(self.main_app_manager.checkboxes)
        self.main_app_manager.ui_layout.addWidget(self.main_app_manager.image_display)
        self.main_app_manager.ui_layout.addWidget(self.main_app_manager.image_number_label)
        self.main_app_manager.ui_layout.addWidget(self.main_app_manager.pf_test_controls)
        self.main_app_manager.ui_layout.addWidget(self.main_app_manager.data_file_time_line)
        self.main_app_manager.ui_layout.addWidget(self.main_app_manager.data_file_controls)
        self.main_app_manager.ui_layout.addWidget(self.main_app_manager.console)

        self.main_app_manager.main_layout.addLayout(self.main_app_manager.ui_layout)
        self.main_app_manager.main_layout.addWidget(self.main_app_manager.plotter)

        self.main_app_manager.data_file_controls.disable()
        #TODO: see if this works or needs to be changed to setReadOnly
        self.main_app_manager.data_file_time_line.disable()

    def connect_gui(self):

        self.main_app_manager.pf_test_controls.runAllTestsClicked.connect(self.run_all_tests)
        self.main_app_manager.pf_test_controls.abortAllTestsClicked.connect(self.abort_all_tests)
        self.main_app_manager.pf_test_controls.runSelectedTestClicked.connect(self.run_selected_test)
        self.main_app_manager.pf_test_controls.abortSelectedTestClicked.connect(self.abort_all_tests)
        self.main_app_manager.pf_test_controls.test_selection_combobox.currentIndexChanged.connect(self.load_test_data)

    def deactivate_mode(self):
        self.disconnect_gui()

        self.mode_active = False
        self.abort_all_tests()

    def disconnect_gui(self):
        self.main_app_manager.data_file_controls.enable()
        self.main_app_manager.data_file_time_line.enable()

        # disconnect signals
        self.main_app_manager.pf_test_controls.runAllTestsClicked.disconnect(self.run_all_tests)
        self.main_app_manager.pf_test_controls.abortAllTestsClicked.disconnect(self.abort_all_tests)
        self.main_app_manager.pf_test_controls.runSelectedTestClicked.disconnect(self.run_selected_test)
        self.main_app_manager.pf_test_controls.abortSelectedTestClicked.disconnect(self.abort_all_tests)
        self.main_app_manager.pf_test_controls.test_selection_combobox.currentIndexChanged.disconnect(
            self.load_test_data)

    def shutdown_hook(self):
        self.abort_all_tests()

class PfModeSaveCalibrationData(PfMode):
    def __init__(self, main_app_manager):
        super().__init__(main_app_manager)
        self.mode_name = "PF - Save Calibration Data"

    def setup_gui(self):
        mode_change_button_layout = QHBoxLayout()
        mode_change_button_layout.addWidget(self.main_app_manager.mode_selector)
        mode_change_button_layout.addWidget(self.main_app_manager.change_parameters_button)

        img_delay_time_line_layout = QHBoxLayout()
        img_delay_time_line_layout.addWidget(self.main_app_manager.data_file_time_line)
        img_delay_time_line_layout.addWidget(self.main_app_manager.image_delay_slider)

        self.main_app_manager.ui_layout.addLayout(mode_change_button_layout)
        self.main_app_manager.ui_layout.addWidget(self.main_app_manager.checkboxes)
        self.main_app_manager.ui_layout.addWidget(self.main_app_manager.start_location_controls)
        self.main_app_manager.ui_layout.addWidget(self.main_app_manager.control_buttons)
        self.main_app_manager.ui_layout.addWidget(self.main_app_manager.image_display)
        self.main_app_manager.ui_layout.addWidget(self.main_app_manager.image_number_label)
        self.main_app_manager.ui_layout.addLayout(img_delay_time_line_layout)
        self.main_app_manager.ui_layout.addWidget(self.main_app_manager.data_file_controls)
        self.main_app_manager.ui_layout.addWidget(self.main_app_manager.save_calibration_data_controls)
        self.main_app_manager.ui_layout.addWidget(self.main_app_manager.console)

        self.main_app_manager.main_layout.addLayout(self.main_app_manager.ui_layout)
        self.main_app_manager.main_layout.addWidget(self.main_app_manager.plotter)

    # def connect_gui(self):
    #     super().connect_gui()
    #     self.main_app_manager.save_calibration_data_controls.saveButtonClicked.connect(self.save_calibration_data)

    def get_data_from_image_msg(self, current_msg):

        # positions, widths, class_estimates, seg_img = self.main_app_manager.trunk_data_connection.get_trunk_data(
        #                                                         current_msg, return_seg_img=True)
        #
        # if positions is None:
        #     if self.main_app_manager.cached_data_creator.cache_data_enabled:
        #         self.main_app_manager.cached_data_creator.cache_tree_data(None, None, None, None,
        #                                                                   current_msg['timestamp'])
        #     self.is_processing = False
        #     return
        #
        # self.main_app_manager.image_number_label.set_img_number_label(
        #     self.main_app_manager.data_manager.current_img_position,
        #     self.main_app_manager.data_manager.num_img_msgs)
        #
        # tree_data = {'positions': positions, 'widths': widths, 'classes': class_estimates}
        # self.main_app_manager.pf_engine.scan_update(tree_data)
        #
        # best_guess = self.main_app_manager.pf_engine.best_particle
        # self.main_app_manager.plotter.update_particles(self.main_app_manager.pf_engine.downsample_particles())
        # self.main_app_manager.plotter.update_position_estimate(best_guess)
        #
        # if self.main_app_manager.cached_data_creator.cache_data_enabled:
        #     self.main_app_manager.cached_data_creator.cache_tree_data(positions, widths, class_estimates, best_guess,
        #                                                               current_msg['timestamp'])
        #     self.main_app_manager.cached_data_creator.save_image(seg_img, current_msg['timestamp'])
        super().get_data_from_image_msg(current_msg)

        if self.main_app_manager.save_calibration_data_controls.save_data_enabled:
            self.main_app_manager.save_calibration_data_controls.save_data(current_msg)

