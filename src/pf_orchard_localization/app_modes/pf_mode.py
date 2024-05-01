from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QHBoxLayout, QVBoxLayout

class PfMode:
    def __init__(self, main_app_manager):
        self.main_app_manager = main_app_manager

        self.pf_continuous_active = False
        self.converged = False
        self.is_processing = False
        self.timer = None

        self.main_app_manager.control_buttons.startStopButtonClicked.connect(self.start_stop_button_pushed)
        self.main_app_manager.control_buttons.single_step_button.clicked.connect(self.cont_button_clicked)

        self.mode_active = False

    def start_stop_button_pushed(self, command):

        # Check current text on button
        if command == "Start":
            self.start_pf_continuous()
        elif command == "Stop":
            self.stop_pf_continuous()

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
        self.main_app_manager.control_buttons.adjust_pf_settings_button.setEnabled(False)
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
        self.main_app_manager.control_buttons.adjust_pf_settings_button.setEnabled(True)
        self.main_app_manager.data_file_controls.data_file_open_button.setEnabled(True)
        self.main_app_manager.start_location_controls.enable()

    def activate_mode(self):
        self.mode_active = True

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
        self.main_app_manager.ui_layout.addWidget(self.main_app_manager.cached_data_creator)
        self.main_app_manager.ui_layout.addWidget(self.main_app_manager.console)

        self.main_app_manager.main_layout.addLayout(self.main_app_manager.ui_layout)
        self.main_app_manager.main_layout.addWidget(self.main_app_manager.plotter)

        self.main_app_manager.reset_pf()

    def deactivate_mode(self):
        self.mode_active = False
        self.main_app_manager.cached_data_creator.enable_checkbox.setChecked(False)
        self.ensure_pf_stopped()

class PfModeCached(PfMode):
    def __init__(self, main_app_manager):
        super().__init__(main_app_manager)

    def get_data_from_image_msg(self, current_msg):

        positions, widths, class_estimates = self.main_app_manager.trunk_data_connection.get_trunk_data(current_msg)

        if positions is None:
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

    def get_odom_data(self, current_msg):
        odom_data = current_msg['data']
        x_odom = odom_data['x_odom']
        theta_odom = odom_data['theta_odom']
        time_stamp_odom = current_msg['timestamp']

        return x_odom, theta_odom, time_stamp_odom

    def activate_mode(self):
        self.mode_active = True

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

        self.main_app_manager.reset_pf()

    def deactivate_mode(self):
        self.mode_active = False
        self.ensure_pf_stopped()