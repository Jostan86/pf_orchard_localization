from PyQt5.QtCore import QTimer

class PfMode:
    def __init__(self, main_app_manager):
        self.main_app_manager = main_app_manager

        self.pf_continuous_active = False
        self.converged = False
        self.is_processing = False
        self.timer = None

        self.main_app_manager.control_buttons.startStopButtonClicked.connect(self.start_stop_button_pushed)

        self.mode_active = False

    def start_stop_button_pushed(self, command):

        # Check current text on button
        if command == "Continue":
            self.cont_button_clicked()
        elif command == "Start":
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
            success = self.main_app_manager.load_next_data_file()
            if not success:
                self.stop_pf_continuous()
                self.main_app_manager.print_message("Reached the end of the data files")
                return

            current_msg = self.main_app_manager.data_manager.get_next_msg()

        self.main_app_manager.data_file_time_line.set_time_line(self.main_app_manager.data_manager.current_data_file_time_stamp)

        if current_msg['topic'] == 'odom':
            x_odom, theta_odom, time_stamp_odom = self.get_odom_data(current_msg)
            self.main_app_manager.pf_engine.handle_odom(x_odom, theta_odom, time_stamp_odom)

        elif current_msg['topic'] == 'image':


            # Get trunk data from trunk connection
            positions, widths, class_estimates = self.main_app_manager.trunk_data_connection.get_trunk_data(current_msg)

            if positions is None:
                self.is_processing = False
                return

            self.main_app_manager.image_number_label.set_img_number_label(self.main_app_manager.data_manager.current_img_position,
                                                              self.main_app_manager.data_manager.num_img_msgs)

            tree_data = {'positions': positions, 'widths': widths, 'classes': class_estimates}
            self.main_app_manager.pf_engine.scan_update(tree_data)

            best_guess = self.main_app_manager.pf_engine.best_particle
            self.main_app_manager.plotter.update_particles(self.main_app_manager.pf_engine.downsample_particles())
            self.main_app_manager.plotter.update_position_estimate(best_guess)

        self.converged = self.main_app_manager.pf_engine.check_convergence()
        if self.converged:
            self.stop_pf_continuous()

        self.is_processing = False

    def get_odom_data(self, current_msg):
        odom_data = current_msg['data']

        if self.main_app_manager.using_cached_data:
            x_odom = odom_data['x_odom']
            theta_odom = odom_data['theta_odom']
            time_stamp_odom = current_msg['timestamp']
        else:
            x_odom = odom_data.twist.twist.linear.x
            theta_odom = odom_data.twist.twist.angular.z
            time_stamp_odom = odom_data.header.stamp.to_sec()

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
        self.main_app_manager.start_location_controls.enable()
        self.main_app_manager.control_buttons.enable()
        self.main_app_manager.image_display.enable()
        self.main_app_manager.image_number_label.enable()
        self.main_app_manager.data_file_time_line.enable()
        self.main_app_manager.data_file_controls.enable()

        self.main_app_manager.control_buttons.enable()

    def deactivate_mode(self):
        self.mode_active = False
        self.ensure_pf_stopped()
