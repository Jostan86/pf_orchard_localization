from PyQt5.QtCore import QTimer

class PlaybackMode:
    def __init__(self, main_app_manager):

        self.main_app_manager = main_app_manager

        self.main_app_manager.image_browsing_controls.playButtonClicked.connect(self.control_image_playback)
        self.main_app_manager.image_browsing_controls.previous_button.clicked.connect(self.prev_button_clicked)
        self.main_app_manager.image_browsing_controls.next_button.clicked.connect(self.next_button_clicked)

        self.playing = False
        self.mode_active = False
        self.is_processing = False
        self.timer = None


    def control_image_playback(self, command):
        if not self.check_for_data():
            return

        if command == "Stop":
            self.main_app_manager.image_browsing_controls.play_fwd_button.setText("Play")
            self.playing = False
            self.timer.stop()
        else:
            self.main_app_manager.image_browsing_controls.play_fwd_button.setText("Stop")
            self.playing = True
            self.timer = QTimer()
            self.timer.timeout.connect(self.next_button_clicked)
            self.timer.start(self.main_app_manager.image_delay_slider.get_delay_ms())
    def prev_button_clicked(self):
        if not self.check_for_data():
            return

        self.current_msg = self.main_app_manager.data_manager.get_prev_img_msg()

        self.image_change_update()

    def next_button_clicked(self):
        if not self.check_for_data():
            return

        if not self.is_processing:
            self.is_processing = True
            self.current_msg = self.main_app_manager.data_manager.get_next_img_msg()
            self.image_change_update()
            self.is_processing = False

    def image_change_update(self):
        if self.current_msg is None:
            self.main_app_manager.print_message("Reached the beginning/end of the data file")
            if self.playing:
                self.control_image_playback("Stop")
        else:
            self.main_app_manager.trunk_data_connection.get_trunk_data(self.current_msg)

            self.main_app_manager.data_file_time_line.set_time_line(self.main_app_manager.data_manager.current_data_file_time_stamp)

        self.main_app_manager.image_number_label.set_img_number_label(self.main_app_manager.data_manager.current_img_position,
                                                          self.main_app_manager.data_manager.num_img_msgs)

    def check_for_data(self):
        if self.main_app_manager.data_manager is None:
            self.main_app_manager.print_message("No data loaded")
            return False
        return True

    def activate_mode(self):
        self.mode_active = True
        self.main_app_manager.image_display.enable()
        self.main_app_manager.image_number_label.enable()
        self.main_app_manager.data_file_time_line.enable()
        self.main_app_manager.data_file_controls.enable()
        self.main_app_manager.image_browsing_controls.enable()

        self.main_app_manager.reset_app()

    def deactivate_mode(self):
        self.mode_active = False