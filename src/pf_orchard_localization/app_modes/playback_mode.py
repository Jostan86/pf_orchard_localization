from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QHBoxLayout
import os
import cv2

class PlaybackMode:
    def __init__(self, main_app_manager):

        self.main_app_manager = main_app_manager

        self.main_app_manager.image_browsing_controls.playButtonClicked.connect(self.control_image_playback)
        self.main_app_manager.image_browsing_controls.previous_button.clicked.connect(self.prev_button_clicked)
        self.main_app_manager.image_browsing_controls.next_button.clicked.connect(self.next_button_clicked)
        self.main_app_manager.image_browsing_controls.save_button.clicked.connect(self.save_button_clicked)

        self.playing = False
        self.mode_active = False
        self.is_processing = False
        self.timer = None
        self.mode_name = "Playback Images"


    def control_image_playback(self, command):
        if not self.check_for_data():
            return

        if command == "Play":
            self.main_app_manager.image_browsing_controls.play_fwd_button.setText("Stop")
            self.playing = True
            self.timer = QTimer()
            self.timer.timeout.connect(self.next_button_clicked)
            self.timer.start(self.main_app_manager.image_delay_slider.get_delay_ms())
        else:
            self.main_app_manager.image_browsing_controls.play_fwd_button.setText("Play")
            self.playing = False
            self.timer.stop()

    def prev_button_clicked(self):
        if not self.check_for_data():
            return

        self.main_app_manager.current_msg = self.main_app_manager.data_manager.get_prev_img_msg()

        self.image_change_update()

    def next_button_clicked(self):
        if not self.check_for_data():
            if self.playing:
                self.control_image_playback("Stop")
            return

        if not self.is_processing:
            self.is_processing = True
            self.main_app_manager.current_msg = self.main_app_manager.data_manager.get_next_img_msg()
            self.image_change_update()
            self.is_processing = False

    def image_change_update(self):
        if self.main_app_manager.current_msg is None:
            self.main_app_manager.print_message("Reached the beginning/end of the data file")
            if self.playing:
                self.control_image_playback("Stop")
        else:
            self.main_app_manager.trunk_data_connection.get_trunk_data(self.main_app_manager.current_msg)

            self.main_app_manager.data_file_time_line.set_time_line(self.main_app_manager.data_manager.current_data_file_time_stamp)

        self.main_app_manager.image_number_label.set_img_number_label(self.main_app_manager.data_manager.current_img_position,
                                                          self.main_app_manager.data_manager.num_img_msgs)

    def save_button_clicked(self):
        if self.main_app_manager.current_msg is None:
            self.main_app_manager.print_message("No image to save")
            return

        if 'rgb_image' not in self.main_app_manager.current_msg:
            self.main_app_manager.print_message("No image to save")
            return

        save_dir = self.main_app_manager.image_browsing_controls.save_location
        if not os.path.exists(save_dir):
            self.main_app_manager.print_message("Invalid save location")
            return

        image_name = str(self.main_app_manager.current_msg['timestamp']) + ".png"
        image_path = os.path.join(save_dir, image_name)
        cv2.imwrite(image_path, self.main_app_manager.current_msg['rgb_image'])


        #
    def check_for_data(self):
        if self.main_app_manager.data_manager is None:
            self.main_app_manager.print_message("No data loaded")
            return False
        return True

    def activate_mode(self):
        self.mode_active = True
        
        self.main_app_manager.data_file_time_line.show()
        self.main_app_manager.image_delay_slider.show()
        self.main_app_manager.mode_selector.show()
        self.main_app_manager.checkboxes.show()
        self.main_app_manager.image_display.show()
        self.main_app_manager.image_number_label.show()
        self.main_app_manager.image_browsing_controls.show()
        self.main_app_manager.data_file_controls.show()
        self.main_app_manager.console.show()

        self.next_button_clicked()


    def deactivate_mode(self):
        self.mode_active = False

    def shutdown_hook(self):
        pass