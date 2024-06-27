from PyQt6.QtCore import QTimer, QThread, QObject, pyqtSlot, pyqtSignal
from PyQt6.QtWidgets import QHBoxLayout
import os
import cv2

class PlaybackThread(QThread):
    
    signal_print_message = pyqtSignal(str)
    signal_set_time_line = pyqtSignal(float)
    signal_set_img_number_label = pyqtSignal(int, int)
    
    def __init__(self, data_manager, get_trunk_data_func, only_single_image, forward, added_delay):
        
        
        
        self.data_manager = data_manager
        self.get_trunk_data_func = get_trunk_data_func
        self.only_single_image = only_single_image
        self.forward = forward
        self.added_delay = added_delay
        
        self.playing = False
        
        super().__init__()
    
    def print_message(self, message):
        self.signal_print_message.emit(message)
    
    def set_time_line(self, time_stamp):
        self.signal_set_time_line.emit(time_stamp)
    
    def set_img_number_label(self, current_img_position, num_img_msgs):
        self.signal_set_img_number_label.emit(current_img_position, num_img_msgs)
        
    def run(self):
        if self.only_single_image:
            if self.forward:
                self.next_image()
            else:
                self.previous_image()
        else:
            self.play_forward()
        
    def play_forward(self):
        self.playing = True
        
        while self.playing:
            self.sleep(self.added_delay)
            
            self.next_image()
            
    
    def previous_image(self):
        current_msg = self.data_manager.get_prev_img_msg()
        self.image_change_update(current_msg)
    
    def next_image(self):
        current_msg = self.data_manager.get_next_img_msg()
        self.image_change_update(current_msg)

    def image_change_update(self, current_msg):
        if current_msg is None:
            self.print_message("Reached the beginning/end of the data file")
            self.playing = False
            return 
        
        self.get_trunk_data_func(current_msg)

        self.set_time_line(self.data_manager.current_data_file_time_stamp)
        self.set_img_number_label(self.data_manager.current_img_position, self.data_manager.num_img_msgs)
    
    @pyqtSlot()
    def stop_playback(self):
        self.playing = False
    
class PlaybackMode(QObject):
    signal_stop_thread = pyqtSignal()
    
    def __init__(self, main_app_manager):

        self.main_app_manager = main_app_manager

        self.mode_active = False
        
        self.playback_thread = None
        
        self.thread_deleted = True
        
        self.mode_name = "Playback Images"
        
        super().__init__()

    def start_playback_thread(self, only_single_image, forward, added_delay):
        
        if self.main_app_manager.data_file_controls.data_manager is None:
            raise ValueError("No data has been loaded.")
        
        if not only_single_image:
            self.enable_disable_widgets(enable=False)
            
        self.thread_deleted = False        
        
        self.playback_thread = PlaybackThread(data_manager=self.main_app_manager.data_file_controls.data_manager,
                                              get_trunk_data_func=self.main_app_manager.trunk_data_connection.get_trunk_data,
                                              only_single_image=only_single_image,
                                              forward=forward,
                                              added_delay=added_delay)
        
        self.playback_thread.signal_print_message.connect(self.main_app_manager.print_message)
        self.playback_thread.signal_set_time_line.connect(self.main_app_manager.data_file_controls.set_time_line)
        self.playback_thread.signal_set_img_number_label.connect(self.main_app_manager.image_number_label.set_img_number_label)
        self.signal_stop_thread.connect(self.playback_thread.stop_playback)
        
        self.playback_thread.destroyed.connect(self.thread_deleted_slot)
        self.playback_thread.finished.connect(self.thread_clean_up)
        
        self.playback_thread.start()
    
    @pyqtSlot()
    def thread_clean_up(self):
        
        self.playback_thread.wait()
        
        self.playback_thread.deleteLater()
        
        self.enable_disable_widgets(enable=True)
    
    @pyqtSlot()
    def thread_deleted_slot(self):
        self.thread_deleted = True
    
    @pyqtSlot()
    def play_button_clicked(self):
        if not self.thread_deleted:
            return
        
        self.start_playback_thread(only_single_image=False, forward=True, added_delay=0)
    
    @pyqtSlot()
    def stop_thread(self):
        self.signal_stop_thread.emit()
        
    @pyqtSlot()
    def prev_button_clicked(self):
        if not self.thread_deleted:
            return
        
        self.start_playback_thread(only_single_image=True, forward=False, added_delay=0)

    @pyqtSlot()
    def next_button_clicked(self):
        if not self.thread_deleted:
            return
        
        self.start_playback_thread(only_single_image=True, forward=True, added_delay=0)

    @pyqtSlot()
    def save_button_clicked(self):
        if self.main_app_manager.data_file_controls.data_manager is None:
            self.main_app_manager.print_message("No data loaded")
            return
        
        current_msg = self.main_app_manager.data_file_controls.data_manager.current_msg

        if 'rgb_image' not in current_msg:
            self.main_app_manager.print_message("No image to save")
            return

        save_dir = self.main_app_manager.image_browsing_controls.save_location
        if not os.path.exists(save_dir):
            
            self.main_app_manager.print_message("Invalid save location")
            return

        image_name = str(current_msg['timestamp']) + ".png"
        image_path = os.path.join(save_dir, image_name)
        cv2.imwrite(image_path, current_msg['rgb_image'])


    def enable_disable_widgets(self, enable):
        
        self.main_app_manager.data_file_controls.setEnabled(enable)
        self.main_app_manager.mode_selector.setEnabled(enable)
        self.main_app_manager.image_delay_slider.setEnabled(enable)
        self.main_app_manager.checkboxes.setEnabled(enable)
        
        self.main_app_manager.image_browsing_controls.set_playing(not enable)
    
    def activate_mode(self):
        
        self.mode_active = True
        
        self.main_app_manager.image_delay_slider.show()
        self.main_app_manager.mode_selector.show()
        self.main_app_manager.checkboxes.show()
        self.main_app_manager.image_display.show()
        self.main_app_manager.image_number_label.show()
        self.main_app_manager.image_browsing_controls.show()
        self.main_app_manager.data_file_controls.show()
        self.main_app_manager.console.show()
        
        self.connect_gui()

        self.next_button_clicked()

    def connect_gui(self):
        self.main_app_manager.image_browsing_controls.playButtonClicked.connect(self.play_button_clicked)
        self.main_app_manager.image_browsing_controls.stopButtonClicked.connect(self.stop_thread)
        self.main_app_manager.image_browsing_controls.previous_button.clicked.connect(self.prev_button_clicked)
        self.main_app_manager.image_browsing_controls.next_button.clicked.connect(self.next_button_clicked)
        self.main_app_manager.image_browsing_controls.save_button.clicked.connect(self.save_button_clicked)
    
    def disconnect_gui(self):
        self.main_app_manager.image_browsing_controls.playButtonClicked.disconnect(self.play_button_clicked)
        self.main_app_manager.image_browsing_controls.stopButtonClicked.disconnect(self.stop_thread)
        self.main_app_manager.image_browsing_controls.previous_button.clicked.disconnect(self.prev_button_clicked)
        self.main_app_manager.image_browsing_controls.next_button.clicked.disconnect(self.next_button_clicked)
        self.main_app_manager.image_browsing_controls.save_button.clicked.disconnect(self.save_button_clicked)

    def deactivate_mode(self):
        self.disconnect_gui()
        self.mode_active = False

    def shutdown_hook(self):
        if not self.thread_deleted:
            self.stop_thread()