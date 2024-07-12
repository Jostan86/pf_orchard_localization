from PyQt5.QtCore import QTimer, QObject, pyqtSignal, pyqtSlot
from PyQt5.QtWidgets import QHBoxLayout, QVBoxLayout, QApplication
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np
# from ..app_managers import PfAppLive
from ..pf_threads import PfLiveThread
# import rclpy

class PfLiveMode(QObject):
    
    stop_pf_signal = pyqtSignal()
    
    def __init__(self, main_app_manager):
        super().__init__()
        
        self.main_app_manager = main_app_manager

        self.mode_active = False

        self.mode_name = "PF - Live Data"

        self.pf_thread = None
        
        self.thread_deleted = True

    
    def ensure_pf_stopped(self):
        if self.thread_deleted:
            return 
        
        self.stop_button_clicked()

    def start_pf(self):     
        
        self.thread_deleted = False
        
        self.enable_disable_widgets(enable=False)
        
        
             

        self.pf_thread = PfLiveThread(pf_engine=self.main_app_manager.pf_engine,
                                     trunk_data_thread=self.main_app_manager.trunk_data_connection)
        
        self.pf_thread.pf_run_message.connect(self.main_app_manager.print_message)
        self.pf_thread.set_queue_size.connect(self.main_app_manager.queue_size_label.set_queue_size)        
        self.pf_thread.plot_best_guess.connect(self.main_app_manager.plotter.update_actual_position)
        self.pf_thread.plot_particles.connect(self.main_app_manager.plotter.update_particles)
        self.pf_thread.signal_segmented_image.connect(self.main_app_manager.image_display.load_image)
        
        self.stop_pf_signal.connect(self.pf_thread.stop_pf)
        
        self.pf_thread.destroyed.connect(self.thread_deleted_slot)
        self.pf_thread.finished.connect(self.thread_clean_up)
        
        self.pf_thread.start() 
        
    def thread_clean_up(self):
        
        self.pf_thread.wait()
        
        self.pf_thread.deleteLater()
        
        self.enable_disable_widgets(enable=True)
    
    
    def enable_disable_widgets(self, enable):

        self.main_app_manager.mode_selector.setEnabled(enable)
        self.main_app_manager.change_parameters_button.setEnabled(enable)
        
        self.main_app_manager.checkboxes.setEnabled(enable)
        self.main_app_manager.start_location_controls.setReadOnly(not enable)
        self.main_app_manager.control_buttons.reset_button.setEnabled(enable)
        self.main_app_manager.control_buttons.single_step_button.setEnabled(enable)
        
        if enable:
            self.main_app_manager.control_buttons.set_start()
            self.main_app_manager.trunk_data_connection.trunk_data_signal.disconnect(self.trunk_data_displayer)
        else:
            self.main_app_manager.control_buttons.set_stop() 
            self.main_app_manager.trunk_data_connection.trunk_data_signal.connect(self.trunk_data_displayer)
    
    @pyqtSlot(object)
    def trunk_data_displayer(self, trunk_data):
        # rgb_image = trunk_data['rgb_image']
        seg_img = trunk_data['seg_img']
        
        # self.main_app_manager.image_display.load_image(rgb_image)
        self.main_app_manager.image_display.load_image(seg_img, 1)
            
    @pyqtSlot()
    def thread_deleted_slot(self):
        self.thread_deleted = True
        
    @pyqtSlot()
    def start_button_clicked(self):
        if not self.thread_deleted:
            return
        
        self.start_pf()
    
    @pyqtSlot()
    def stop_button_clicked(self):
        if self.thread_deleted:
            return
        
        self.stop_pf_signal.emit()
        
    def setup_gui(self):
        
        self.main_app_manager.mode_selector.show()
        self.main_app_manager.change_parameters_button.show()
        self.main_app_manager.checkboxes.show()
        self.main_app_manager.start_location_controls.show()
        self.main_app_manager.control_buttons.show()
        self.main_app_manager.control_buttons.single_step_button.hide()
        self.main_app_manager.queue_size_label.show()
        self.main_app_manager.image_display.show()
        self.main_app_manager.console.show()
        self.main_app_manager.plotter.show()

    def connect_gui(self):
        self.main_app_manager.control_buttons.startButtonClicked.connect(self.start_button_clicked)
        self.main_app_manager.control_buttons.stopButtonClicked.connect(self.stop_button_clicked)
        self.main_app_manager.trunk_data_connection.trunk_data_signal.connect(self.trunk_data_displayer)

        
    def disconnect_gui(self):
        self.main_app_manager.control_buttons.startButtonClicked.disconnect(self.start_button_clicked)
        self.main_app_manager.control_buttons.stopButtonClicked.disconnect(self.stop_button_clicked)
        self.main_app_manager.trunk_data_connection.trunk_data_signal.disconnect(self.trunk_data_displayer)

        
    def activate_mode(self):
        self.mode_active = True

        self.setup_gui()
        self.connect_gui()

        self.main_app_manager.reset_pf()
        
    def deactivate_mode(self):
        if not self.mode_active:
            return
        
        self.disconnect_gui()
        self.mode_active = False
        self.ensure_pf_stopped()
        
    def shutdown_hook(self):
        self.ensure_pf_stopped()