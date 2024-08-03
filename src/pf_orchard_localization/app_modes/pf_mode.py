from PyQt5.QtCore import pyqtSignal, pyqtSlot, QObject
from ..pf_threads import PfBagThread, PfCachedThread, PfTestExecutorQt
        
    
class PfRecordedDataMode(QObject):
    """
    This class handles the Recorded Data Mode of the application. The Recorded Data Mode is used to run the particle filter on ros2 bag data.
    """
    
    stop_pf_signal = pyqtSignal()
    
    def __init__(self, main_app_manager):
        """
        Initialize the mode
        """        
        self.main_app_manager = main_app_manager

        self.mode_active = False

        self.mode_name = "PF - Recorded Data"
        
        self.pf_thread = None
        
        self.thread_deleted = True
        
        super().__init__()

    def ensure_pf_stopped(self):
        """
        Ensures that the particle filter thread is stopped
        """
        if self.thread_deleted:
            return 
        
        self.stop_button_clicked()

    def enable_disable_widgets(self, enable):
        """
        Enables/disables the widgets in the GUI when the PF thread is running
        """
        self.main_app_manager.mode_selector.setEnabled(enable)
        self.main_app_manager.change_parameters_button.setEnabled(enable)
        
        self.main_app_manager.data_file_controls.data_file_time_line.setReadOnly(not enable)
        self.main_app_manager.image_delay_slider.setEnabled(enable)
        self.main_app_manager.checkboxes.setEnabled(enable)
        self.main_app_manager.start_location_controls.setReadOnly(not enable)
        self.main_app_manager.control_buttons.reset_button.setEnabled(enable)
        self.main_app_manager.control_buttons.single_step_button.setEnabled(enable)
        self.main_app_manager.data_file_controls.setEnabled(enable)
        
        if enable:
            self.main_app_manager.control_buttons.set_start()
        else:
            self.main_app_manager.control_buttons.set_stop() 
        
        self.enable_disable_widgets_unique(enable)
    
    def enable_disable_widgets_unique(self, enable):
        """
        Enables or disables the widgets that are unique to this mode when the PF thread is running, subclasses should override this method
        """
        self.main_app_manager.cached_data_creator.enable_checkbox.setEnabled(enable)
        
    def start_pf(self, single_image):     
        """
        Starts the particle filter thread

        Args:
            single_image: bool: True to run one image then stop, False to run continuously
        """        
        self.thread_deleted = False
        

        if not single_image:   
            self.enable_disable_widgets(enable=False)
             

        self.pf_thread = PfBagThread(pf_engine=self.main_app_manager.pf_engine,
                                     data_manager=self.main_app_manager.data_file_controls.data_manager,
                                     trunk_data_thread=self.main_app_manager.trunk_data_connection,
                                     stop_when_converged=self.main_app_manager.parameters_pf.stop_when_converged,
                                     only_single_image=single_image,
                                     added_delay=self.main_app_manager.image_delay_slider.get_delay_ms()/1000,
                                     image_fps=self.main_app_manager.parameters_data.image_fps,
                                     use_visual_odom=self.main_app_manager.parameters_data.use_visual_odom,
                                     cache_data_enabled=self.main_app_manager.cached_data_creator.cache_data_enabled)
        
        self.pf_thread.load_next_data_file.connect(self.main_app_manager.data_file_controls.load_next_data_file)
        self.pf_thread.pf_run_message.connect(self.main_app_manager.print_message)
        self.pf_thread.set_time_line.connect(self.main_app_manager.data_file_controls.set_time_line)
        self.pf_thread.cache_msg.connect(self.main_app_manager.cached_data_creator.cache_data)
        self.pf_thread.set_img_number_label.connect(self.main_app_manager.image_number_label.set_img_number_label)  
        self.pf_thread.plot_best_guess.connect(self.main_app_manager.plotter.update_actual_position)
        self.pf_thread.plot_particles.connect(self.main_app_manager.plotter.update_particles)
        
        
        self.main_app_manager.data_file_controls.data_file_loaded.connect(self.pf_thread.data_manager_receiver)  
        
        self.stop_pf_signal.connect(self.pf_thread.stop_pf)
        
        self.pf_thread.destroyed.connect(self.thread_deleted_slot)
        self.pf_thread.finished.connect(self.thread_clean_up)
        
        self.pf_thread.start()        
        
    def thread_clean_up(self):
        """
        Cleans up the particle filter thread after it has finished
        """
        
        self.pf_thread.wait()
        
        self.pf_thread.deleteLater()
        
        self.enable_disable_widgets(enable=True)

    def activate_mode(self):
        """
        Activates the recorded data mode
        """
        self.mode_active = True

        self.setup_gui()
        self.connect_gui()

        self.main_app_manager.reset_pf()

    def setup_gui(self):
        """
        Sets up the GUI for the mode
        """

        self.main_app_manager.mode_selector.show()
        self.main_app_manager.change_parameters_button.show()
        
        self.main_app_manager.image_delay_slider.show()
        
        self.main_app_manager.checkboxes.show()
        self.main_app_manager.start_location_controls.show()
        self.main_app_manager.control_buttons.show()
        self.main_app_manager.image_display.show()
        self.main_app_manager.image_number_label.show()
        self.main_app_manager.data_file_controls.show()
        self.main_app_manager.cached_data_creator.show()
        self.main_app_manager.console.show()
        
        self.main_app_manager.plotter.show()
    
    @pyqtSlot()
    def thread_deleted_slot(self):
        """
        Slot to handle when the thread is deleted, connected to the destroyed signal of the particle filter thread
        """
        self.thread_deleted = True
        
    @pyqtSlot()
    def start_button_clicked(self):
        """
        Slot to handle the start button clicked signal, starts the particle filter thread
        """
        if not self.thread_deleted:
            return
        
        self.start_pf(False)
    
    @pyqtSlot()
    def continue_button_clicked(self):
        """
        Slot to handle the continue button clicked signal, sends one image through the particle filter 
        """
        if not self.thread_deleted:
            return
        
        self.start_pf(True)
    
    @pyqtSlot()
    def stop_button_clicked(self):
        """
        Slot to handle the stop button clicked signal, stops the particle filter thread if it's running
        """
        if self.thread_deleted:
            return
        
        self.stop_pf_signal.emit()
        
    def connect_gui(self):
        """
        Connects the GUI signals to the slots for the recorded data mode
        """
        self.main_app_manager.control_buttons.startButtonClicked.connect(self.start_button_clicked)
        self.main_app_manager.control_buttons.stopButtonClicked.connect(self.stop_button_clicked)
        self.main_app_manager.control_buttons.single_step_button.clicked.connect(self.continue_button_clicked)
        
    
    def disconnect_gui(self):
        """
        Disconnects the GUI signals from the slots for the recorded data mode
        """
        self.main_app_manager.control_buttons.startButtonClicked.disconnect(self.start_button_clicked)
        self.main_app_manager.control_buttons.stopButtonClicked.disconnect(self.stop_button_clicked)
        self.main_app_manager.control_buttons.single_step_button.clicked.disconnect(self.continue_button_clicked)
        self.main_app_manager.cached_data_creator.enable_checkbox.setChecked(False)
    
    def deactivate_mode(self):
        """
        Deactivates the recorded data mode
        """
        if not self.mode_active:
            return

        self.disconnect_gui()

        self.mode_active = False

        self.ensure_pf_stopped()
   

    def shutdown_hook(self):
        """
        Hook to run when the application is shutting down, ensures the particle filter thread is stopped
        """
        self.ensure_pf_stopped()

class PfModeCached(PfRecordedDataMode):
    """
    This class handles the Cached Data Mode of the application. The Cached Data Mode is used to run the particle filter on data where the results of the odometry and image
    processing have been cached and loaded to avoid processing the same data repeatedly when testing the particle filter.
    """
    def __init__(self, main_app_manager):
        """
        Initialize the mode, extends the parent class method to set the mode name 
        """
        super().__init__(main_app_manager)
        self.mode_name = "PF - Cached Data"

    def enable_disable_widgets_unique(self, enable):
        """
        Enables or disables the widgets that are unique to this mode when the PF thread is running. Overrides the method in the parent class. There are no unique widgets for this mode.
        """
        pass

    def start_pf(self, single_image):     
        """
        Starts the particle filter thread. Overrides the method in the parent class.

        Args:
            single_image: bool: True to run one image then stop, False to run continuously
        """
        
        self.thread_deleted = False
        
        if not single_image:   
            self.enable_disable_widgets(enable=False) 

        self.pf_thread = PfCachedThread(pf_engine=self.main_app_manager.pf_engine,
                                     data_manager=self.main_app_manager.data_file_controls.data_manager,
                                     trunk_data_thread=self.main_app_manager.trunk_data_connection,
                                     stop_when_converged=self.main_app_manager.parameters_pf.stop_when_converged,
                                     only_single_image=single_image,
                                     added_delay=self.main_app_manager.image_delay_slider.get_delay_ms()/1000,
                                     cache_data_enabled=False,
                                     use_visual_odom=self.main_app_manager.parameters_data.use_visual_odom,
                                     image_fps=self.main_app_manager.parameters_data.image_fps)
        
        # self.pf_thread.load_next_data_file.connect(self.main_app_manager.data_file_controls.load_next_data_file)
        self.pf_thread.pf_run_message.connect(self.main_app_manager.print_message)
        self.pf_thread.set_time_line.connect(self.main_app_manager.data_file_controls.set_time_line)
        # self.pf_thread.cache_msg.connect(self.main_app_manager.cached_data_creator.cache_data)
        self.pf_thread.set_img_number_label.connect(self.main_app_manager.image_number_label.set_img_number_label)  
        self.pf_thread.plot_best_guess.connect(self.main_app_manager.plotter.update_actual_position)
        self.pf_thread.plot_particles.connect(self.main_app_manager.plotter.update_particles)
        
        
        self.stop_pf_signal.connect(self.pf_thread.stop_pf)
        
        # self.main_app_manager.data_file_controls.data_file_loaded.connect(self.pf_thread.data_manager_receiver)   
            
        self.pf_thread.destroyed.connect(self.thread_deleted_slot)

        self.pf_thread.finished.connect(self.thread_clean_up)
        
        self.pf_thread.start()        
                
    def setup_gui(self):
        """
        Sets up the GUI for the cached data mode. Overrides the method in the parent class.
        """        
        self.main_app_manager.mode_selector.show()
        self.main_app_manager.change_parameters_button.show()
        self.main_app_manager.image_delay_slider.show()
        self.main_app_manager.checkboxes.show()
        self.main_app_manager.start_location_controls.show()
        self.main_app_manager.control_buttons.show()
        self.main_app_manager.image_display.show()
        self.main_app_manager.image_number_label.show()
        self.main_app_manager.data_file_controls.show()
        self.main_app_manager.console.show()
        self.main_app_manager.plotter.show()

class PfModeCachedTests(PfModeCached):
    """
    This class handles the Cached Data Tests Mode of the application. The Cached Data Tests Mode is used to run the particle filter repeatedly on cached data and test the results.
    """
    
    def __init__(self, main_app_manager):
        super().__init__(main_app_manager)
        """
        Initialize the mode, extends the parent class method to set the mode name 
        """
        self.mode_name = "PF - Cached Data Tests"
        
        self.running_all_tests = False
    
    def start_pf(self, load_data_only=False):
        """
        Starts the particle filter thread. Overrides the method in the parent class.

        Args:
            load_data_only: bool: True to just load the data, False to run the particle filter
        """
        
        self.thread_deleted = False
        
        test_index = None
        
        if not self.run_all_tests_flag:
            test_index = self.main_app_manager.pf_test_controls.get_selected_test()
            
        if not load_data_only:
            self.enable_disable_widgets(enable=False)
  
        self.pf_thread = PfTestExecutorQt(pf_engine=self.main_app_manager.pf_engine,
                                          parameters_pf=self.main_app_manager.parameters_pf,
                                          test_info_path=self.main_app_manager.parameters_data.test_start_info_path,
                                          cached_data_files_dir=self.main_app_manager.parameters_data.data_file_dir,
                                          num_trials=self.main_app_manager.pf_test_controls.get_num_trials_per_location(),
                                          get_trunk_data_func=self.main_app_manager.trunk_data_connection.get_trunk_data,
                                          save_path=self.main_app_manager.pf_test_controls.get_save_path(),
                                          convergence_threshold=0.5,
                                          test_index=test_index,
                                          load_data_only=load_data_only,)
        
        self.pf_thread.reset_pf_app.connect(self.main_app_manager.reset_pf)
        self.pf_thread.update_test_number.connect(self.main_app_manager.pf_test_controls.update_test_number)
        self.pf_thread.update_trial_number.connect(self.main_app_manager.pf_test_controls.update_trial_number)
        self.pf_thread.set_time_line.connect(self.main_app_manager.data_file_controls.set_time_line)
        self.pf_thread.plot_gt_position.connect(self.main_app_manager.plotter.update_actual_position)
        self.pf_thread.plot_particles.connect(self.main_app_manager.plotter.update_particles)
        self.pf_thread.update_image_number.connect(self.main_app_manager.image_number_label.set_img_number_label)
        self.pf_thread.update_trial_info.connect(self.main_app_manager.pf_test_controls.update_trial_info)
        self.pf_thread.update_ui_with_trial_results.connect(self.main_app_manager.pf_test_controls.update_trial_results)
        self.pf_thread.print_message.connect(self.main_app_manager.print_message)
        
        self.stop_pf_signal.connect(self.pf_thread.stop_pf)
        
        self.pf_thread.destroyed.connect(self.thread_deleted_slot)
        self.pf_thread.finished.connect(self.thread_clean_up)
        
        self.pf_thread.start()
        
    @pyqtSlot() 
    def run_all_tests(self):
        """
        Slot to handle the run all tests signal, starts the particle filter thread to run all the tests
        """
        if not self.thread_deleted:
            return
        
        self.run_all_tests_flag = True
        
        self.start_pf()
    
    @pyqtSlot()
    def run_selected_test(self):
        """
        Slot to handle the run selected test signal, starts the particle filter thread to run the selected test
        """
        if not self.thread_deleted:
            return
        
        self.run_all_tests_flag = False
        
        self.start_pf()
    
    @pyqtSlot()
    def load_data_only(self):
        """
        Slot to handle the load data only signal, starts the particle filter thread but just loads the test data
        """
        if not self.thread_deleted:
            return
        
        self.run_all_tests_flag = False
        
        self.start_pf(load_data_only=True)
    
    def enable_disable_widgets(self, enable):
        """
        Enables/disables the widgets in the GUI when the PF thread is running, overrides the method in PfRecordedDataMode.
        """
        self.main_app_manager.mode_selector.setEnabled(enable)
        self.main_app_manager.change_parameters_button.setEnabled(enable)
        
        self.main_app_manager.checkboxes.setEnabled(enable)
        
        if self.run_all_tests_flag:
            self.main_app_manager.pf_test_controls.set_running_all_tests(not enable)
        else:
            self.main_app_manager.pf_test_controls.set_running_selected_test(not enable)
        

    def setup_gui(self):
        """
        Sets up the GUI for the cached data tests mode. Overrides the method in the parent class.
        """
        
        self.main_app_manager.mode_selector.show()
        self.main_app_manager.change_parameters_button.show()
        
        self.main_app_manager.checkboxes.show()
        self.main_app_manager.start_location_controls.show()
        
        self.main_app_manager.image_display.show()
        self.main_app_manager.image_number_label.show()
        self.main_app_manager.pf_test_controls.show()
        self.main_app_manager.data_file_controls.show()
        self.main_app_manager.console.show()
        self.main_app_manager.plotter.show()


    def connect_gui(self):
        """
        Connects the GUI signals to the slots for the cached data tests mode, overrides the method in PfRecordedDataMode.
        """

        self.main_app_manager.pf_test_controls.runAllTestsClicked.connect(self.run_all_tests)
        self.main_app_manager.pf_test_controls.abortAllTestsClicked.connect(self.stop_button_clicked)
        self.main_app_manager.pf_test_controls.runSelectedTestClicked.connect(self.run_selected_test)
        self.main_app_manager.pf_test_controls.abortSelectedTestClicked.connect(self.stop_button_clicked)
        self.main_app_manager.pf_test_controls.test_selection_combobox.currentIndexChanged.connect(self.load_data_only)
        
        self.main_app_manager.start_location_controls.setReadOnly(True)
        self.main_app_manager.data_file_controls.setEnabled(False)

    def activate_mode(self):
        """
        Activates the cached data tests mode, extends the method in PfRecordedDataMode.
        """

        super().activate_mode()
        
        self.main_app_manager.pf_test_controls.load_pf_test_names()
        
        self.load_data_only()

    def deactivate_mode(self):
        """
        Deactivates the cached data tests mode, overrides the method in PfRecordedDataMode.
        """
        self.disconnect_gui()
        """
        Deactivates the cached data tests mode, overrides the method in PfRecordedDataMode.
        """
        self.mode_active = False
        self.stop_button_clicked()

    def disconnect_gui(self):
        """
        Disconnects the GUI signals from the slots for the cached data tests mode, overrides the method in PfRecordedDataMode.
        """
        # disconnect signals
        self.main_app_manager.pf_test_controls.runAllTestsClicked.disconnect(self.run_all_tests)
        self.main_app_manager.pf_test_controls.abortAllTestsClicked.disconnect(self.stop_button_clicked)
        self.main_app_manager.pf_test_controls.runSelectedTestClicked.disconnect(self.run_selected_test)
        self.main_app_manager.pf_test_controls.abortSelectedTestClicked.disconnect(self.stop_button_clicked)
        self.main_app_manager.pf_test_controls.test_selection_combobox.currentIndexChanged.disconnect(self.load_data_only)
        
        self.main_app_manager.start_location_controls.setReadOnly(False)
        self.main_app_manager.data_file_controls.setEnabled(True)


class PfModeSaveCalibrationData(PfRecordedDataMode):
    """
    This class handles the Save Calibration Data Mode of the application. The Save Calibration Data Mode is used to save the calibration data to a file.
    """
    
    signal_save_data = pyqtSignal(dict)
    
    def __init__(self, main_app_manager):
        """
        Initialize the mode, extends the parent class method to set the mode name
        """

        super().__init__(main_app_manager)
        self.mode_name = "PF - Save Calibration Data"

    def setup_gui(self):
        """
        Extends the method in the parent class to set up the GUI for the save calibration data mode
        """
        
        super().setup_gui()
        
        self.main_app_manager.save_calibration_data_controls.show()
        
        self.main_app_manager.cached_data_creator.hide()
        
    def enable_disable_widgets_unique(self, enable):
        """
        Enables or disables the widgets that are unique to this mode when the PF thread is running, overrides the method in the parent class.
        """

        self.main_app_manager.save_calibration_data_controls.set_running(not enable)
        
    def connect_gui(self):
        """
        Extends the method in the parent class to connect the GUI signals to the slots for the save calibration data mode
        """
        # self.main_app_manager.control_buttons.startButtonClicked.connect(self.start_button_clicked)
        # self.main_app_manager.control_buttons.stopButtonClicked.connect(self.stop_button_clicked)
        # self.main_app_manager.control_buttons.single_step_button.clicked.connect(self.continue_button_clicked)
        self.main_app_manager.trunk_data_connection.signal_save_calibration_data.connect(self.main_app_manager.save_calibration_data_controls.save_data)
        super().connect_gui()
    
    def disconnect_gui(self):
        """
        Extends the method in the parent class to disconnect the GUI signals from the slots for the save calibration data mode
        """
    #     self.main_app_manager.control_buttons.startButtonClicked.disconnect(self.start_button_clicked)
    #     self.main_app_manager.control_buttons.stopButtonClicked.disconnect(self.stop_button_clicked)
    #     self.main_app_manager.control_buttons.single_step_button.clicked.disconnect(self.continue_button_clicked)
        self.main_app_manager.trunk_data_connection.signal_save_calibration_data.disconnect(self.main_app_manager.save_calibration_data_controls.save_data)
        super().disconnect_gui()
