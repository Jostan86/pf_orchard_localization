#!/usr/bin/env python3
import sys
from dataclasses import fields
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtWidgets import QApplication, QVBoxLayout, QHBoxLayout, QWidget
from PyQt5.QtGui import QSurfaceFormat
from ..custom_widgets import (PfMainWindow, PfControlButtons, PfStartLocationControls, PfCheckBoxes,
                              Console, ImageDisplay, PfModeSelector, ImageBrowsingControls, PfPlotter,
                              DataFileControls, ImageNumberLabel, ImageDelaySlider, CachedDataCreator,
                              PfChangeParametersButton, PfTestControls, PfQueueSizeLabel, CalibrationDataControls)
from ..utils.parameters import ParametersPf, ParametersBagData, ParametersCachedData, ParametersLiveData
from ..pf_engine import PfEngine
from map_data_tools import MapData
import logging
from ..trunk_data_connection import TrunkDataConnection, TrunkDataConnectionCachedData, TrunkDataConnectionRosService, TrunkDataConnectionRosSub
from ..app_modes import PfRecordedDataMode, PfModeCached, PfModeCachedTests, PlaybackMode, PfLiveMode, PfModeSaveCalibrationData
import os
from functools import partial
import time
import copy


class PfAppBase(PfMainWindow):
    """
    Base class for the particle filter localization app, contains the main structure of the app. Inherited by the
    PfAppBags, PfAppCached, and PfAppLive classes which are the specific implementations of the app for the different
    use cases.
    """
    
    def __init__(self, config_file_path: str, logging_level=logging.DEBUG):
        """
        Initialize the app

        Args:
            config_file_path (str): Path to the configuration file
            logging_level: Logging level for the app 
        """
        super().__init__()

        # Initialize parameters
        self.init_data_parameters()
        self.parameters_data.load_from_yaml(config_file_path)
        self.parameters_pf = ParametersPf()
        self.parameters_pf.load_from_yaml(self.parameters_data.pf_config_file_path)

        # Load the map data
        self.map_data = MapData(map_data_path=self.parameters_data.map_data_path, move_origin=True, origin_offset=(5, 5))

        # Initialize the particle filter engine
        self.pf_engine = PfEngine(self.map_data)

        self.init_widgets()
        self.draw_ui()

        self.current_msg = None
        self.active_mode = None

        self.setup_trunk_data_connection()
        
        self.connect_slots()
        
        self.init_loaded_data()

        self.modes = []

        self.init_modes()

        self.mode_selector.set_modes(self.modes)

        self.mode_changed()

    def init_data_parameters(self):
        """
        Initialize the data parameters
        """
        raise NotImplementedError("init_data_parameters must be implemented in the subclass of PfAppBase")

    def init_loaded_data(self):
        """
        Initialize the loaded data
        """
        raise NotImplementedError("init_loaded_data must be implemented in the subclass of PfAppBase")
        
    def init_modes(self):
        """
        Initialize the different app modes
        """
        raise NotImplementedError("init_modes must be implemented in the subclass of PfAppBase")
    
    def draw_ui(self):
        """
        Draw the user interface
        """
        raise NotImplementedError("draw_ui must be implemented in the subclass of PfAppBase")


    def init_widgets(self):
        """
        Initialize all the widgets used in the app
        """
        # Set up the main window
        self.setWindowTitle("Orchard Particle Filter Localization App")
        self.init_window_display_settings()

        self.widget_list = []

        self.main_layout = QHBoxLayout()

        # Setup widgets shared by all the app types
        self.start_location_controls = PfStartLocationControls(self)
        self.control_buttons = PfControlButtons(self)
        self.change_parameters_button = PfChangeParametersButton(self)
        self.mode_selector = PfModeSelector()
        self.image_display = ImageDisplay(num_camera_feeds=1, scale_factor=1.0)
        self.console = Console()

        # Setup the checkboxes
        self.checkboxes = PfCheckBoxes()
        self.checkboxes.all_checkbox_info.append(
            ["include_width_checkbox", "Use Width in Weight Calculation", self.parameters_pf.include_width])
        self.checkboxes.all_checkbox_info.append(
            ["stop_when_converged_checkbox", "Stop When Converged", self.parameters_pf.stop_when_converged])
        self.checkboxes.all_checkbox_info.append(
            ["show_pre_filtered_segmentation_checkbox", "Show Pre-Filtered Segmentation", False])
        self.checkboxes.all_checkbox_info.append(
            ["show_rgb_image_checkbox", "Show Original Image", False])
        self.checkboxes.init_checkboxes()

        self.plotter = PfPlotter(self.map_data)

        # Set the central widget of the app
        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)
        central_widget.setLayout(self.main_layout)

        # Setup the widgets unique to the app types
        self.init_widgets_unique()

        # Add the widgets to the list of all widgets
        self.widget_list += [self.start_location_controls, self.checkboxes, self.change_parameters_button,
                            self.mode_selector, self.control_buttons, self.image_display, self.console, self.plotter,]

    def init_widgets_unique(self):
        """
        Initialize the widgets unique to the app type
        """
        raise NotImplementedError("init_widgets_unique must be implemented in the subclass of PfAppBase")

    def connect_slots(self):
        """
        Connect the UI elements to the app functions
        """

        self.control_buttons.reset_button.clicked.connect(lambda x: self.reset_pf(use_ui_parameters=True))

        self.plotter.plot_widget.clicked.connect(self.start_location_controls.set_start_location_from_plot_click)

        self.checkboxes.include_width_checkbox.stateChanged.connect(self.include_width_changed)
        self.checkboxes.show_rgb_image_checkbox.stateChanged.connect(self.image_display_checkbox_changed)
        self.checkboxes.show_pre_filtered_segmentation_checkbox.stateChanged.connect(self.image_display_checkbox_changed)
        self.checkboxes.stop_when_converged_checkbox.stateChanged.connect(self.stop_when_converged_changed)

        self.mode_selector.mode_selector.currentIndexChanged.connect(self.mode_changed)
        
        self.trunk_data_connection.signal_segmented_image.connect(self.image_display.load_image)
        self.trunk_data_connection.signal_unfiltered_image.connect(self.image_display.load_image)
        self.trunk_data_connection.signal_original_image.connect(self.image_display.load_image)
        self.trunk_data_connection.signal_print_message.connect(self.print_message)

        self.connect_slots_unique()

    def connect_slots_unique(self):
        """
        Connect the app functions unique to the app type
        """
        raise NotImplementedError("connect_slots_unique must be implemented in the subclass of PfAppBase")

    def setup_trunk_data_connection(self):
        """
        Setup the object that connects the app to the trunk segmenter and analyzer
        """
        raise NotImplementedError("setup_trunk_data_connection must be implemented in the subclass of PfAppBase")

    def image_display_checkbox_changed(self):
        """
        Change the camera feed display based on the checkboxes selected, called when one of the checkboxes that
        changes the image display is changed.
        """

        show_rgb_image = self.checkboxes.show_rgb_image_checkbox.isChecked()
        show_pre_filtered_segmentation = self.checkboxes.show_pre_filtered_segmentation_checkbox.isChecked()

        # Number of camera feeds needed
        num_camera_feeds = 1 + show_rgb_image + show_pre_filtered_segmentation

        # Check if the image display is already in the layout, if so, remove it. It returns -1 if it is not in the layout
        image_display_index = self.ui_layout.indexOf(self.image_display)
        if image_display_index != -1:
            self.ui_layout.removeWidget(self.image_display)

        # Delete and remove the image display widget and object and create a new one with the new number of camera feeds
        self.widget_list.remove(self.image_display)
        self.image_display.deleteLater()
        self.image_display = ImageDisplay(num_camera_feeds=num_camera_feeds, scale_factor=self.parameters_data.image_display_scale)
        self.widget_list.insert(0, self.image_display)

        # If the image display was in the layout, add it back in where it was
        if image_display_index != -1:
            self.ui_layout.insertWidget(image_display_index, self.image_display)

        # Tell the trunk_data_connection object where to display the images
        display_num = 0
        if show_rgb_image:
            self.trunk_data_connection.set_original_image_display_num(display_num)
            display_num += 1
        else:
            self.trunk_data_connection.set_original_image_display_num(-1)
        if show_pre_filtered_segmentation:
            self.trunk_data_connection.set_unfiltered_image_display_num(display_num)
            display_num += 1
        else:
            self.trunk_data_connection.set_unfiltered_image_display_num(-1)
        self.trunk_data_connection.set_segmented_image_display_num(display_num)

        # Update the image display with the current image if there is one
        if self.current_msg is not None:
            request = {"current_msg": self.current_msg, "for_display_only": True}
            self.trunk_data_connection.handle_request(request)

    def include_width_changed(self):
        """
        Change the include width parameter in the particle filter parameters according to the checkbox
        """
        self.parameters_pf.include_width = self.checkboxes.include_width_checkbox.isChecked()

    def stop_when_converged_changed(self):
        """
        Change the stop when converged parameter in the particle filter parameters according to the checkbox
        """
        self.parameters_pf.stop_when_converged = self.checkboxes.stop_when_converged_checkbox.isChecked()
    
    def hide_all_widgets(self):
        """
        Hide all the widgets in the app
        """
        for widget in self.widget_list:
            if widget is not None:
                widget.hide()           
                
    def mode_changed(self):
        """
        Change the active mode based on the mode selector
        """

        # Find the active mode and deactivate it
        for mode in self.modes:
            if mode.mode_active:
                mode.deactivate_mode()

        self.hide_all_widgets()

        # Find the new active mode and activate it
        for mode in self.modes:
            if mode.mode_name == self.mode_selector.mode:
                mode.activate_mode()
                self.active_mode = mode
                break

    @pyqtSlot(bool)
    def reset_pf(self, use_ui_parameters=True):
        """
        Reset the particle filter.

        Args:
            use_ui_parameters (bool): Whether to use the parameters set in the UI or the parameters set in the app
        """
        if use_ui_parameters:
            self.start_location_controls.get_parameters()
        else:
            self.start_location_controls.set_parameters()

        self.pf_engine.reset_pf(self.parameters_pf)

        self.reset_gui()

    def reset_gui(self):
        """
        Reset the particle filter elements in the GUI
        """

        self.control_buttons.set_num_particles(self.pf_engine.particles.shape[0])
        self.plotter.update_particles(self.pf_engine.downsample_particles())
        self.plotter.update_position_estimate(None)

    def get_pf_active(self):
        """
        Get whether the particle filter is currently active
        """

        if self.active_mode is not None:
            if hasattr(self.active_mode, "pf_continuous_active"):
                return self.active_mode.pf_continuous_active
        
        return False

    def get_pf_parameters(self):
        """
        Get the particle filter parameters
        """

        return self.parameters_pf

    def set_pf_parameters(self, parameters: ParametersPf):
        """
        Set the particle filter parameters

        Args:
            parameters (ParametersPf): Particle filter parameters
        """

        if self.get_pf_active():
            self.print_message("Cannot change parameters while the particle filter is running")
            return False
        self.parameters_pf = parameters
        return True

    @pyqtSlot(str)
    def print_message(self, message: str):
        """
        Print a message to the console

        Args:
            message (str): Message to print
        """

        if isinstance(message, list):
            for msg in message:
                self.console(msg)
        else:
            self.console(message)

    def display_pf_settings(self):
        """
        Display the current particle filter settings
        """

        for field in fields(self.parameters_pf):
            value = getattr(self.parameters_pf, field.name)
            self.print_message(field.name + ": " + str(value))

    def closeEvent(self, event):
        for mode in self.modes:
            if mode.mode_active:
                mode.shutdown_hook()

        event.accept()

class PfAppBags(PfAppBase):
    """
    Application class for the particle filter localization app using pre-recorded ros2 bag files.
    """
    def __init__(self, config_file_path, logging_level=logging.DEBUG):
        self.using_cached_data = False
        super().__init__(config_file_path, logging_level=logging_level)


    def init_data_parameters(self):
        """
        Initialize the data parameters
        """
        self.parameters_data = ParametersBagData()
    
    def init_loaded_data(self):
        """
        Initialize the loaded data
        """
        self.data_manager = None

        if self.parameters_data.initial_data_time is not None and self.data_manager is not None:
            self.data_file_time_line.set_time_line(float(self.parameters_data.initial_data_time))
            self.data_manager.set_time_stamp(float(self.parameters_data.initial_data_time))
        self.data_file_controls.open_data_file()
        self.data_file_controls.open_data_file()

    def setup_trunk_data_connection(self):
        """
        Setup the object that connects the app to the trunk segmenter and analyzer
        """
        self.trunk_data_connection = TrunkDataConnectionRosService()
        self.trunk_data_connection.start()
        self.image_display_checkbox_changed()
        
    def init_widgets_unique(self):
        """
        Initialize the widgets unique to bag data app
        """
        self.image_browsing_controls = ImageBrowsingControls()
        self.data_file_controls = DataFileControls(self.parameters_data)
        self.image_number_label = ImageNumberLabel()
        self.image_delay_slider = ImageDelaySlider()
        self.cached_data_creator = CachedDataCreator(self)
        self.save_calibration_data_controls = CalibrationDataControls(self)

        self.widget_list += [self.image_browsing_controls, self.data_file_controls,
                                self.image_number_label, self.image_delay_slider, self.cached_data_creator, self.save_calibration_data_controls]

    def draw_ui(self):
        """
        Draw the user interface
        """
        mode_change_button_layout = QHBoxLayout()
        mode_change_button_layout.addWidget(self.mode_selector)
        mode_change_button_layout.addWidget(self.change_parameters_button)

        control_layout = QHBoxLayout()
        control_layout.addWidget(self.control_buttons)
        control_layout.addWidget(self.image_delay_slider)

        self.ui_layout = QVBoxLayout()
        self.ui_layout.addLayout(mode_change_button_layout)
        self.ui_layout.addWidget(self.checkboxes)
        self.ui_layout.addWidget(self.start_location_controls)
        self.ui_layout.addLayout(control_layout)
        self.ui_layout.addWidget(self.image_display)
        self.ui_layout.addWidget(self.image_number_label)
        self.ui_layout.addWidget(self.image_browsing_controls)
        self.ui_layout.addWidget(self.data_file_controls)
        self.ui_layout.addWidget(self.cached_data_creator)
        self.ui_layout.addWidget(self.save_calibration_data_controls)
        self.ui_layout.addWidget(self.console)
        
        self.main_layout.addLayout(self.ui_layout)
        self.main_layout.addWidget(self.plotter)
        
    def connect_slots_unique(self):
        """
        Connect the app functions unique to the bag data app
        """
        self.data_file_controls.set_image_display.connect(self.trunk_data_connection.handle_request)

        self.data_file_controls.data_file_controls_message.connect(self.print_message)
        self.data_file_controls.reset_pf.connect(self.reset_pf)
        self.data_file_controls.set_img_number_label.connect(self.image_number_label.set_img_number_label)        

    def init_modes(self):
        """
        Initialize the different app modes for the bag data app
        """
        self.pf_mode = PfRecordedDataMode(self)
        self.modes.append(self.pf_mode)
        self.playback_mode = PlaybackMode(self)
        self.modes.append(self.playback_mode)
        self.get_calibration_data_mode = PfModeSaveCalibrationData(self)
        self.modes.append(self.get_calibration_data_mode)

class PfAppCached(PfAppBase):
    """
    Application class for the particle filter localization app using cached data, where the 'cached data' is the results
    from the trunk segmenter and analyzer saved to disk, to avoid having to run the trunk segmenter and analyzer repeatedly
    on the same data when testing the particle filter.
    """
    
    def __init__(self, config_file_path, logging_level=logging.INFO):
        self.using_cached_data = True
        super().__init__(config_file_path, logging_level=logging_level)
        
        # check if it's a file
        if self.parameters_data.test_start_info_path is None:
            raise FileNotFoundError("No test_start_info_path given in config file")           
        if not os.path.isfile(self.parameters_data.test_start_info_path):
            raise FileNotFoundError("Invalid test_start_info_path given in config file")
            

    def init_data_parameters(self):
        """
        Initialize the data parameters
        """
        self.parameters_data = ParametersCachedData()
    
    def init_loaded_data(self):
        """
        Initialize the loaded data
        """
        self.data_file_controls.open_data_file()

    def setup_trunk_data_connection(self):
        """
        Setup the object that connects the app to the trunk segmenter and analyzer
        """
        self.trunk_data_connection = TrunkDataConnectionCachedData(self.parameters_data.cached_image_dir)
        
        self.trunk_data_connection.start()
        self.image_display_checkbox_changed()

    def init_widgets_unique(self):
        """
        Initialize the widgets unique to cached data app
        """
        self.image_browsing_controls = ImageBrowsingControls()
        self.data_file_controls = DataFileControls(self.parameters_data, using_cached_data=True)
        self.image_number_label = ImageNumberLabel()
        self.image_delay_slider = ImageDelaySlider()
        self.cached_data_creator = CachedDataCreator(self)
        self.pf_test_controls = PfTestControls(self)

        self.widget_list += [self.image_browsing_controls, self.data_file_controls, self.image_number_label,
                             self.image_delay_slider, self.cached_data_creator, self.pf_test_controls]
    
    def draw_ui(self):
        """
        Draw the user interface for the cached data app
        """

        mode_change_button_layout = QHBoxLayout()
        mode_change_button_layout.addWidget(self.mode_selector)
        mode_change_button_layout.addWidget(self.change_parameters_button)

        control_layout = QHBoxLayout()
        control_layout.addWidget(self.control_buttons)
        control_layout.addWidget(self.image_delay_slider)

        self.ui_layout = QVBoxLayout()
        
        self.ui_layout.addLayout(mode_change_button_layout)
        self.ui_layout.addWidget(self.checkboxes)
        self.ui_layout.addWidget(self.start_location_controls)
        self.ui_layout.addLayout(control_layout)
        self.ui_layout.addWidget(self.image_display)
        self.ui_layout.addWidget(self.image_number_label)
        self.ui_layout.addWidget(self.image_browsing_controls)
        self.ui_layout.addWidget(self.data_file_controls)
        self.ui_layout.addWidget(self.cached_data_creator)
        self.ui_layout.addWidget(self.pf_test_controls)
        self.ui_layout.addWidget(self.console)
        
        self.main_layout.addLayout(self.ui_layout)
        self.main_layout.addWidget(self.plotter)
    
    def connect_slots_unique(self):
        """
        Connect the app functions unique to the cached data app
        """
        self.data_file_controls.set_image_display.connect(self.trunk_data_connection.handle_request)

        self.data_file_controls.data_file_controls_message.connect(self.print_message)
        self.data_file_controls.reset_pf.connect(self.reset_pf)
        self.data_file_controls.set_img_number_label.connect(self.image_number_label.set_img_number_label)   
        
    def init_modes(self):
        """
        Initialize the different app modes for the cached data app
        """
        self.pf_mode = PfModeCached(self)
        self.modes.append(self.pf_mode)
        
        self.pf_tests_mode = PfModeCachedTests(self)
        self.modes.append(self.pf_tests_mode)        
        
        self.playback_mode = PlaybackMode(self)
        self.modes.append(self.playback_mode)

    def image_display_checkbox_changed(self):
        """
        Override the image display checkbox changed function to handle the cached data case, where the image display
        settings cannot be changed.
        """
        self.trunk_data_connection.set_segmented_image_display_num(0)
        self.print_message("Cannot change image display settings when using cached data")
        

class PfAppLive(PfAppBase):
    """
    Application class for the particle filter localization app using live data from the trunk segmenter and analyzer.
    """

    def __init__(self, config_file_path, logging_level=logging.DEBUG):
        super().__init__(config_file_path, logging_level=logging_level)

    def init_data_parameters(self):
        """
        Initialize the data parameters
        """
        self.parameters_data = ParametersLiveData()

    def setup_trunk_data_connection(self):
        """
        Setup the object that connects the app to the trunk segmenter and analyzer
        """
        # self.trunk_data_connection = TrunkDataConnectionRosService()
        # self.trunk_data_connection.start()
        self.trunk_data_connection = TrunkDataConnectionRosSub(self.parameters_data.rgb_topic, self.parameters_data.gnss_topic)
        self.image_display_checkbox_changed(init=True)
        self.trunk_data_connection.start()
    
    def image_display_checkbox_changed(self, init=False):
        """
        Override the image display checkbox changed function to handle the live data case, where the image display
        settings cannot be changed.

        Args:
            init (bool): Whether the function is being called during initialization
        """

        if not init:
            self.print_message("Cannot change image display settings when using live data")
            
        self.checkboxes.show_rgb_image_checkbox.setChecked(True)
        self.checkboxes.show_pre_filtered_segmentation_checkbox.setChecked(False)
        super().image_display_checkbox_changed()

    def init_widgets_unique(self):
        """
        Initialize the widgets unique to live data app
        """
        self.queue_size_label = PfQueueSizeLabel()

        self.widget_list += [self.queue_size_label]
    
    def draw_ui(self):
        """
        Draw the user interface for the live data app
        """
        mode_change_button_layout = QHBoxLayout()
        mode_change_button_layout.addWidget(self.mode_selector)
        mode_change_button_layout.addWidget(self.change_parameters_button)

        self.ui_layout = QVBoxLayout()
        
        self.ui_layout.addLayout(mode_change_button_layout)
        self.ui_layout.addWidget(self.checkboxes)
        self.ui_layout.addWidget(self.start_location_controls)
        self.ui_layout.addWidget(self.control_buttons)
        self.ui_layout.addWidget(self.queue_size_label)
        self.ui_layout.addWidget(self.image_display)
        self.ui_layout.addWidget(self.console)
        
        self.main_layout.addLayout(self.ui_layout)
        self.main_layout.addWidget(self.plotter)
        
        self.checkboxes.show_pre_filtered_segmentation_checkbox.hide()
        self.checkboxes.show_rgb_image_checkbox.hide()
    

    def connect_slots_unique(self):
        """
        Connect the app functions unique to the live data app
        """
        self.trunk_data_connection.gnss_data_signal.connect(self.plotter.update_gnss_estimate)
        self.trunk_data_connection.gnss_data_signal.connect(self.start_location_controls.set_gps_position)
        self.control_buttons.center_on_gps_button.clicked.connect(self.start_location_controls.set_start_location_from_gps)   
    
    def init_modes(self):
        self.pf_live_mode = PfLiveMode(self)
        self.modes.append(self.pf_live_mode)

    def init_loaded_data(self):
        pass






