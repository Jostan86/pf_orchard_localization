#!/usr/bin/env python3

import sys
from dataclasses import fields
from PyQt5.QtWidgets import QApplication, QVBoxLayout, QHBoxLayout, QWidget
from ..custom_widgets import (PfMainWindow, PfControlButtons, PfStartLocationControls, PfCheckBoxes,
                              Console, ImageDisplay, PfModeSelector, ImageBrowsingControls, TreatingPFPlotter,
                              DataFileControls, DataFileTimeLine, ImageNumberLabel, ImageDelaySlider, CachedDataCreator)
from ..utils.parameters import ParametersPf, ParametersBagData, ParametersCachedData
from ..pf_engine import PFEngine
from ..utils.get_map_data import get_map_data
import logging
from ..trunk_data_connection import TrunkDataConnection, TrunkDataConnectionCachedData
from ..app_modes import PfMode, PfModeCached, PlaybackMode
from ..recorded_data_loaders import BagDataLoader, CachedDataLoader
import os
from functools import partial

# Separate the trunk analyzer import to a separate function to avoid import if not needed


class PfAppBags(PfMainWindow):
    def __init__(self, config_file_path, use_cached_data=False, logging_level=logging.DEBUG):
        super().__init__()

        self.using_cached_data = use_cached_data

        self.parameters_data = ParametersCachedData() if self.using_cached_data else ParametersBagData()

        self.parameters_data.load_from_yaml(config_file_path)
        self.parameters_pf = ParametersPf()
        self.parameters_pf.load_from_yaml(self.parameters_data.pf_config_file_path)

        self.map_data = get_map_data(map_data_path=self.parameters_data.map_data_path)
        self.pf_engine = PFEngine(self.map_data)

        self.init_widgets()

        self.current_msg = None

        self.connect_app_to_ui()

        self.setup_trunk_data_connection()

        self.data_manager = None

        # Select the first item in the combo box
        self.open_data_file()

        if self.parameters_data.initial_data_time is not None and self.data_manager is not None:
            self.data_file_time_line.set_time_line(float(self.parameters_data.initial_data_time))
            self.data_manager.set_time_stamp(float(self.parameters_data.initial_data_time))

        self.pf_mode = PfModeCached(self) if self.using_cached_data else PfMode(self)
        self.playback_mode = PlaybackMode(self)

        self.modes = [self.pf_mode, self.playback_mode]

        self.mode_changed()

    def init_widgets(self):
        # Set up the main window
        self.setWindowTitle("Orchard Localization App")

        self.init_window_display_settings()

        self.main_layout = QHBoxLayout()

        self.ui_layout = QVBoxLayout()

        self.start_location_controls = PfStartLocationControls(self)
        self.control_buttons = PfControlButtons(self)
        self.mode_selector = PfModeSelector()
        self.image_display = ImageDisplay(num_camera_feeds=1, scale_factor=1.0)
        self.image_browsing_controls = ImageBrowsingControls()
        self.data_file_controls = DataFileControls(self, self.parameters_data)
        self.console = Console()
        self.data_file_time_line = DataFileTimeLine()
        self.image_number_label = ImageNumberLabel()
        self.image_delay_slider = ImageDelaySlider()
        self.cached_data_creator = CachedDataCreator(self)

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

        self.plotter = TreatingPFPlotter(self.map_data)

        # Create central widget
        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)
        central_widget.setLayout(self.main_layout)

        # Note that ui_layout always needs to be at the end of this list or bugs will occur, or really, it needs to have
        # top level widgets added to it last
        self.widget_list = [self.start_location_controls, self.checkboxes, self.mode_selector, self.control_buttons,
                            self.image_display, self.image_number_label, self.image_browsing_controls,
                            self.data_file_time_line, self.image_delay_slider, self.data_file_controls,
                            self.cached_data_creator, self.console, self.plotter, self.ui_layout]

    def connect_app_to_ui(self):
        self.control_buttons.reset_button.clicked.connect(self.reset_pf)

        self.plotter.plot_widget.clicked.connect(self.start_location_controls.set_start_location_from_plot_click)

        self.checkboxes.include_width_checkbox.stateChanged.connect(self.include_width_changed)
        self.checkboxes.show_rgb_image_checkbox.stateChanged.connect(self.image_display_checkbox_changed)
        self.checkboxes.show_pre_filtered_segmentation_checkbox.stateChanged.connect(self.image_display_checkbox_changed)

        self.checkboxes.stop_when_converged_checkbox.stateChanged.connect(self.stop_when_converged_changed)

        self.mode_selector.mode_selector.currentIndexChanged.connect(self.mode_changed)

        self.data_file_time_line.data_file_time_line.returnPressed.connect(self.data_file_time_line_edited)

    def disable_all_widgets(self):
        for widget in self.widget_list:
            widget.setParent(None)

    def setup_trunk_data_connection(self):
        if self.using_cached_data:
            self.trunk_data_connection = TrunkDataConnectionCachedData(self.parameters_data.cached_image_dir,
                                                                       self.image_display.load_image,
                                                                       actual_position_plot_func=self.plotter.update_actual_position)
        else:
            self.trunk_data_connection = TrunkDataConnection(self.parameters_data.width_estimation_config_file_path)
            self.image_display_checkbox_changed()

    def image_display_checkbox_changed(self):
        if self.using_cached_data:
            self.print_message("Cannot change image display settings when using cached data")
            return

        show_rgb_image = self.checkboxes.show_rgb_image_checkbox.isChecked()
        show_pre_filtered_segmentation = self.checkboxes.show_pre_filtered_segmentation_checkbox.isChecked()

        num_camera_feeds = 1 + show_rgb_image + show_pre_filtered_segmentation

        # check if the image display is already in the layout
        image_display_index =  self.ui_layout.indexOf(self.image_display)
        if image_display_index != -1:
            self.ui_layout.removeWidget(self.image_display)

        self.widget_list.remove(self.image_display)
        self.image_display.deleteLater()
        self.image_display = ImageDisplay(num_camera_feeds=num_camera_feeds, scale_factor=self.parameters_data.image_display_scale)
        self.widget_list.insert(0, self.image_display)

        if image_display_index != -1:
            self.ui_layout.insertWidget(image_display_index, self.image_display)

        display_num = 0

        if show_rgb_image:
            self.trunk_data_connection.original_image_display_func = partial(self.image_display.load_image, img_num=display_num)
            display_num += 1
        else:
            self.trunk_data_connection.original_image_display_func = None

        if show_pre_filtered_segmentation:
            self.trunk_data_connection.pre_filtered_segmentation_display_func = partial(self.image_display.load_image, img_num=display_num)
            display_num += 1
        else:
            self.trunk_data_connection.pre_filtered_segmentation_display_func = None

        self.trunk_data_connection.seg_image_display_func = partial(self.image_display.load_image, img_num=display_num)

        if self.current_msg is not None:
            self.trunk_data_connection.get_trunk_data(self.current_msg)

    def include_width_changed(self):
        self.parameters_pf.include_width = self.checkboxes.include_width_checkbox.isChecked()

    def stop_when_converged_changed(self):
        self.parameters_pf.stop_when_converged = self.checkboxes.stop_when_converged_checkbox.isChecked()
        
    def mode_changed(self):
        for mode in self.modes:
            if mode.mode_active:
                mode.deactivate_mode()

        self.disable_all_widgets()

        if self.mode_selector.mode == "Scroll Images":
            self.playback_mode.activate_mode()
        elif self.mode_selector.mode == "PF - Recorded Data":
            self.pf_mode.activate_mode()

    def data_file_time_line_edited(self):
        time_stamp = self.data_file_time_line.data_file_time_line.text()
        # Check if the value is a number
        try:
            time_stamp = float(time_stamp)
        except ValueError:
            self.print_message("Invalid time stamp")
            return

        message, self.current_msg = self.data_manager.set_time_stamp(time_stamp)

        self.print_message(message)

        if self.current_msg is not None:
            self.trunk_data_connection.get_trunk_data(self.current_msg)
            self.data_file_time_line.set_time_line(self.data_manager.current_data_file_time_stamp)

        self.image_number_label.set_img_number_label(self.data_manager.current_img_position,
                                                          self.data_manager.num_img_msgs)

    def setup_data_manager(self, data_file_name):

        data_manager = self.data_file_controls.open_data_file(data_file_name)

        if data_manager is not None:
            self.data_manager = data_manager
            self.current_msg = self.data_manager.get_next_img_msg()
            self.trunk_data_connection.get_trunk_data(self.current_msg)
            self.image_number_label.set_img_number_label(self.data_manager.current_img_position,
                                                              self.data_manager.num_img_msgs)

    def open_data_file(self, data_file_name=None, load_first_image=True):

        if data_file_name is None:
            data_file_name = self.data_file_controls.current_data_file_selection

        self.data_file_controls.set_opening()

        data_file_path = self.data_file_controls.data_file_dir + data_file_name

        if not os.path.isfile(data_file_path):
            self.data_file_controls.set_opened("Invalid file name")
            return
        elif self.using_cached_data and data_file_path.endswith(".json"):
            data_manager = CachedDataLoader(data_file_path)
        elif data_file_path.endswith(".bag"):
            data_manager = BagDataLoader(data_file_path, self.parameters_data.depth_topic, self.parameters_data.rgb_topic, self.parameters_data.odom_topic)
        else:
            self.data_file_controls.set_opened("Invalid file type")
            return

        if data_manager.num_img_msgs == 0:
            self.data_file_open_button.set_opened("No images found in data file, check topic names")
            return

        self.data_file_time_line.set_time_line(data_manager.current_data_file_time_stamp)

        self.data_manager = data_manager

        msg = ["Opened bag file: " + data_file_name,]
        msg.append("Number of Odom messages: " + str(self.data_manager.num_odom_msgs))
        msg.append("Number of images: " + str(self.data_manager.num_img_msgs))

        self.data_file_controls.set_opened(msg)

        if load_first_image:
            self.current_msg = self.data_manager.get_next_img_msg()
            self.trunk_data_connection.get_trunk_data(self.current_msg)
            self.image_number_label.set_img_number_label(self.data_manager.current_img_position,
                                                         self.data_manager.num_img_msgs)

    def load_next_data_file(self, load_first_image=True):
        current_data_file_name = self.data_manager.current_data_file_name

        if self.using_cached_data:
            self.print_message("Cannot load next data file when using cached data")
            return False

        next_data_file_name = self.data_file_controls.get_next_data_file_name(current_data_file_name)

        if next_data_file_name is None:
            self.print_message("Reached the end of the data files")
            return False

        self.open_data_file(next_data_file_name, load_first_image)

        return True

    def reset_pf(self):
        self.start_location_controls.get_parameters()

        self.pf_engine.reset_pf(self.parameters_pf)

        self.reset_gui()

    def reset_gui(self):
        self.control_buttons.num_particles_label.setText(str(self.parameters_pf.num_particles))
        self.plotter.update_particles(self.pf_engine.downsample_particles())
        self.plotter.update_position_estimate(None)

    def get_pf_active(self):
        return self.pf_mode.pf_continuous_active

    def get_pf_parameters(self):
        return self.parameters_pf

    def set_pf_parameters(self, parameters):
        if self.pf_mode.pf_continuous_active:
            self.print_message("Cannot change parameters while PF is active")
            return False
        self.parameters_pf = parameters
        return True

    def print_message(self, message):
        if isinstance(message, list):
            for msg in message:
                self.console(msg)
        else:
            self.console(message)

    def display_pf_settings(self):
        for field in fields(self.parameters_pf):
            value = getattr(self.parameters_pf, field.name)
            self.print_message(field.name + ": " + str(value))


if __name__ == "__main__":
    pf_app_bag_config_file_path = "/home/jostan/OneDrive/Docs/Grad_school/Research/code_projects/pf_orchard_localization/config/parameters_pf_app_bags.yaml"
    app = QApplication(sys.argv)

    pf_bag_app = PfAppBags(pf_app_bag_config_file_path)
    pf_bag_app.show()

    sys.exit(app.exec_())





