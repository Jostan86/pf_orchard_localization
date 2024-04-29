#!/usr/bin/env python3

import sys
from dataclasses import fields
from PyQt5.QtWidgets import QApplication, QVBoxLayout, QHBoxLayout, QWidget
from PyQt5.QtCore import QTimer
from ..custom_widgets import (PfMainWindow, PfControlButtons, PfStartLocationControls, PfCheckBoxes,
                              Console, ImageDisplay, PfModeSelector, ImageBrowsingControls, TreatingPFPlotter,
                              DataFileControls, DataFileTimeLine, ImageNumberLabel, ImageDelaySlider)
from ..utils.parameters import ParametersPf, ParametersBagData, ParametersCachedData
from ..pf_engine import PFEngine
from ..utils.get_map_data import get_map_data
import logging
import numpy as np
import cv2
from typing import Callable, Optional, List
import time

# Separate the trunk analyzer import to a separate function to avoid import if not needed
def import_trunk_analyzer(width_estimation_config_file_path):
    from width_estimation import TrunkAnalyzer, TrunkSegmenter
    return TrunkAnalyzer(width_estimation_config_file_path, combine_segmenter=False), TrunkSegmenter(width_estimation_config_file_path)

# class for trunk data connection
class TrunkDataConnection:
    def __init__(self,
                 width_estimation_config_file_path: str = None,
                 seg_image_display_func: Callable[[Optional[np.ndarray], int], None] = None,
                 class_mapping=(1, 2, 0),
                 offset=(0, 0),
                 message_printer: Callable[[List[str]], None] = None):

        self.init_trunk_analyzer(width_estimation_config_file_path)

        self.class_mapping = class_mapping
        self.seg_image_display_func = seg_image_display_func
        self.offset = offset
        self.message_printer = message_printer

    def init_trunk_analyzer(self, width_estimation_config_file_path):
        if width_estimation_config_file_path is not None:
            self.trunk_analyzer, self.trunk_segmenter = import_trunk_analyzer(width_estimation_config_file_path)

    def get_trunk_data(self, current_msg):
        start_time = time.time()
        results_dict, results = self.trunk_segmenter.get_results(current_msg['rgb_image'])

        positions, widths, class_estimates, seg_img = self.get_results(current_msg, results_dict, results)

        print(f"Time to get trunk data: {time.time() - start_time}")

        if positions is not None:
            positions[:, 0] += self.offset[0]
            positions[:, 1] += self.offset[1]

        if seg_img is not None and self.seg_image_display_func is not None:
            self.seg_image_display_func(seg_img, 0)
        elif self.seg_image_display_func is not None:
            self.seg_image_display_func(current_msg['rgb_image'], 0)

        return positions, widths, class_estimates

    def get_results(self, current_msg, results_dict, results):

        positions, widths, class_estimates, x_positions_in_image, results_kept = (
            self.trunk_analyzer.pf_helper(current_msg['depth_image'], results_dict=results_dict))

        if results_kept is not None:
            seg_img = results[results_kept].plot()
        else:
            seg_img = current_msg['rgb_image']

        if class_estimates is not None:
            class_estimates = self.remap_classes(class_estimates)

        return positions, widths, class_estimates, seg_img

    def remap_classes(self, class_estimates):
        for i, mapped_class in enumerate(self.class_mapping):
            class_estimates[class_estimates == i] = mapped_class

        return class_estimates

    def print_messages(self, positions, widths):
        messages = []

        msg_str = "Widths: "
        for width in widths:
            width *= 100
            msg_str += str(round(width, 2)) + "cm,  "
        messages.append(msg_str)
        msg_str = "Positions: "
        for position in positions:
            msg_str += "(" + str(round(position[0], 3)) + ", " + str(round(position[1], 3)) + ") "
        messages.append(msg_str)
        messages.append("---")

        if self.message_printer is not None:
            self.message_printer(messages)

class TrunkDataConnectionImageSelect(TrunkDataConnection):
    def __init__(self,
                 width_estimation_config_file_path: str = None,
                 seg_image_display_func: Callable[[Optional[np.ndarray], int], None] = None,
                 og_seg_image_display_func: Callable[[Optional[np.ndarray], int], None] = None,
                 class_mapping=(1, 2, 0),
                 offset=(0, 0),
                 message_printer: Callable[[List[str]], None] = None):
        super().__init__(width_estimation_config_file_path, seg_image_display_func, class_mapping, offset, message_printer)

        self.og_seg_image_display_func = og_seg_image_display_func

    def get_trunk_data(self, current_msg):
        results_dict, results = self.trunk_segmenter.get_results(current_msg['rgb_image'])

        if self.og_seg_image_display_func is not None:
            seg_img_og = results.plot()
        else:
            seg_img_og = None

        positions, widths, class_estimates, seg_img = self.get_results(current_msg, results_dict, results)

        if seg_img_og is not None:
            self.og_seg_image_display_func(seg_img_og, 0)
        else:
            self.og_seg_image_display_func(current_msg['rgb_image'], 0)

        if seg_img is not None:
            self.seg_image_display_func(seg_img, 1)
        else:
            self.seg_image_display_func(None, 1)

class TrunkDataConnectionCachedData(TrunkDataConnection):
    def __init__(self,
                 cached_img_directory,
                 seg_image_display_func: Callable[[Optional[np.ndarray], int], None],
                 class_mapping=(1, 2, 0),
                 offset=(0, 0),
                 message_printer: Callable[[List[str]], None] = None,
                 actual_position_plot_func: Callable[[np.ndarray], None] = None):

        super().__init__(seg_image_display_func=seg_image_display_func,
                         class_mapping=class_mapping,
                         offset=offset,
                         message_printer=message_printer)

        self.cached_img_directory = cached_img_directory
        self.actual_position_plot_func = actual_position_plot_func

    def get_trunk_data(self, current_msg):

        if current_msg['data'] is None:
            return None, None, None

        msg_data = current_msg['data']['tree_data']
        positions = np.array(msg_data['positions'])
        widths = np.array(msg_data['widths'])
        class_estimates = np.array(msg_data['classes'], dtype=np.int32)

        if self.actual_position_plot_func is not None:
            actual_position = (current_msg['data']['location_estimate'])
            actual_position_np = np.array([actual_position['x'], actual_position['y']])
            self.actual_position_plot_func(actual_position_np)

        seg_img = self.load_cached_img(current_msg['timestamp'])


        if seg_img is not None and self.seg_image_display_func is not None:
            self.seg_image_display_func(seg_img, 0)

        if class_estimates is not None:
            class_estimates = self.remap_classes(class_estimates)
            self.print_messages(positions, widths)

        return positions, widths, class_estimates

    def load_cached_img(self, time_stamp):
        time_stamp = str(int(1000*time_stamp))
        file_path = self.cached_img_directory + "/" + time_stamp + ".png"
        img = cv2.imread(file_path)
        return img


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


class PfAppBags(PfMainWindow):
    def __init__(self, config_file_path, use_cached_data=False, logging_level=logging.DEBUG):
        super().__init__()

        self.using_cached_data = use_cached_data

        if self.using_cached_data:
            self.parameters_data = ParametersCachedData()
        else:
            self.parameters_data = ParametersBagData()

        self.parameters_data.load_from_yaml(config_file_path)
        self.parameters_pf = ParametersPf()
        self.parameters_pf.load_from_yaml(self.parameters_data.pf_config_file_path)

        self.map_data = get_map_data(map_data_path=self.parameters_data.map_data_path)
        self.pf_engine = PFEngine(self.map_data)

        self.init_ui()

        self.current_msg = None

        self.connect_app_to_ui()

        if self.using_cached_data:
            self.trunk_data_connection = TrunkDataConnectionCachedData(self.parameters_data.cached_image_dir,
                                                                       self.image_display.load_image,
                                                                       actual_position_plot_func=self.plotter.update_actual_position)
        else:
            self.trunk_data_connection = TrunkDataConnection(self.parameters_data.width_estimation_config_file_path,
                                                             self.image_display.load_image)

        self.data_manager = None

        # Select the first item in the combo box
        self.setup_data_manager(None)

        if self.parameters_data.initial_data_time is not None and self.data_manager is not None:
            self.data_file_time_line.set_time_line(float(self.parameters_data.initial_data_time))
            self.data_manager.set_time_stamp(float(self.parameters_data.initial_data_time))

        self.pf_mode = PfMode(self)
        self.playback_mode = PlaybackMode(self)

        self.modes = [self.pf_mode, self.playback_mode]

        self.mode_changed()

        self.reset_app()

    def init_ui(self):
        # Set up the main window
        self.setWindowTitle("Orchard Localization App")

        self.init_window_display_settings()

        self.main_layout = QHBoxLayout()

        self.ui_layout = QVBoxLayout()

        self.start_location_controls = PfStartLocationControls(self, self.parameters_pf)
        self.control_buttons = PfControlButtons(self)
        self.mode_selector = PfModeSelector()
        self.image_display = ImageDisplay(num_camera_feeds=1, scale_factor=1.0)
        self.image_browsing_controls = ImageBrowsingControls()
        self.data_file_controls = DataFileControls(self, self.parameters_data)
        self.console = Console()
        self.data_file_time_line = DataFileTimeLine()
        self.image_number_label = ImageNumberLabel()
        self.image_delay_slider = ImageDelaySlider()

        self.checkboxes = PfCheckBoxes()
        self.checkboxes.all_checkbox_info.append(
            ["include_width_checkbox", "Use Width in Weight Calculation", self.parameters_pf.include_width])
        self.checkboxes.all_checkbox_info.append(
            ["stop_when_converged_checkbox", "Stop When Converged", self.parameters_pf.stop_when_converged])
        self.checkboxes.init_checkboxes()

        self.delay_and_time_line_layout = QHBoxLayout()
        self.delay_and_time_line_layout.addWidget(self.data_file_time_line)
        self.delay_and_time_line_layout.addWidget(self.image_delay_slider)

        self.ui_layout.addWidget(self.start_location_controls)
        self.ui_layout.addWidget(self.checkboxes)
        self.ui_layout.addWidget(self.mode_selector)
        self.ui_layout.addWidget(self.control_buttons)
        self.ui_layout.addWidget(self.image_display)
        self.ui_layout.addWidget(self.image_number_label)
        self.ui_layout.addWidget(self.image_browsing_controls)

        self.ui_layout.addLayout(self.delay_and_time_line_layout)
        self.ui_layout.addWidget(self.data_file_controls)
        self.ui_layout.addWidget(self.console)

        self.plotter = TreatingPFPlotter(self.map_data)

        self.main_layout.addLayout(self.ui_layout)
        self.main_layout.addWidget(self.plotter)

        # Create central widget
        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)
        central_widget.setLayout(self.main_layout)

        self.toggle_widget_list = [self.start_location_controls, self.control_buttons,
                                   self.image_display, self.image_number_label, self.image_browsing_controls,
                                   self.data_file_time_line, self.data_file_controls]

    def connect_app_to_ui(self):
        self.control_buttons.reset_button.clicked.connect(self.reset_app)

        self.plotter.plot_widget.clicked.connect(self.start_location_controls.set_start_location_from_plot_click)

        self.checkboxes.include_width_checkbox.stateChanged.connect(self.include_width_changed)

        self.checkboxes.stop_when_converged_checkbox.stateChanged.connect(self.stop_when_converged_changed)

        self.mode_selector.mode_selector.currentIndexChanged.connect(self.mode_changed)

        self.data_file_time_line.data_file_time_line.returnPressed.connect(self.data_file_time_line_edited)

    def enable_all_widgets(self):
        for widget in self.toggle_widget_list:
            widget.enable()

    def disable_all_widgets(self):
        for widget in self.toggle_widget_list:
            widget.disable()

    def reset_app(self):
        self.start_location_controls.get_parameters()

        self.pf_engine.reset_pf(self.parameters_pf)

        self.reset_gui()
    def reset_gui(self):
        self.control_buttons.num_particles_label.setText(str(self.parameters_pf.num_particles))
        self.plotter.update_particles(self.pf_engine.downsample_particles())
        self.plotter.update_position_estimate(None)


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
        elif self.mode_selector.mode == "Manual":
            self.pf_mode.activate_mode()
            self.control_buttons.set_continue()
        elif self.mode_selector.mode == "Continuous":
            self.pf_mode.activate_mode()
            self.control_buttons.set_start()

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

    def load_next_data_file(self):
        self.data_file_controls.load_next_data_file()

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





