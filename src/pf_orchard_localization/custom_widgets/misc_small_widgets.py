from PyQt5.QtWidgets import (QLabel, QPushButton, QVBoxLayout, QHBoxLayout, QWidget, QApplication, QLineEdit, QCheckBox,
                             QPlainTextEdit, QMainWindow, QComboBox, QFileDialog, QInputDialog, QDialog, QSlider,
                             QListWidget, QMessageBox, QSpinBox)
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import Qt, pyqtSignal
import math
import cv2
import numpy as np
from ..utils.parameters import ParametersPf
from . import PfSettingsDialog
import time
import logging
import json
import os
import csv

class PfMainWindow(QMainWindow):
    def __init__(self):
        """Constructs the GUI for the particle filter app"""
        super().__init__()

    def init_window_display_settings(self):
        self.setGeometry(0, 0, 1700, 900)

        desktop = QApplication.desktop()
        target_screen_number = 0
        if target_screen_number < desktop.screenCount():
            target_screen = desktop.screen(target_screen_number)
            self.move(target_screen.geometry().left(), target_screen.geometry().top())

        logging.debug(f"Target screen number: {target_screen_number}")
        logging.debug(f"Screen size: {target_screen.geometry().width()} x {target_screen.geometry().height()}")
        logging.debug(f"App size: {self.width()} x {self.height()}")
        logging.debug(f"App position: {self.x()} x {self.y()}")

class PfChangeParametersButton(QWidget):
    def __init__(self, main_app_manager):
        super().__init__()

        self.main_app_manager = main_app_manager

        # Button to trigger popup to adjust pf settings
        self.adjust_pf_settings_button = QPushButton("Change Particle Filter Parameters")
        self.adjust_pf_settings_button.setToolTip("Adjust the particle filter parameters")
        self.adjust_pf_settings_button.setFixedWidth(300)

        self.layout = QHBoxLayout()

        self.layout.addWidget(self.adjust_pf_settings_button)

        self.setLayout(self.layout)

        self.adjust_pf_settings_button.clicked.connect(self.adjust_pf_settings)

    def adjust_pf_settings(self):
            # Method for adjusting the particle filter settings by opening a settings dialog
            pf_active = self.main_app_manager.get_pf_active()

            if pf_active:
                self.main_app_manager.print_message("Cannot adjust settings while PF is running")
                return

            parameters_pf = self.main_app_manager.get_pf_parameters()

            settings_dialog = PfSettingsDialog(current_settings=parameters_pf)

            if settings_dialog.exec_() == QDialog.Accepted:
                new_settings = settings_dialog.get_settings()
                if new_settings is None:
                    self.main_app_manager.print_message("Update Failed")
                    return
            else:
                self.main_app_manager.print_message("Update Failed")
                return

            success = self.main_app_manager.set_pf_parameters(new_settings)
            if success:
                self.main_app_manager.print_message("Update Successful")
                self.main_app_manager.display_pf_settings()
                self.main_app_manager.reset_pf()
            else:
                self.main_app_manager.print_message("Update Failed")

    def disable(self):
        self.adjust_pf_settings_button.setDisabled(True)

    def enable(self):
        self.adjust_pf_settings_button.setDisabled(False)

class PfControlButtons(QWidget):
    startButtonClicked = pyqtSignal()
    stopButtonClicked = pyqtSignal()

    def __init__(self, main_app_manager):
        super().__init__()

        self.main_app_manager = main_app_manager

        # Button to reset the particle filter
        self.reset_button = QPushButton("Reset PF")
        self.reset_button.setToolTip("Reset the particle filter")

        self.start_stop_button = QPushButton("Start")
        self.start_stop_button.setToolTip("Start the particle filter")

        self.single_step_button = QPushButton("Take Step")
        self.single_step_button.setToolTip("Continue the next step in the particle filter")

        self.top_layer_layout = QHBoxLayout()
        self.top_layer_layout.addWidget(self.start_stop_button)
        self.top_layer_layout.addWidget(self.single_step_button)
        self.top_layer_layout.addWidget(self.reset_button)

        self.num_particles_label = QLabel("0")
        self.num_particles_layout = QHBoxLayout()
        self.num_particles_layout.addWidget(QLabel("Current number of particles:"))
        self.num_particles_layout.addWidget(self.num_particles_label)
        self.num_particles_layout.addStretch(1)

        self.control_layout = QVBoxLayout()
        self.control_layout.addLayout(self.top_layer_layout)
        self.control_layout.addLayout(self.num_particles_layout)

        self.setLayout(self.control_layout)

        self.start_stop_button.clicked.connect(self.start_stop_button_clicked)

    def set_num_particles(self, num_particles):
        self.num_particles_label.setText(str(num_particles))
    def start_stop_button_clicked(self):
        if self.start_stop_button.text() == "Start":
            self.startButtonClicked.emit()
        else:
            self.stopButtonClicked.emit()

    def disable(self):
        self.reset_button.setDisabled(True)
        self.adjust_pf_settings_button.setDisabled(True)
        self.start_stop_button.setDisabled(True)

    def enable(self):
        self.reset_button.setDisabled(False)
        self.adjust_pf_settings_button.setDisabled(False)
        self.start_stop_button.setDisabled(False)

    def set_start(self):
        self.start_stop_button.setText("Start")
        self.start_stop_button.setToolTip("Start the particle filter")

    def set_stop(self):
        self.start_stop_button.setText("Stop")
        self.start_stop_button.setToolTip("Stop the particle filter")

class PfStartLocationControls(QWidget):
    def __init__(self, main_app_manager):
        super().__init__()

        self.main_app_manager = main_app_manager

        self.start_x_input = QLineEdit()
        self.start_x_input.setToolTip(
            "X coordinate of the center of the starting block, shift click on the plot to set this")
        self.start_y_input = QLineEdit()
        self.start_y_input.setToolTip(
            "Y coordinate of the center of the starting block, shift click on the plot to set this")
        self.rotation_input = QLineEdit()
        self.rotation_input.setToolTip("Rotation of the starting block in degrees, -32 aligns with the rows nicely")

        self.orientation_center_input = QLineEdit()
        self.orientation_center_input.setToolTip("Mid value of the orientations of the particles")

        self.orientation_range_input = QLineEdit()
        self.orientation_range_input.setToolTip(
            "Range of the orientations of the particles, centered around the mid value."
            "So a mid value of 0 and a range of 10 would result in particles with orientations"
            "between -5 and 5 degrees.")

        self.start_location_layout = QHBoxLayout()
        self.start_location_layout.addWidget(QLabel("Start Pose Center:"))
        self.start_location_layout.addWidget(QLabel("X:"))
        self.start_location_layout.addWidget(self.start_x_input)
        self.start_location_layout.addWidget(QLabel("Y:"))
        self.start_location_layout.addWidget(self.start_y_input)
        self.start_location_layout.addWidget(QLabel("Rotation (deg):"))
        self.start_location_layout.addWidget(self.rotation_input)

        self.start_orientation_layout = QHBoxLayout()
        self.start_orientation_layout.addWidget(QLabel("Orientation Center (deg):"))
        self.start_orientation_layout.addWidget(self.orientation_center_input)
        self.start_orientation_layout.addWidget(QLabel("Orientation Range (deg):"))
        self.start_orientation_layout.addWidget(self.orientation_range_input)

        self.start_width_input = QLineEdit()
        self.start_width_input.setToolTip("Width of the starting block")
        self.start_height_input = QLineEdit()
        self.start_height_input.setToolTip("Height of the starting block")

        self.start_width_height_layout = QHBoxLayout()
        self.start_width_height_layout.addWidget(QLabel("Start Pose Size:"))
        self.start_width_height_layout.addWidget(QLabel("Width:"))
        self.start_width_height_layout.addWidget(self.start_width_input)
        self.start_width_height_layout.addWidget(QLabel("Height:"))
        self.start_width_height_layout.addWidget(self.start_height_input)

        self.start_layout = QVBoxLayout()
        self.start_layout.addLayout(self.start_location_layout)
        self.start_layout.addLayout(self.start_width_height_layout)
        self.start_layout.addLayout(self.start_orientation_layout)

        self.setLayout(self.start_layout)

        self.set_parameters()

    def set_parameters(self):
        pf_settings = self.main_app_manager.get_pf_parameters()
        self.start_x_input.setText(str(pf_settings.start_pose_center_x))
        self.start_y_input.setText(str(pf_settings.start_pose_center_y))
        self.rotation_input.setText(str(pf_settings.start_rotation))
        self.orientation_center_input.setText(str(pf_settings.start_orientation_center))
        self.orientation_range_input.setText(str(pf_settings.start_orientation_range))
        self.start_width_input.setText(str(pf_settings.start_width))
        self.start_height_input.setText(str(pf_settings.start_height))

    def get_parameters(self):
        current_settings = self.main_app_manager.get_pf_parameters()
        current_settings.start_pose_center_x = float(self.start_x_input.text())
        current_settings.start_pose_center_y = float(self.start_y_input.text())
        current_settings.start_rotation = float(self.rotation_input.text())
        current_settings.start_orientation_center = float(self.orientation_center_input.text())
        current_settings.start_orientation_range = float(self.orientation_range_input.text())
        current_settings.start_width = float(self.start_width_input.text())
        current_settings.start_height = float(self.start_height_input.text())

        success = self.main_app_manager.set_pf_parameters(current_settings)

        if not success:
            self.main_app_manager.print_message("Failed to set start location parameters")
    def set_start_location_from_plot_click(self, x, y, shift_pressed):
        # Method for handling when the plot is clicked, if shift is held down, set the particle start position to the
        # clicked location

        if self.main_app_manager.get_pf_active():
            return
        if x is None:
            return

        self.main_app_manager.print_message("Plot clicked at: x = " + str(round(x, 2)) + ", y = " + str(round(y, 2)))
        if shift_pressed:
            self.start_x_input.setText(str(round(x, 2)))
            self.start_y_input.setText(str(round(y, 2)))
            self.main_app_manager.reset_pf()
        else:
            self.main_app_manager.print_message("Shift click to set particle start position")


    def disable(self):
        self.start_x_input.setReadOnly(True)
        self.start_y_input.setReadOnly(True)
        self.rotation_input.setReadOnly(True)
        self.orientation_center_input.setReadOnly(True)
        self.orientation_range_input.setReadOnly(True)
        self.start_width_input.setReadOnly(True)
        self.start_height_input.setReadOnly(True)

    def enable(self):
        self.start_x_input.setReadOnly(False)
        self.start_y_input.setReadOnly(False)
        self.rotation_input.setReadOnly(False)
        self.orientation_center_input.setReadOnly(False)
        self.orientation_range_input.setReadOnly(False)
        self.start_width_input.setReadOnly(False)
        self.start_height_input.setReadOnly(False)

class PfCheckBoxes(QWidget):
    def __init__(self):
        super().__init__()
        self.all_checkbox_info = []
        self.num_boxes_per_row = 3

    def init_checkboxes(self):
        num_checkboxes = len(self.all_checkbox_info)
        num_rows = math.ceil(num_checkboxes/self.num_boxes_per_row)

        self.checkbox_overall_layout = QVBoxLayout()

        for i in range(num_rows):
            checkboxes_info_sub_section = self.all_checkbox_info[i*self.num_boxes_per_row:(i+1)*self.num_boxes_per_row]
            checkbox_layout = QHBoxLayout()

            for checkbox_info in checkboxes_info_sub_section:
                checkbox = QCheckBox(checkbox_info[1])
                checkbox.setChecked(checkbox_info[2])
                setattr(self, checkbox_info[0], checkbox)
                checkbox_layout.addWidget(checkbox)

            self.checkbox_overall_layout.addLayout(checkbox_layout)

        self.setLayout(self.checkbox_overall_layout)

    def disable(self):
        for checkbox_info in self.all_checkbox_info:
            getattr(self, checkbox_info[0]).setDisabled(True)

    def enable(self):
        for checkbox_info in self.all_checkbox_info:
            getattr(self, checkbox_info[0]).setDisabled(False)



class Console(QWidget):
    def __init__(self):
        super().__init__()

        self.console = QPlainTextEdit(self)
        self.console.setReadOnly(True)

        self.clear_console_button = QPushButton("Clear Console")
        self.clear_console_button.clicked.connect(self.console.clear)

        self.console_layout = QVBoxLayout()
        self.console_layout.addWidget(self.console)
        self.console_layout.addWidget(self.clear_console_button)

        self.setLayout(self.console_layout)

    def __call__(self, message):
        self.print_message(message)

    def print_message(self, message):
        self.console.appendPlainText(message)

    def print_time_message(self, time_s, msg, ms=True):
        time_tot = (time.time() - time_s)
        if ms:
            time_tot = time_tot * 1000
        time_tot = round(time_tot, 1)
        if ms:
            msg_str = msg + str(time_tot) + "ms"
        else:
            msg_str = msg + str(time_tot) + "s"
        self.print_message(msg_str)

    def disable(self):
        self.clear_console_button.setDisabled(True)

    def enable(self):
        self.clear_console_button.setDisabled(False)

class ImageDisplay(QWidget):
    def __init__(self, num_camera_feeds=1, image_size=(480, 640), scale_factor=1.5):
        super().__init__()

        logging.debug(f"Starting Image Display with {num_camera_feeds} camera feeds, image size {image_size}, scale factor {scale_factor}")

        self.num_camera_feeds = num_camera_feeds
        self.image_height = int(image_size[0] * scale_factor)
        self.image_width = int(image_size[1] * scale_factor)

        self.main_layout = QHBoxLayout()

        self.picture_layout = QHBoxLayout()
        self.picture_labels = []
        for i in range(self.num_camera_feeds):
            picture_label = QLabel(self)
            picture_label.resize(self.image_height, self.image_width)
            picture_label.setAlignment(Qt.AlignCenter)
            self.picture_labels.append(picture_label)
            self.picture_layout.addWidget(picture_label)
            self.load_image(img=None, img_num=i)

        self.main_layout.addLayout(self.picture_layout)
        self.setLayout(self.main_layout)

    def load_image(self, img=None, img_num=0):
        """
        Load an image into the GUI image viewer
        Parameters
        ----------
        img : OpenCV image

        Returns None
        -------

        """
        # If image is none make a blank image
        if img is None:
            img = np.ones((self.image_height, self.image_width, 3), dtype=np.uint8) * 155

        # Convert the image to a Qt image and display it
        image_cv2 = img
        image_rgb = cv2.cvtColor(image_cv2, cv2.COLOR_BGR2RGB)
        image_qt = QImage(image_rgb.data, image_rgb.shape[1], image_rgb.shape[0], QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(image_qt)

        pixmap_scaled = pixmap.scaled(self.picture_labels[img_num].size(), Qt.KeepAspectRatio)
        self.picture_labels[img_num].setPixmap(pixmap_scaled)

        QApplication.processEvents()

    def disable(self):
        for label in self.picture_labels:
            label.setDisabled(True)

    def enable(self):
        for label in self.picture_labels:
            label.setDisabled(False)


class PfModeSelector(QWidget):
    def __init__(self, mode_options=()):
        super().__init__()

        logging.debug(f"Starting Mode Selector with options {mode_options}")

        self.mode_selector = QComboBox()
        self.mode_selector.setFixedWidth(300)

        mode_selector_layout = QHBoxLayout()

        mode_label = QLabel("Mode:")
        mode_selector_layout.addWidget(mode_label)
        mode_selector_layout.addWidget(self.mode_selector)
        mode_selector_layout.addStretch(1)

        self.setLayout(mode_selector_layout)

    def set_modes(self, modes):
        self.mode_selector.clear()
        for mode in modes:
            self.mode_selector.addItem(mode.mode_name)

    @property
    def mode(self):
        return self.mode_selector.currentText()

    def disable(self):
        self.mode_selector.setDisabled(True)

    def enable(self):
        self.mode_selector.setDisabled(False)


class ImageBrowsingControls(QWidget):
    playButtonClicked = pyqtSignal(str)

    def __init__(self):
        super().__init__()

        self.previous_button = QPushButton("Previous")
        self.previous_button.setToolTip("Go to the previous image")
        self.next_button = QPushButton("Next")
        self.next_button.setToolTip("Go to the next image")
        self.play_fwd_button = QPushButton("Play")
        self.play_fwd_button.setToolTip("Play the images forward")

        self.save_button = QPushButton("Save Image")
        self.save_button.setToolTip("Save the current image")
        self.save_button.setMinimumWidth(200)
        self.save_location_input = QLineEdit()
        self.save_location_input.setToolTip("Location to save the images to, click change to change this location")
        self.save_location_input.setPlaceholderText("Save Location")
        self.save_location_change_button = QPushButton("Change")
        self.save_location_change_button.setToolTip("Change the location to save the images to")
        self.save_location_change_button.setMinimumWidth(150)

        self.img_browsing_buttons_layout = QHBoxLayout()
        self.img_browsing_buttons_layout.addWidget(self.previous_button)
        self.img_browsing_buttons_layout.addWidget(self.next_button)
        self.img_browsing_buttons_layout.addWidget(self.play_fwd_button)

        self.save_settings_layout = QHBoxLayout()
        self.save_settings_layout.addWidget(self.save_button)
        self.save_settings_layout.addWidget(self.save_location_input)
        self.save_settings_layout.addWidget(self.save_location_change_button)

        self.overall_layout = QVBoxLayout()
        self.overall_layout.addLayout(self.img_browsing_buttons_layout)
        self.overall_layout.addLayout(self.save_settings_layout)

        self.setLayout(self.overall_layout)

        self.play_fwd_button.clicked.connect(self.emit_play_button_clicked)
        self.save_location_change_button.clicked.connect(self.change_save_location)

    def emit_play_button_clicked(self):
        command = self.play_fwd_button.text()
        self.playButtonClicked.emit(command)

    def change_save_location(self):
        save_location = QFileDialog.getExistingDirectory(self, "Select Save Location") + "/"
        self.save_location_input.setText(save_location)

    @property
    def save_location(self):
        return self.save_location_input.text()

    def disable(self):
        self.previous_button.setDisabled(True)
        self.next_button.setDisabled(True)
        self.play_fwd_button.setDisabled(True)
        self.save_button.setDisabled(True)

    def enable(self):
        self.previous_button.setDisabled(False)
        self.next_button.setDisabled(False)
        self.play_fwd_button.setDisabled(False)
        self.save_button.setDisabled(False)

class DataFileTimeLine(QWidget):
    def __init__(self):
        super().__init__()

        self.data_file_time_line = QLineEdit()
        self.data_file_time_line.setFixedWidth(150)

        label = QLabel("Current Data File Time:")
        label.setFixedWidth(175)

        self.data_file_time_line_layout = QHBoxLayout()
        self.data_file_time_line_layout.addWidget(label)
        self.data_file_time_line_layout.addWidget(self.data_file_time_line)

        self.setFixedWidth(350)

        self.setLayout(self.data_file_time_line_layout)

    def set_time_line(self, time_stamp: float):
        time_stamp = round(time_stamp, 2)
        self.data_file_time_line.setText(str(time_stamp))

    def disable(self):
        self.data_file_time_line.setDisabled(True)

    def enable(self):
        self.data_file_time_line.setDisabled(False)

class ImageNumberLabel(QWidget):
    def __init__(self):
        super().__init__()

        self.img_number_label = QLabel(self)
        self.img_number_label.setAlignment(Qt.AlignCenter)

        self.img_number_layout = QVBoxLayout()
        self.img_number_layout.addWidget(self.img_number_label)

        self.setLayout(self.img_number_layout)

    def set_img_number_label(self, img_number, total_imgs):
        self.img_number_label.setText("Image " + str(img_number) + " of " + str(total_imgs))

    def disable(self):
        self.img_number_label.setDisabled(True)

    def enable(self):
        self.img_number_label.setDisabled(False)

class ImageDelaySlider(QWidget):
    def __init__(self):
        super().__init__()

        label = QLabel("Delay Between Images:")

        start_value_ms = 50

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setMaximum(500)
        self.slider.setValue(start_value_ms)
        self.slider.setTickInterval(10)
        self.slider.setTickPosition(QSlider.NoTicks)

        self.value_label = QLabel(str(start_value_ms) + " ms")

        label.setFixedWidth(175)
        self.slider.setFixedWidth(200)
        self.value_label.setFixedWidth(60)
        self.setFixedWidth(465)

        self.slider_layout = QHBoxLayout()
        self.slider_layout.addWidget(label)
        self.slider_layout.addWidget(self.slider)
        self.slider_layout.addWidget(self.value_label)
        self.setLayout(self.slider_layout)

        self.slider.valueChanged.connect(self.update_value_label)

    def update_value_label(self):
        self.value_label.setText(str(self.slider.value()) + "ms")

    def get_delay_ms(self):
        return self.slider.value()

    def disable(self):
        self.slider.setDisabled(True)

    def enable(self):
        self.slider.setDisabled(False)

class PfQueueSizeLabel(QWidget):
    def __init__(self):
        super().__init__()

        self.queue_size_label = QLabel(self)
        self.queue_size_label.setToolTip("Number of image messages in the queue")

        self.queue_size_layout = QVBoxLayout()
        self.queue_size_layout.addWidget(self.queue_size_label)

        self.setLayout(self.queue_size_layout)

    def set_queue_size(self, queue_size):
        self.queue_size_label.setText("Queue Size: " + str(queue_size))

    def disable(self):
        self.queue_size_label.setDisabled(True)

    def enable(self):
        self.queue_size_label.setDisabled(False)

class RosConnectButton(QWidget):
    disconnectButtonClicked = pyqtSignal()
    connectButtonClicked = pyqtSignal()

    def __init__(self):
        super().__init__()

        self.connect_button = QPushButton("Connect to ROS")
        self.connect_button.setToolTip("Connect to ROS")
        self.connect_button_layout = QVBoxLayout()
        self.connect_button_layout.addWidget(self.connect_button)

        self.connect_button.clicked.connect(self.connect_button_clicked)

        self.setLayout(self.connect_button_layout)

    def set_disconnected(self):
        self.connect_button.setText("Connect to ROS")
        self.connect_button.setToolTip("Connect to ROS")

    def set_connected(self):
        self.connect_button.setText("Disconnect from ROS")
        self.connect_button.setToolTip("Disconnect from ROS")

    def connect_button_clicked(self):
        if self.connect_button.text() == "Connect to ROS":
            self.connectButtonClicked.emit()
        else:
            self.disconnectButtonClicked.emit()

    def disable(self):
        self.connect_button.setDisabled(True)

    def enable(self):
        self.connect_button.setDisabled(False)

class CachedDataCreator(QWidget):
    def __init__(self, main_app_manager):
        super().__init__()

        self.main_app_manager = main_app_manager

        self.cache_data_enabled = False
        self.cache = {}

        button_width = 140

        self.enable_checkbox = QCheckBox("Cache Data")
        self.enable_checkbox.setToolTip("Enable caching of data")
        self.enable_checkbox.setChecked(self.cache_data_enabled)
        self.enable_checkbox.setMinimumWidth(180)

        self.save_label = QLabel("Save Directory:")
        self.save_label.setToolTip("Directory to save the cached data in")
        self.save_label.setMinimumWidth(105)

        self.save_directory_input = QLineEdit()
        self.save_directory_input.setToolTip("Directory to save the cached data in")
        self.save_directory_input.setPlaceholderText("Save Directory")
        self.save_directory_input.setReadOnly(True)

        self.change_save_directory_button = QPushButton("Change")
        self.change_save_directory_button.setToolTip("Change the directory to save the cached data in")
        self.change_save_directory_button.setMinimumWidth(button_width)

        self.save_images_checkbox = QCheckBox("Save Images")
        self.save_images_checkbox.setToolTip("Save segmented image also for display on playback")
        self.save_images_checkbox.setChecked(True)
        self.save_images_checkbox.setMinimumWidth(button_width)

        self.cache_size_label = QLabel("Cache Size: 0 messages")
        self.cache_size_label.setToolTip("Number of messages currently in the cache")
        self.cache_size_label.setMinimumWidth(180)

        self.file_name_label = QLabel("File Name:")
        self.file_name_label.setToolTip("Name of the file to save the cached data as")
        self.file_name_label.setMinimumWidth(105)

        self.file_name_input = QLineEdit()
        self.file_name_input.setToolTip("Name of the file to save the cached data as")
        self.start_text = "File Name (e.g. run1_data)"
        self.file_name_input.setPlaceholderText(self.start_text)

        self.save_button = QPushButton("Save Cache")
        self.save_button.setToolTip("Save the cached data to the specified file")
        self.save_button.setMinimumWidth(button_width)

        self.reset_cache_button = QPushButton("Reset Cache")
        self.reset_cache_button.setToolTip("Reset the cache")
        self.reset_cache_button.setMinimumWidth(button_width)


        self.main_layout = QVBoxLayout()
        self.top_layout = QHBoxLayout()
        self.bottom_layout = QHBoxLayout()

        self.top_layout.addWidget(self.enable_checkbox)
        self.top_layout.addWidget(self.save_label)
        self.top_layout.addWidget(self.save_directory_input)
        self.top_layout.addWidget(self.change_save_directory_button)
        self.top_layout.addWidget(self.save_images_checkbox)

        self.bottom_layout.addWidget(self.cache_size_label)
        self.bottom_layout.addWidget(self.file_name_label)
        self.bottom_layout.addWidget(self.file_name_input)
        self.bottom_layout.addWidget(self.save_button)
        self.bottom_layout.addWidget(self.reset_cache_button)

        self.main_layout.addLayout(self.top_layout)
        self.main_layout.addLayout(self.bottom_layout)

        self.setLayout(self.main_layout)

        self.enable_checkbox.stateChanged.connect(self.cache_data_checkbox_changed)
        self.change_save_directory_button.clicked.connect(self.change_save_directory)
        self.save_button.clicked.connect(self.save_cache)
        self.reset_cache_button.clicked.connect(self.reset_cache)

        self.cache_data_checkbox_changed()

    def change_save_directory(self):
        save_location = QFileDialog.getExistingDirectory(self, "Select Save Location") + "/"
        self.save_directory_input.setText(save_location)
        # check for "images" folder, if it doesn't exist, create it

    def reset_cache(self):
        self.cache = {}
        self.cache_size_label.setText("Cache Size: 0 messages")

    def get_timestamp_str(self, time_stamp):
        return str(time_stamp*1000).split(".")[0]
    def cache_odom_data(self, x_odom, theta_odom, time_stamp_odom):
        self.cache[self.get_timestamp_str(time_stamp_odom)] = {"x_odom": x_odom, "theta_odom": theta_odom, "time_stamp": time_stamp_odom}
        self.cache_size_label.setText("Cache Size: " + str(len(self.cache)) + " messages")

    def cache_tree_data(self, positions, widths, class_estimates, location_estimate, time_stamp):
        if positions is None:
            self.cache[self.get_timestamp_str(time_stamp)] = None
            return
        tree_data = {"positions": positions.tolist(), "widths": widths.tolist(), "classes": class_estimates.tolist()}
        location_estimate = {"x": location_estimate[0], "y": location_estimate[1], "theta": location_estimate[2]}
        self.cache[self.get_timestamp_str(time_stamp)] = {"tree_data": tree_data, "location_estimate": location_estimate}

    def save_cache(self):
        if len(self.cache) == 0:
            self.main_app_manager.print_message("No data to save")
            return

        if not os.path.exists(self.save_directory_input.text()):
            self.main_app_manager.print_message("Save directory does not exist")
            return

        if self.file_name_input.text() == "" or self.file_name_input.text() == self.start_text:
            self.main_app_manager.print_message("Please enter a file name")
            return

        save_location = self.save_directory_input.text() + self.file_name_input.text() + ".json"

        # check if file already exists, if so, have popup to ask if they want to overwrite
        if os.path.exists(save_location):
            msg_box = QMessageBox()
            msg_box.setIcon(QMessageBox.Warning)
            msg_box.setText("File already exists")
            msg_box.setInformativeText("Do you want to overwrite the file?")
            msg_box.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
            msg_box.setDefaultButton(QMessageBox.No)
            ret = msg_box.exec_()
            if ret == QMessageBox.No:
                return

        with open(save_location, 'w') as f:
            json.dump(self.cache, f)

        self.main_app_manager.print_message("Cache saved to: " + save_location)

    def save_image(self, img, time_stamp):
        if self.save_images_checkbox.isChecked() == False:
            self.main_app_manager.print_message("Images not being saved")
            return

        save_directory = self.save_directory_input.text()

        if not os.path.exists(save_directory):
            self.main_app_manager.print_message("Save directory does not exist")
            return

        save_directory = save_directory + "images/"

        if not os.path.exists(save_directory):
            os.makedirs(save_directory)

        save_location = save_directory + self.get_timestamp_str(time_stamp) + ".png"

        cv2.imwrite(save_location, img)

    def cache_data_checkbox_changed(self):
        if self.enable_checkbox.isChecked():
            self.cache_data_enabled = True
            self.set_input_disabled(False)
        else:
            self.cache_data_enabled = False
            self.reset_cache()
            self.set_input_disabled(True)

    def set_input_disabled(self, disabled):
        self.save_images_checkbox.setDisabled(disabled)
        self.save_directory_input.setDisabled(disabled)
        self.change_save_directory_button.setDisabled(disabled)
        self.file_name_input.setDisabled(disabled)
        self.save_button.setDisabled(disabled)
        self.reset_cache_button.setDisabled(disabled)

    def enable (self):
        self.enable_checkbox.setDisabled(False)
        self.set_input_disabled(False)

    def disable(self):
        self.enable_checkbox.setDisabled(True)
        self.set_input_disabled(True)


class PfTestControls(QWidget):
    runAllTestsClicked = pyqtSignal()
    abortAllTestsClicked = pyqtSignal()
    runSelectedTestClicked = pyqtSignal()
    abortSelectedTestClicked = pyqtSignal()

    def __init__(self, main_app_manager):
        super().__init__()

        self.main_app_manager = main_app_manager

        self.run_all_button = QPushButton("Run All Tests")
        self.run_all_button.setToolTip("Start the test regimen")
        self.run_all_button.setMinimumWidth(120)

        self.run_selected_button = QPushButton("Run Selected Test")
        self.run_selected_button.setToolTip("Run the selected test")
        self.run_selected_button.setMinimumWidth(120)

        self.reset_button = QPushButton("Reset")
        self.reset_button.setToolTip("Reset the test regimen")
        self.reset_button.setMinimumWidth(120)

        self.save_button = QPushButton("Save Results")
        self.save_button.setToolTip("Save the test results to a file")
        self.save_button.setMinimumWidth(120)

        self.test_selection_label = QLabel("Test Selection:")
        self.test_selection_label.setToolTip("Select the test to run")

        self.test_selection_combobox = QComboBox()
        self.test_selection_combobox.setToolTip("Select the test to run")

        self.load_tests_button = QPushButton("Load Test Regimen")
        self.load_tests_button.setToolTip("Load a test regimen from a file")
        self.load_tests_button.setFixedWidth(200)

        self.tests_per_location_label = QLabel("Tests Per Location:")
        self.tests_per_location_label.setToolTip("Number of tests to run at each location")

        self.tests_per_location_spinbox = QSpinBox()
        self.tests_per_location_spinbox.setToolTip("Number of tests to run at each location")
        self.tests_per_location_spinbox.setMinimum(1)
        self.tests_per_location_spinbox.setMaximum(100)
        self.tests_per_location_spinbox.setValue(1)
        self.tests_per_location_spinbox.setFixedWidth(50)

        self.test_number_label = QLabel("Test:")
        self.test_number_label.setToolTip("Current test number")
        self.test_number_label.setFixedWidth(80)

        self.trial_number_label = QLabel("Trial:")
        self.trial_number_label.setToolTip("Current trial number")
        self.trial_number_label.setFixedWidth(80)

        self.time_elapsed_label = QLabel("Time Elapsed:")
        self.time_elapsed_label.setToolTip("Time elapsed since trial start")
        self.time_elapsed_label.setFixedWidth(150)

        self.particle_number_label = QLabel("Particles Count:")
        self.particle_number_label.setToolTip("Number of particles currently in the filter")
        self.particle_number_label.setFixedWidth(180)

        self.convergence_rate_label = QLabel("Convergence Rate: Current: 0.0, All: 0.0")
        self.convergence_rate_label.setToolTip("Convergence rate of all the tests")
        self.convergence_rate_label.setFixedWidth(300)

        self.test_time_label = QLabel("Average Time: Current: 0.0, All: 0.0")
        self.test_time_label.setToolTip("Average time of all the tests")
        self.test_time_label.setFixedWidth(300)

        self.main_layout = QVBoxLayout()

        self.layer_1_layout = QHBoxLayout()
        self.layer_1_layout.addWidget(self.run_all_button)
        self.layer_1_layout.addWidget(self.run_selected_button)
        self.layer_1_layout.addWidget(self.reset_button)
        self.layer_1_layout.addWidget(self.save_button)

        self.layer_2_layout = QHBoxLayout()
        self.layer_2_layout.addWidget(self.test_selection_label)
        self.layer_2_layout.addWidget(self.test_selection_combobox)
        self.layer_2_layout.addWidget(self.load_tests_button)
        self.layer_2_layout.addWidget(self.tests_per_location_label)
        self.layer_2_layout.addWidget(self.tests_per_location_spinbox)

        self.layer_3_layout = QHBoxLayout()
        self.layer_3_layout.addWidget(self.test_number_label)
        self.layer_3_layout.addWidget(self.trial_number_label)
        self.layer_3_layout.addWidget(self.time_elapsed_label)
        self.layer_3_layout.addWidget(self.particle_number_label)

        self.layer_4_layout = QHBoxLayout()
        self.layer_4_layout.addWidget(self.convergence_rate_label)
        self.layer_4_layout.addWidget(self.test_time_label)

        self.main_layout.addLayout(self.layer_1_layout)
        self.main_layout.addLayout(self.layer_2_layout)
        self.main_layout.addLayout(self.layer_3_layout)
        self.main_layout.addLayout(self.layer_4_layout)

        self.setLayout(self.main_layout)

        self.load_tests_button.clicked.connect(self.load_tests_button_clicked)
        self.run_all_button.clicked.connect(self.run_all_button_clicked)
        self.run_selected_button.clicked.connect(self.run_selected_button_clicked)
        self.reset_button.clicked.connect(self.reset_tests)
        self.save_button.clicked.connect(self.save_results)

        self.test_regimen = None

    def run_all_button_clicked(self):
        if not self.tests_loaded:
            return
        if self.run_all_button.text() == "Run All Tests":
            self.runAllTestsClicked.emit()
        else:
            self.abortAllTestsClicked.emit()

    def run_selected_button_clicked(self):
        if not self.tests_loaded:
            return
        if self.run_selected_button.text() == "Run Selected Test":
            self.runSelectedTestClicked.emit()
        else:
            self.abortSelectedTestClicked.emit()

    @property
    def tests_loaded(self):
        if self.test_regimen is None:
            self.main_app_manager.print_message("No tests loaded")
            return False
        if len(self.test_regimen.pf_tests) == 0:
            self.main_app_manager.print_message("No tests loaded")
            return False
        return True

    def set_running_all_tests(self, running_all_tests: bool):
        if running_all_tests:
            self.run_all_button.setText("Abort Tests")
            self.run_all_button.setToolTip("Abort the test regimen")
        else:
            self.run_all_button.setText("Run All Tests")
            self.run_all_button.setToolTip("Start the test regimen")
        self.run_selected_button.setDisabled(running_all_tests)
        self.reset_button.setDisabled(running_all_tests)
        self.save_button.setDisabled(running_all_tests)
        self.load_tests_button.setDisabled(running_all_tests)
        self.tests_per_location_spinbox.setDisabled(running_all_tests)
        self.test_selection_combobox.setDisabled(running_all_tests)


    def set_running_selected_test(self, running_selected_test: bool):
        if running_selected_test:
            self.run_selected_button.setText("Abort Test")
            self.run_selected_button.setToolTip("Abort the selected test")
        else:
            self.run_selected_button.setText("Run Selected Test")
            self.run_selected_button.setToolTip("Run the selected test")
        self.run_all_button.setDisabled(running_selected_test)
        self.reset_button.setDisabled(running_selected_test)
        self.save_button.setDisabled(running_selected_test)
        self.load_tests_button.setDisabled(running_selected_test)
        self.tests_per_location_spinbox.setDisabled(running_selected_test)
        self.test_selection_combobox.setDisabled(running_selected_test)

    def get_selected_test(self):
        return self.test_selection_combobox.currentIndex()

    def get_num_trials_per_location(self):
        return self.tests_per_location_spinbox.value()

    def load_tests_button_clicked(self):
        file_name = QFileDialog.getOpenFileName(self, "Select Test Regimen", filter="*.csv")
        if file_name == "":
            return
        else:
            self.load_test(file_name[0])

    def load_test(self, file_name):
        self.test_regimen = PfTestRegimen(file_name, self.main_app_manager.print_message)

        self.test_selection_combobox.clear()
        for test_name in self.test_regimen.test_names:
            self.test_selection_combobox.addItem(test_name)

    def save_results(self):
        save_path = QFileDialog.getSaveFileName(self, "Save Results", filter="*.csv")
        if save_path == "":
            return
        self.test_regimen.process_results(save_path[0])

    def reset_tests(self):
        if not self.tests_loaded:
            return

        self.test_regimen.reset_tests()

    def update_trial_info(self, trial_time_elapsed, num_particles):
        self.time_elapsed_label.setText("Time Elapsed: " + str(round(trial_time_elapsed, 1)) + "s")
        self.particle_number_label.setText("Particles Count: " + str(num_particles))

    def update_trial_number(self, trial_num):
        self.trial_number_label.setText("Trial: " + str(trial_num))

    def update_test_number(self, test_num):
        self.test_number_label.setText("Test: " + str(test_num))

    def update_convergence_rate(self, convergence_rate_current, convergence_rate_all):
        self.convergence_rate_label.setText("Convergence Rate: Current: " + str(round(convergence_rate_current, 2)) + ", All: " + str(round(convergence_rate_all, 2)))

    def update_test_time(self, test_time_current, test_time_all):
        self.test_time_label.setText("Average Time: Current: " + str(round(test_time_current, 2)) + ", All: " + str(round(test_time_all, 2)))





class PfTestRegimen:
    def __init__(self, file_name, print_message_func):

        self.print_message_func = print_message_func

        self.file_name = file_name
        self.test_names = []
        self.pf_tests = []

        with open(file_name, newline='') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                self.test_names.append(row['test_name'])
                pf_test = PfTest(row['test_name'],
                                 float(row['start_x']),
                                 float(row['start_y']),
                                 float(row['start_width']),
                                 float(row['start_length']),
                                 float(row['start_rotation']),
                                 float(row['orientation_center']),
                                 float(row['orientation_range']),
                                 row['data_file_name'],
                                 float(row['start_time']))

                self.pf_tests.append(pf_test)

        self.num_tests = len(self.test_names)
        self.current_test = 0

        self.num_trials_per_location = 1

    def process_results(self, save_path=None):
        completed_tests = []
        for pf_test in self.pf_tests:
            if pf_test.test_completed:
                completed_tests.append(pf_test)

        avg_convergence_rates = np.zeros(len(completed_tests))
        avg_times_all = np.zeros(len(completed_tests))
        avg_times_converged = np.zeros(len(completed_tests))

        for i in range(len(completed_tests)):
            convergence_rate, avg_time_all, avg_time_converged = completed_tests[i].get_results()
            avg_convergence_rates[i] = convergence_rate
            avg_times_all[i] = avg_time_all
            avg_times_converged[i] = avg_time_converged

        # Calculate the overall average time for trials that converged, removing nan values
        overall_avg_time_converged = np.nanmean(avg_times_converged)

        # Calculate the overall average convergence rate
        overall_avg_convergence_rate = np.mean(avg_convergence_rates)

        # Save the results to a csv file
        if save_path is not None:
            if not save_path.endswith(".csv"):
                save_path = save_path + ".csv"

            # check if the file already exists, if so, have popup to ask if they want to overwrite
            if os.path.exists(save_path):
                msg_box = QMessageBox()
                msg_box.setIcon(QMessageBox.Warning)
                msg_box.setText("File already exists")
                msg_box.setInformativeText("Do you want to overwrite the file?")
                msg_box.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
                msg_box.setDefaultButton(QMessageBox.No)
                ret = msg_box.exec_()
                if ret == QMessageBox.No:
                    return

            with open(save_path, "w") as f:
                writer = csv.writer(f)
                writer.writerow(["Start Location", "Convergence Rate", "Average Time", "Average Time Converged"])
                for i in range(len(completed_tests)):
                    writer.writerow([i, avg_convergence_rates[i], avg_times_all[i], avg_times_converged[i]])

                writer.writerow(["Overall Average Time Converged", overall_avg_time_converged])
                writer.writerow(["Overall Average Convergence Rate", overall_avg_convergence_rate])

        # Print the results to the console
        self.print_message_func("Results:")
        for i in range(len(completed_tests)):
            self.print_message_func("Start Location: {}   Convergence Rate: {}   Average Time: {}   Average Time Converged: {}".format(
                i, avg_convergence_rates[i], avg_times_all[i], avg_times_converged[i]
            ))
        self.print_message_func("Overall Average Time Converged: {}".format(overall_avg_time_converged))
        self.print_message_func("Overall Average Convergence Rate: {}".format(overall_avg_convergence_rate))

    def reset_tests(self):
        for pf_test in self.pf_tests:
            pf_test.reset_results()

    def reset_test(self, test_num):
        self.pf_tests[test_num].reset_results()



class PfTest:
    def __init__(self, test_name, start_x, start_y, start_width, start_length, start_rotation, orientation_center,
                 orientation_range, data_file_name, start_time):
        self.test_name = test_name
        self.start_x = start_x
        self.start_y = start_y
        self.start_width = start_width
        self.start_length = start_length
        self.start_rotation = start_rotation
        self.orientation_center = orientation_center
        self.orientation_range = orientation_range
        self.data_file_name = data_file_name
        self.start_time = start_time

        self.results_distances = []
        self.results_convergence_accuracy = []
        self.results_run_times = []

        self.test_completed = False

    def __repr__(self):
        return f"Test Name: {self.test_name}, Start X: {self.start_x}, Start Y: {self.start_y}, Start Width: {self.start_width}, Start Length: {self.start_length}, Start Rotation: {self.start_rotation}, Orientation Center: {self.orientation_center}, Orientation Range: {self.orientation_range}, Data File Name: {self.data_file_name}, Start Time: {self.start_time}"

    def reset_results(self):
        self.test_completed = False
        self.results_distances = []
        self.results_convergence_accuracy = []
        self.results_run_times = []

    def add_results(self, run_time, correct_convergence, distance_to_converge):
        self.results_distances.append(distance_to_converge)
        self.results_convergence_accuracy.append(correct_convergence)
        self.results_run_times.append(run_time)

    def get_results(self):
        convergences = np.array(self.results_convergence_accuracy)
        run_times = np.array(self.results_run_times)

        # Calculate the convergence rate
        convergence_rate = np.sum(convergences) / len(convergences)

        # Calculate the average time to convergence
        avg_time_all = np.sum(run_times) / len(run_times)

        # Calculate the average time for trials that converged
        avg_time_converged = np.sum(run_times * convergences) / np.sum(convergences)

        return convergence_rate, avg_time_all, avg_time_converged

    def set_completed(self):
        self.test_completed = True








