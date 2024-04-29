from PyQt5.QtWidgets import (QLabel, QPushButton, QVBoxLayout, QHBoxLayout, QWidget, QApplication, QLineEdit, QCheckBox,
                             QPlainTextEdit, QMainWindow, QComboBox, QFileDialog, QInputDialog, QDialog, QSlider)
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import Qt, pyqtSignal
import math
import cv2
import numpy as np
from ..utils.parameters import ParametersPf
from . import PfSettingsDialog
import time
import logging

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


class PfControlButtons(QWidget):
    startStopButtonClicked = pyqtSignal(str)

    def __init__(self, main_app_manager):
        super().__init__()

        self.main_app_manager = main_app_manager

        # Button to reset the particle filter
        self.reset_button = QPushButton("Reset")
        self.reset_button.setToolTip("Reset the particle filter")

        # Button to trigger popup to adjust pf settings
        self.adjust_pf_settings_button = QPushButton("PF Parameters")
        self.adjust_pf_settings_button.setToolTip("Adjust the particle filter parameters")

        self.start_stop_button = QPushButton("Start")
        self.start_stop_button.setToolTip("Start the particle filter")


        self.top_layer_layout = QHBoxLayout()
        self.top_layer_layout.addWidget(self.reset_button)
        self.top_layer_layout.addWidget(self.adjust_pf_settings_button)

        self.num_particles_label = QLabel("0")
        self.num_particles_layout = QHBoxLayout()
        self.num_particles_layout.addWidget(QLabel("Current number of particles:"))
        self.num_particles_layout.addWidget(self.num_particles_label)
        self.num_particles_layout.addStretch(1)

        self.control_layout = QVBoxLayout()
        self.control_layout.addLayout(self.top_layer_layout)
        self.control_layout.addWidget(self.start_stop_button)
        self.control_layout.addLayout(self.num_particles_layout)

        self.setLayout(self.control_layout)

        self.adjust_pf_settings_button.clicked.connect(self.adjust_pf_settings)
        self.start_stop_button.clicked.connect(self.start_stop_button_clicked)

    def start_stop_button_clicked(self):
        command = self.start_stop_button.text()
        self.startStopButtonClicked.emit(command)

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
            self.main_app_manager.reset_app()
        else:
            self.main_app_manager.print_message("Update Failed")

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

    def set_continue(self):
        self.start_stop_button.setText("Continue")
        self.start_stop_button.setToolTip("Continue the next step in the particle filter")

class PfStartLocationControls(QWidget):
    def __init__(self, main_app_manager, starting_settings: ParametersPf):
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
        self.start_width_height_layout.addWidget(QLabel("Start Pose Width/Height:"))
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
            self.main_app_manager.reset_app()
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

    def init_checkboxes(self):
        num_checkboxes = len(self.all_checkbox_info)
        num_rows = math.ceil(num_checkboxes/4)

        self.checkbox_overall_layout = QVBoxLayout()

        for i in range(num_rows):
            checkboxes_info_sub_section = self.all_checkbox_info[i*4:(i+1)*4]
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
    def __init__(self, mode_options=("Scroll Images", "Continuous", "Manual")):
        super().__init__()

        logging.debug(f"Starting Mode Selector with options {mode_options}")

        self.mode_selector = QComboBox()
        for mode_option in mode_options:
            self.mode_selector.addItem(mode_option)
        self.mode_selector.setFixedWidth(300)

        mode_selector_layout = QHBoxLayout()

        mode_label = QLabel("Mode:")
        mode_selector_layout.addWidget(mode_label)
        mode_selector_layout.addWidget(self.mode_selector)
        mode_selector_layout.addStretch(1)

        self.setLayout(mode_selector_layout)

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
        self.next_button = QPushButton("Next")
        self.play_fwd_button = QPushButton("Play")
        self.save_button = QPushButton("Save Image")

        self.img_browsing_buttons_layout = QHBoxLayout()
        self.img_browsing_buttons_layout.addWidget(self.previous_button)
        self.img_browsing_buttons_layout.addWidget(self.next_button)
        self.img_browsing_buttons_layout.addWidget(self.play_fwd_button)
        self.img_browsing_buttons_layout.addWidget(self.save_button)

        self.setLayout(self.img_browsing_buttons_layout)

        self.play_fwd_button.clicked.connect(self.emit_play_button_clicked)

    def emit_play_button_clicked(self):
        command = self.play_fwd_button.text()
        self.playButtonClicked.emit(command)



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

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setMaximum(1000)
        self.slider.setValue(500)
        self.slider.setTickInterval(10)
        self.slider.setTickPosition(QSlider.NoTicks)

        self.value_label = QLabel("500ms")

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