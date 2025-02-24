from PyQt5.QtWidgets import (QLabel, QPushButton, QVBoxLayout, QHBoxLayout, QWidget, QLineEdit, QCheckBox,
                             QPlainTextEdit, QComboBox, QFileDialog, QDialog, QSlider, QSpinBox)
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot
import math
import cv2
import numpy as np
from . import PfSettingsDialog
import time
import logging
import json
import os
import copy
from map_data_tools import map_data 
from pf_orchard_localization.data_managers import data_msgs

from typing import TYPE_CHECKING, List, Dict, Tuple
if TYPE_CHECKING:
    from pf_orchard_localization import app_managers, app_modes

logger = logging.getLogger(__name__)

class PfChangeParametersButton(QWidget):
    """
    Widget for changing the particle filter parameters
    """

    def __init__(self, main_app_manager: 'app_managers.AnyManager'):
        super().__init__()

        self.main_app_manager = main_app_manager

        self.adjust_pf_settings_button = QPushButton("Change Particle Filter Parameters")
        self.adjust_pf_settings_button.setToolTip("Adjust the particle filter parameters")
        self.adjust_pf_settings_button.setFixedWidth(300)

        self.layout = QHBoxLayout()

        self.layout.addWidget(self.adjust_pf_settings_button)

        self.setLayout(self.layout)

        self.adjust_pf_settings_button.clicked.connect(self.adjust_pf_settings)

    def adjust_pf_settings(self):
        """
        Adjust the particle filter settings
        """
            
        pf_active = self.main_app_manager.get_pf_active()

        if pf_active:
            self.main_app_manager.print_message("Cannot adjust settings while PF is running")
            return

        parameters_pf = self.main_app_manager.get_pf_parameters()

        settings_dialog = PfSettingsDialog(current_settings=parameters_pf)

        # if settings_dialog.exec() == QDialog.DialogCode.Accepted: # Qt6
        if settings_dialog.exec() == QDialog.Accepted:
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


class PfControlButtons(QWidget):
    """
    Widget to setup the buttons for controlling the particle filter
    """

    startButtonClicked = pyqtSignal()
    stopButtonClicked = pyqtSignal()

    def __init__(self, main_app_manager):
        super().__init__()

        self.main_app_manager = main_app_manager

        self.reset_button = QPushButton("Reset PF")
        self.reset_button.setToolTip("Reset the particle filter")

        self.start_stop_button = QPushButton("Start")
        self.start_stop_button.setToolTip("Start the particle filter")

        self.single_step_button = QPushButton("Take Step")
        self.single_step_button.setToolTip("Continue the next step in the particle filter")

        self.center_on_gps_button = QPushButton("Center on GPS")
        self.center_on_gps_button.setToolTip("Center the particles on the GPS location")

        self.top_layer_layout = QHBoxLayout()
        self.top_layer_layout.addWidget(self.start_stop_button)
        self.top_layer_layout.addWidget(self.single_step_button)
        self.top_layer_layout.addWidget(self.reset_button)
        self.top_layer_layout.addWidget(self.center_on_gps_button)

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
        """
        Set the number of particles label
        
        Args:
            num_particles (int): Number of particles
        """
        self.num_particles_label.setText(str(num_particles))
    
    @pyqtSlot()
    def start_stop_button_clicked(self):
        """
        Slot for when the start/stop button is clicked
        """
        if self.start_stop_button.text() == "Start":
            self.startButtonClicked.emit()
        elif self.start_stop_button.text() == "Stop":
            self.stopButtonClicked.emit()

    def set_start(self):
        """
        Set the button to 'Start'
        """
        self.start_stop_button.setText("Start")
        self.start_stop_button.setToolTip("Start the particle filter")

    def set_stop(self):
        """
        Set the button to 'Stop'
        """
        self.start_stop_button.setText("Stop")
        self.start_stop_button.setToolTip("Stop the particle filter")


class PfStartLocationControls(QWidget):
    """
    Widget for setting the starting state of the particles
    """

    def __init__(self, main_app_manager: 'app_managers.AnyManager'):
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

        self.gps_x = None
        self.gps_y = None

    def set_parameters(self):
        """
        Set the parameters from the main app manager
        """
        pf_settings = self.main_app_manager.get_pf_parameters()
        self.start_x_input.setText(str(pf_settings.start_pose_center_x))
        self.start_y_input.setText(str(pf_settings.start_pose_center_y))
        self.rotation_input.setText(str(pf_settings.start_rotation))
        self.orientation_center_input.setText(str(pf_settings.start_orientation_center))
        self.orientation_range_input.setText(str(pf_settings.start_orientation_range))
        self.start_width_input.setText(str(pf_settings.start_width))
        self.start_height_input.setText(str(pf_settings.start_height))

    def get_parameters(self):
        """
        Get the parameters from the GUI and (try to) set the main app manager parameters
        """
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

    @pyqtSlot(dict)
    def set_gps_position(self, gps_data: data_msgs.Gnss):
        """
        Set the GPS position from the GPS data
        """
        self.gps_x = gps_data.map_x
        self.gps_y = gps_data.map_y

    def set_start_location_from_gps(self):
        """
        Set the start center location to the GPS location
        """

        if self.gps_x is None or self.gps_y is None:
            self.main_app_manager.print_message("No GPS data available")
            return
        
        # Check if the GPS data is out of range, TODO: make this a setting or somehow automated
        if -25 < self.gps_x > 100 or -25 < self.gps_y > 180:
            self.main_app_manager.print_message("GPS data appears out of range")
            return
        
        self.set_start_location_from_plot_click(self.gps_x, self.gps_y, True)

    def set_start_location_from_plot_click(self, x, y, shift_pressed):
        """
        Set the start location from a plot click if shift is pressed

        Args:
            x (float): x coordinate of the click
            y (float): y coordinate of the click
            shift_pressed (bool): True if shift is pressed
        """
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


    def setReadOnly(self, read_only=True):
        """
        Set the widget to read only mode

        Args:
            read_only (bool): True to set the widget to read only, False to set it to read/write
        """
        self.start_x_input.setReadOnly(read_only)
        self.start_y_input.setReadOnly(read_only)
        self.rotation_input.setReadOnly(read_only)
        self.orientation_center_input.setReadOnly(read_only)
        self.orientation_range_input.setReadOnly(read_only)
        self.start_width_input.setReadOnly(read_only)
        self.start_height_input.setReadOnly(read_only)


class PfCheckBoxes(QWidget):
    """
    Widget for setting the checkboxes for the particle filter app
    """

    def __init__(self):
        super().__init__()
        self.all_checkbox_info = []
        self.num_boxes_per_row = 3

    def init_checkboxes(self):
        """
        Initialize the checkboxes in the all_checkbox_info list, these must be added externally
        """

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


class Console(QWidget):
    """
    Console widget for displaying messages in the pf app
    """
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
        """
        Override the call method to print a message
        """
        self.print_message(message)

    def print_message(self, message):
        """
        Print a message to the console by appending it to the end

        Args:
            message (str): Message to print
        """
        self.console.appendPlainText(message)


class ImageLabel(QLabel):
    """
    Label for displaying images
    """
    def __init__(self, image_size=(480, 640), scale_factor=1.5):
        
        self.image_height = int(image_size[0] * scale_factor)
        self.image_width = int(image_size[1] * scale_factor)
        
        super().__init__()

        self.resize(self.image_height, self.image_width)
        # self.setAlignment(Qt.AlignmentFlag.AlignCenter) # Qt6
        self.setAlignment(Qt.AlignCenter)

        self.load_image(None)

    def load_image(self, img: np.ndarray):
        """
        Load an image into the GUI image viewer
        
        Args:
            img (np.array): Image to load
            img_num (int): Position to load the image into
        """
        # If image is none make a blank image
        if img is None:
            img = np.ones((self.image_height, self.image_width, 3), dtype=np.uint8) * 155

        # Convert the image to a Qt image and display it
        image_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        # image_qt = QImage(image_rgb.data, image_rgb.shape[1], image_rgb.shape[0], QImage.Format.Format_RGB888) # Qt6
        image_qt = QImage(image_rgb.data, image_rgb.shape[1], image_rgb.shape[0], QImage.Format_RGB888)
        
        pixmap = QPixmap.fromImage(image_qt)

        # pixmap_scaled = pixmap.scaled(self.picture_labels[img_num].size(), Qt.AspectRatioMode.KeepAspectRatio) # Qt6
        pixmap_scaled = pixmap.scaled(self.size(), Qt.KeepAspectRatio)
        
        self.setPixmap(pixmap_scaled)

        # QApplication.processEvents()


class ImageDisplay(QWidget):
    """
    Widget for displaying images
    """

    imageDisplayChangeSignal = pyqtSignal(dict)

    def __init__(self, image_size=(480, 640), scale_factor=1.5):
        """
        Initialize the image display widget
        
        Args:
            image_size (tuple): Size of the image to display
            scale_factor (float): Scale factor to apply to the image
            """
        super().__init__()

        self.image_height = int(image_size[0] * scale_factor)
        self.image_width = int(image_size[1] * scale_factor)

        self.checkbox_states: dict = {}

        self.main_layout = QVBoxLayout()

        self.checkboxes_layout = QHBoxLayout()
        self.image_checkboxes: List[QCheckBox] = []
        
        self.checkboxes_layout.addWidget(QLabel("Extra Images to Show: "))
        self.original_image_checkbox = QCheckBox("Original", checked=False)
        self.unfiltered_image_checkbox = QCheckBox("Unfiltered", checked=False)
        self.depth_image_checkbox = QCheckBox("Depth", checked=True)
        
        self.image_checkboxes.append(self.original_image_checkbox)
        self.image_checkboxes.append(self.unfiltered_image_checkbox)
        self.image_checkboxes.append(self.depth_image_checkbox)

        for checkbox in self.image_checkboxes:
            self.checkboxes_layout.addWidget(checkbox)
            checkbox.stateChanged.connect(self.checkbox_changed)
        self.checkboxes_layout.addStretch(1)

        self.picture_layout = QHBoxLayout()
        self.picture_labels: list[QLabel] = []

        self.original_image_label = ImageLabel(image_size=image_size, scale_factor=scale_factor)
        self.picture_labels.append(self.original_image_label)
        self.unfiltered_segmented_image_label = ImageLabel(image_size=image_size, scale_factor=scale_factor)
        self.picture_labels.append(self.unfiltered_segmented_image_label)
        self.segmented_image_display = ImageLabel(image_size=image_size, scale_factor=scale_factor)
        self.picture_labels.append(self.segmented_image_display)
        self.depth_image_display = ImageLabel(image_size=image_size, scale_factor=scale_factor)
        self.picture_labels.append(self.depth_image_display)
        
        for picture_label in self.picture_labels:
            self.picture_layout.addWidget(picture_label)

        self.main_layout.addLayout(self.checkboxes_layout)
        self.main_layout.addLayout(self.picture_layout)

        self.setLayout(self.main_layout)

    def checkbox_changed(self):
        """
        Slot for when the checkboxes are changed
        """
        self.original_image_label.hide()
        self.unfiltered_segmented_image_label.hide()
        self.segmented_image_display.hide()
        
        self.checkbox_states = {}

        if self.original_image_checkbox.isChecked():
            self.original_image_label.show()
            self.checkbox_states['original'] = True
        if self.unfiltered_image_checkbox.isChecked():
            self.unfiltered_segmented_image_label.show()
            self.checkbox_states['unfiltered'] = True
        if self.depth_image_checkbox.isChecked():
            self.segmented_image_display.show()
            self.checkbox_states['depth'] = True
        
        self.imageDisplayChangeSignal.emit(self.checkbox_states)
    
    @pyqtSlot(data_msgs.Image)
    def set_images(self, image_msg_data: data_msgs.Image):
        """
        Set the images to display
        
        Args:
            image_msg_data (data_msgs.Image): Image data to display
        """
        if image_msg_data.rgb_image is not None:
            self.original_image_label.load_image(image_msg_data.rgb_image)
        if image_msg_data.unfiltered_segmented_image is not None:
            self.unfiltered_segmented_image_label.load_image(image_msg_data.unfiltered_segmented_image)
        if image_msg_data.segmented_image is not None:
            self.segmented_image_display.load_image(image_msg_data.segmented_image)
        if image_msg_data.visualized_depth_image is not None:
            self.depth_image_display.load_image(image_msg_data.visualized_depth_image)


class PfModeSelector(QWidget):
    """
    Widget for selecting the mode to run
    """
    def __init__(self, mode_options=()):
        """
        Initialize the mode selector widget

        Args:
            mode_options (list): List of mode options to display
        """

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

    def set_modes(self, modes: List['app_modes.AnyMode']):
        """
        Set the modes in the mode selector

        Args:
            modes (list): List of modes to set
        """

        self.mode_selector.blockSignals(True)
        
        self.mode_selector.clear()
        for mode in modes:
            self.mode_selector.addItem(mode.mode_name)

        self.mode_selector.blockSignals(False)

    @property
    def mode(self):
        """
        Get the current mode selected

        Returns:
            str: The current mode selected
        """
        return self.mode_selector.currentText()


class ImageBrowsingControls(QWidget):
    """
    Widget for controlling the image browsing in the playback mode
    """

    playButtonClicked = pyqtSignal()
    stopButtonClicked = pyqtSignal()

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

        self.play_fwd_button.clicked.connect(self.play_button_clicked)
        self.save_location_change_button.clicked.connect(self.change_save_location)

    def set_playing(self, playing: bool):
        """
        Set the playing state of the widget, Stop if playing, Play if not
        
        Args:
            playing (bool): True if playing, False if not
        """
        if playing:
            self.play_fwd_button.setText("Stop")
        else:
            self.play_fwd_button.setText("Play")
        self.previous_button.setDisabled(playing)
        self.next_button.setDisabled(playing)
        self.save_button.setDisabled(playing)
    
    @pyqtSlot()
    def play_button_clicked(self):
        """
        Slot for when the play button is clicked. If the button is play, emit the play signal, if it's stop, emit the stop signal
        """
        if self.play_fwd_button.text() == "Play":
            self.playButtonClicked.emit()
        else:
            self.stopButtonClicked.emit()

    def change_save_location(self):
        """
        Change the save location for the images
        """
        save_location = QFileDialog.getExistingDirectory(self, "Select Save Location") + "/"
        self.save_location_input.setText(save_location)

    @property
    def save_location(self):
        """
        Get the save location

        Returns:
            str: The save location
        """
        return self.save_location_input.text()


class ImageNumberLabel(QWidget):
    """
    Widget for displaying the image number when using recorded data
    """

    def __init__(self):
        super().__init__()

        self.img_number_label = QLabel(self)
        # self.img_number_label.setAlignment(Qt.AlignmentFlag.AlignCenter) # Qt6
        self.img_number_label.setAlignment(Qt.AlignCenter)
        

        self.img_number_layout = QVBoxLayout()
        self.img_number_layout.addWidget(self.img_number_label)

        self.setLayout(self.img_number_layout)

    @pyqtSlot(int, int)
    def set_img_number_label(self, img_number, total_imgs):
        """
        Set the image number label

        Args:
            img_number (int): The current image number
            total_imgs (int): The total number of images
        """
        self.img_number_label.setText("Image " + str(img_number) + " of " + str(total_imgs))


class TimeMultiplierSlider(QWidget):
    """
    Widget for setting the added delay between images when using recorded data
    """

    def __init__(self):
        super().__init__()

        label = QLabel("Frame Delay Multiplier:")

        initial_value = 100

        # self.slider = QSlider(Qt.Orientation.Horizontal) # Qt6
        self.slider = QSlider(Qt.Horizontal) 
        self.slider.setMinimum(0)
        self.slider.setMaximum(400)
        self.slider.setValue(initial_value)
        self.slider.setTickInterval(10)
        # self.slider.setTickPosition(QSlider.TickPosition.NoTicks) # Qt6
        self.slider.setTickPosition(QSlider.NoTicks)        

        # divide by 100 and round to 2 decimal places
        time_label = f"{self.slider.value()/ 100:.2f}x"
        self.value_label = QLabel(time_label)

        label.setFixedWidth(170)
        self.slider.setFixedWidth(150)
        self.value_label.setFixedWidth(60)
        self.setFixedWidth(410)

        self.slider_layout = QHBoxLayout()
        self.slider_layout.addWidget(label)
        self.slider_layout.addWidget(self.slider)
        self.slider_layout.addWidget(self.value_label)
        self.setLayout(self.slider_layout)

        self.slider.valueChanged.connect(self.update_value_label)

    def update_value_label(self):
        """
        Update the value label to the current value of the slider
        """
        time_label = f"{self.slider.value()/ 100:.2f}x"
        self.value_label.setText(time_label)
        

    def get_multiplier_value(self):
        """
        Get the delay in ms

        Returns:
            int: The delay in ms
        """
        return self.slider.value()/100

class PfQueueSizeLabel(QWidget):
    """
    Widget for displaying the size of the image queue
    """

    def __init__(self):
        super().__init__()

        self.queue_size_label = QLabel(self)
        self.queue_size_label.setToolTip("Number of image messages in the queue")

        self.queue_size_layout = QVBoxLayout()
        self.queue_size_layout.addWidget(self.queue_size_label)

        self.setLayout(self.queue_size_layout)

    def set_queue_size(self, queue_size):
        """
        Set the queue size label

        Args:
            queue_size (int): The size of the queue
        """
        self.queue_size_label.setText("Queue Size: " + str(queue_size))




class CalibrationDataControls(QWidget):
    """
    Widget for creating/saving calibration data. Where the 'calibration data' is an rgb image, depth image, and ground
    truth width and x position in the image of a tree. Therefore only images of trees with a ground truth width (which
    is stored in the map data) can be saved as calibration data. This data can then be used to tune the trunk width
    estimation algorithm.
    """

    def __init__(self, main_app_manager: "app_managers.RosBags"):
        """
        Initialize the widget

        Args:
            main_app_manager (PfMainAppManager): Main app manager
        """
        super().__init__()
        self.main_app_manager = main_app_manager

        self.save_data_checkbox = QCheckBox("Save Data")
        self.save_data_checkbox.setToolTip("Save the current data as calibration data")

        self.save_location_label = QLabel("Save Location:")
        self.save_location_label.setToolTip("Location to save the calibration data")
        self.save_location_input = QLineEdit()
        self.save_location_input.setToolTip("Location to save the calibration data")
        self.save_location_input.setPlaceholderText("Save Location")
        self.change_save_location_button = QPushButton("Change")
        self.change_save_location_button.setToolTip("Change the location to save the calibration data")

        self.data_note_label = QLabel("Data Note:")
        self.data_note_label.setToolTip("Note to save with the data")
        self.data_note_input = QLineEdit()
        self.data_note_input.setPlaceholderText("e.g. from september 2021 jazz")
        self.data_note_input.setToolTip("Note to save with the data")
        # self.data_note_input.setFixedWidth(450)

        self.ground_truth_date_label = QLabel("Ground Truth Date:")
        self.ground_truth_date = None
        self.date_edit = QSpinBox()
        self.date_edit.setRange(2000, 2100)
        self.date_edit.setValue(2021)
        self.date_edit.setToolTip("Ground truth date of the data")
        self.date_edit.setFixedWidth(150)


        self.layout_1 = QHBoxLayout()
        self.layout_2 = QHBoxLayout()

        self.layout_1.addWidget(self.save_data_checkbox)
        self.layout_1.addWidget(self.save_location_label)
        self.layout_1.addWidget(self.save_location_input)
        self.layout_1.addWidget(self.change_save_location_button)

        self.layout_2.addWidget(self.data_note_label)
        self.layout_2.addWidget(self.data_note_input)

        self.layout_2.addWidget(self.ground_truth_date_label)
        self.layout_2.addWidget(self.date_edit)

        self.main_layout = QVBoxLayout()

        self.main_layout.addLayout(self.layout_1)
        self.main_layout.addLayout(self.layout_2)

        self.setLayout(self.main_layout)

        self.change_save_location_button.clicked.connect(self.change_save_location)
        self.save_data_checkbox.stateChanged.connect(self.save_data_checkbox_changed)
        
        self.previous_x_position_in_image = None
        
    @pyqtSlot(data_msgs.Image)
    def save_data(self, image_msg_data: data_msgs.Image):
        """
        This function will be called if the save calibration mode is active and the save data checkbox is checked. It first 
        check if a tree was seen, and if so it assumes the best particle is correct and finds the corresponding tree in the
        map data. It then saves the rgb image, depth image, and some data about the tree to the save location if there is 
        a ground truth width for the tree. The images are saved as the timestamp of the data, and the data is saved in a
        json file with the following fields:
        - tree_number: The tree number in the map data
        - tree_position: The position of the tree in lat, lon
        - note: A note about the data
        - ground_truth_width: The ground truth width of the tree
        - test_tree_number: The test tree number of the tree
        - ground_truth_date: The ground truth date of the data
        - x_position_in_image: The x position of the tree in the image
        - measured_width: The measured width of the tree  

        Args:
            image_msg_data (data_msgs.Image): Image message with the data to save
        """
        
        x_positions_in_image = image_msg_data.x_positions_in_image

        if image_msg_data.x_positions_in_image is None:
            self.main_app_manager.print_message("No trunk data available")
            return

        if self.save_data_checkbox.isChecked() == False:
            self.main_app_manager.print_message("Data not being saved")
            return

        closest_objects, kept_idx = self.find_closest_tree(image_msg_data)

        if len(closest_objects) == 0:
            self.main_app_manager.print_message("No tree detected")
            return

        # TODO: I could probably just change find closest tree to just return the closest, but it'd also be weird for this to happen so idk, maybe be good to catch it
        if len(closest_objects) > 1:
            self.main_app_manager.print_message("More than one tree detected ???")
            return

        tree_data = closest_objects[0]
        kept_idx = kept_idx[0]

        rgb_image = image_msg_data.rgb_image
        depth_image = image_msg_data.depth_image
        save_location = self.save_location_input.text()

        rgb_dir = save_location + "rgb/"
        depth_dir = save_location + "depth/"
        data_dir = save_location + "data/"

        if not os.path.exists(rgb_dir):
            os.makedirs(rgb_dir)
        if not os.path.exists(depth_dir):
            os.makedirs(depth_dir)
        if not os.path.exists(data_dir):
            os.makedirs(data_dir)

        if not os.path.exists(save_location):
            self.main_app_manager.print_message("Save location does not exist")
            return

        ground_truth_date = self.date_edit.value()

        file_name = str(image_msg_data.bag_timestamp) + ".png"

        data_note = self.data_note_input.text()

        tree_data.convert_to_lat_lon()

        x_position_in_image = x_positions_in_image[kept_idx]
        measured_width = self.main_app_manager.trunk_data_connection.widths[kept_idx]

        data_to_save = {"tree_number": tree_data.object_number,
                        "tree_position": tree_data.position_estimate,
                        "note": data_note,
                        "ground_truth_width": tree_data.ground_truth_width,
                        "test_tree_number": tree_data.test_tree_number,
                        "ground_truth_date": ground_truth_date,
                        "x_position_in_image": int(x_position_in_image),
                        "measured_width": measured_width,
                        }

        rgb_save_location = rgb_dir + file_name
        depth_save_location = depth_dir + file_name
        data_save_location = data_dir + file_name.replace(".png", ".json")

        cv2.imwrite(rgb_save_location, rgb_image)
        cv2.imwrite(depth_save_location, depth_image)
        with open(data_save_location, 'w') as f:
            json.dump(data_to_save, f, indent=4)

        self.main_app_manager.print_message("Data saved to: " + data_save_location)

    def find_closest_tree(self, image_msg_data: data_msgs.Image) -> Tuple[List[map_data.ObjectData], List[int]]:
        """
        Find the closest tree to the best particle and return the tree data and the index of the tree in the trunk data

        Returns:
            list: List of the closest tree data
            list: List of the index of the tree in the trunk data
        """
        best_particle = self.main_app_manager.pf_engine.best_particle

        # make the 3, array a 1,3 array
        best_particle = best_particle.reshape(1, -1)

        tree_global_coords = self.main_app_manager.pf_engine.get_object_global_locations(best_particle, image_msg_data.object_locations)

        closest_objects: List[map_data.ObjectData] = []
        kept_idx = []

        for i in range(len(tree_global_coords)):

            # if the object is a post, skip it
            if image_msg_data.object_classes[i] == 1:
                continue
            
            # get the closest object to the best particle
            distance, idx = self.main_app_manager.pf_engine.kd_tree.query(tree_global_coords[i, :, :])

            # if the distance is greater than 0.25m, skip it
            if distance > 0.25:
                continue
            
            # Make a copy of the tree's map data
            closest_object: map_data.ObjectData = copy.deepcopy(self.main_app_manager.map_data.map_data[idx[0]])

            # If the tree has no ground truth width, skip it
            if closest_object.ground_truth_width is None:
                continue
            
            closest_objects.append(closest_object)

            kept_idx.append(i)

        return closest_objects, kept_idx

    @pyqtSlot()
    def change_save_location(self):
        """
        Slot that opens a dialog to change the save location for the calibration data when the button is clicked
        """
        save_location = QFileDialog.getExistingDirectory(self, "Select Save Location") + "/"
        self.save_location_input.setText(save_location)

    def set_running(self, running):
        """
        Set the widget to running mode

        Args:
            running (bool): True to set the widget to running mode, False to set it to not running mode
        """
        self.save_data_checkbox.setDisabled(running)
        self.save_location_input.setDisabled(running)
        self.change_save_location_button.setDisabled(running)
        self.data_note_input.setDisabled(running)
        self.date_edit.setDisabled(running)
    
    def hide(self):
        """
        Extend the hide method to ensure the the save data checkbox is unchecked
        """ 
        self.save_data_checkbox.setChecked(False)
        super().hide()

    @pyqtSlot(int)
    def save_data_checkbox_changed(self, check_state):
        """
        Slot for when the save data checkbox is changed. Sets the main app manager to save calibration data if checked

        Args:
            check_state (int): The state of the checkbox
        """

        checked = self.save_data_checkbox.isChecked()
                    
        if checked:
            self.main_app_manager.trunk_data_connection.set_emitting_save_calibration_data(True)
        else:
            self.main_app_manager.trunk_data_connection.set_emitting_save_calibration_data(False)

