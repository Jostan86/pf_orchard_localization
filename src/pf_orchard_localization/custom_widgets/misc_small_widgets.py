from PyQt5.QtWidgets import (QLabel, QPushButton, QVBoxLayout, QHBoxLayout, QWidget, QApplication, QLineEdit, QCheckBox,
                             QPlainTextEdit, QMainWindow, QComboBox, QFileDialog, QInputDialog, QDialog, QSlider,
                             QListWidget, QMessageBox, QSpinBox)
from PyQt5.QtGui import QImage, QPixmap, QGuiApplication
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


class PfMainWindow(QMainWindow):
    """
    Main window for the particle filter application
    """

    def __init__(self):
        super().__init__()

    def init_window_display_settings(self):
        """Initializes the window display settings"""

        self.setGeometry(0, 0, 1700, 900)

        # # Access the primary screen
        # desktop = QGuiApplication.primaryScreen()
        # target_screen_number = 0

        # # Check the number of screens
        # screens = QGuiApplication.screens()
        # if target_screen_number < len(screens):
        #     target_screen = screens[target_screen_number]
        #     self.move(target_screen.geometry().left(), target_screen.geometry().top())
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
    """
    Widget for changing the particle filter parameters
    """

    def __init__(self, main_app_manager):
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
    def set_gps_position(self, gps_data):
        """
        Set the GPS position from the GPS data
        """
        self.gps_x = gps_data['easting']
        self.gps_y = gps_data['northing']

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

class ImageDisplay(QWidget):
    """
    Widget for displaying images
    """
    def __init__(self, num_camera_feeds=1, image_size=(480, 640), scale_factor=1.5):
        """
        Initialize the image display widget
        
        Args:
            num_camera_feeds (int): Number of camera feeds to display
            image_size (tuple): Size of the image to display
            scale_factor (float): Scale factor to apply to the image
            """
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
            # picture_label.setAlignment(Qt.AlignmentFlag.AlignCenter) # Qt6
            picture_label.setAlignment(Qt.AlignCenter)
            self.picture_labels.append(picture_label)
            self.picture_layout.addWidget(picture_label)
            self.load_image(img=None, img_num=i)

        self.main_layout.addLayout(self.picture_layout)
        self.setLayout(self.main_layout)

    def load_image(self, img=None, img_num=0):
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
        image_cv2 = img
        image_rgb = cv2.cvtColor(image_cv2, cv2.COLOR_BGR2RGB)
        # image_qt = QImage(image_rgb.data, image_rgb.shape[1], image_rgb.shape[0], QImage.Format.Format_RGB888) # Qt6
        image_qt = QImage(image_rgb.data, image_rgb.shape[1], image_rgb.shape[0], QImage.Format_RGB888)
        
        pixmap = QPixmap.fromImage(image_qt)

        # pixmap_scaled = pixmap.scaled(self.picture_labels[img_num].size(), Qt.AspectRatioMode.KeepAspectRatio) # Qt6
        pixmap_scaled = pixmap.scaled(self.picture_labels[img_num].size(), Qt.KeepAspectRatio)
        
        self.picture_labels[img_num].setPixmap(pixmap_scaled)

        QApplication.processEvents()


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

    def set_modes(self, modes):
        """
        Set the modes in the mode selector

        Args:
            modes (list): List of modes to set
        """

        self.mode_selector.clear()
        for mode in modes:
            self.mode_selector.addItem(mode.mode_name)

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


class ImageDelaySlider(QWidget):
    """
    Widget for setting the added delay between images when using recorded data
    """

    def __init__(self):
        super().__init__()

        label = QLabel("Delay Between Images:")

        start_value_ms = 0

        # self.slider = QSlider(Qt.Orientation.Horizontal) # Qt6
        self.slider = QSlider(Qt.Horizontal) 
        self.slider.setMinimum(0)
        self.slider.setMaximum(500)
        self.slider.setValue(start_value_ms)
        self.slider.setTickInterval(10)
        # self.slider.setTickPosition(QSlider.TickPosition.NoTicks) # Qt6
        self.slider.setTickPosition(QSlider.NoTicks)        

        self.value_label = QLabel(str(start_value_ms) + " ms")

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
        self.value_label.setText(str(self.slider.value()) + "ms")

    def get_delay_ms(self):
        """
        Get the delay in ms

        Returns:
            int: The delay in ms
        """
        return self.slider.value()

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


class CachedDataCreator(QWidget):
    """
    Widget to aid in cacheing data in the app 
    """
    def __init__(self, main_app_manager):
        """
        Initialize the widget
        
        Args:
            main_app_manager (PfMainAppManager): Main app manager
        """
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

    @pyqtSlot()
    def change_save_directory(self):
        """
        Slot for the changing the save directory for the cached data when the button is clicked
        """
        save_location = QFileDialog.getExistingDirectory(self, "Select Save Location") + "/"
        self.save_directory_input.setText(save_location)
        # check for "images" folder, if it doesn't exist, create it

    @pyqtSlot()
    def reset_cache(self):
        """
        Slot for resetting the cache when the button is clicked
        """
        self.cache = {}
        self.cache_size_label.setText("Cache Size: 0 messages")

    def get_timestamp_str(self, time_stamp):
        """
        Get the timestamp as a string

        Args:
            time_stamp (float): Time stamp
        
        Returns:
            str: The time stamp as a string
        """
        return str(time_stamp*1000).split(".")[0]
    
    @pyqtSlot(dict)
    def cache_data(self, msg):
        """
        Slot for caching data when it's received
        
        Args:
            msg (dict): The message to cache
        """
        if msg['topic'] == "odom":
            self.cache_odom_data(msg['x_odom'], msg['theta_odom'], msg['time_stamp'])
            
        elif msg['topic'] == "image":
            self.cache_tree_data(msg['positions'], msg['widths'], msg['class_estimates'], msg['location_estimate'], msg['time_stamp'])
            
            if self.save_images_checkbox.isChecked() and msg['image'] is not None:
                self.save_image(msg['image'], msg['time_stamp'])
                
    
    def cache_odom_data(self, x_odom, theta_odom, time_stamp_odom):
        """
        Cache the odometry data

        Args:
            x_odom (float): X position of the odometry
            theta_odom (float): Theta position of the odometry
            time_stamp_odom (float): Time stamp of the odometry
        """
        self.cache[self.get_timestamp_str(time_stamp_odom)] = {"x_odom": x_odom, "theta_odom": theta_odom, "time_stamp": time_stamp_odom}
        self.cache_size_label.setText("Cache Size: " + str(len(self.cache)) + " messages")

    def cache_tree_data(self, positions, widths, class_estimates, location_estimate, time_stamp):
        """
        Cache the tree data

        Args:
            positions (np.array): Positions of the trees
            widths (np.array): Widths of the trees
            class_estimates (np.array): Class estimates of the trees
            location_estimate (np.array): Location estimate of the trees
            time_stamp (float): Time stamp of the tree data
        """
        if positions is None:
            self.cache[self.get_timestamp_str(time_stamp)] = None
            return
        tree_data = {"positions": positions.tolist(), "widths": widths.tolist(), "classes": class_estimates.tolist()}
        location_estimate = {"x": location_estimate[0], "y": location_estimate[1], "theta": location_estimate[2]}
        self.cache[self.get_timestamp_str(time_stamp)] = {"tree_data": tree_data, "location_estimate": location_estimate}

    @pyqtSlot()
    def save_cache(self):
        """
        Slot for saving the cache when the button is clicked. Does some check to ensure the save location is valid
        """
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
            # msg_box.setStandardButtons(QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No) # Qt6
            # msg_box.setDefaultButton(QMessageBox.StandardButton.No)
            # ret = msg_box.exec()
            # if ret == QMessageBox.StandardButton.No:
                # return
            msg_box.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
            msg_box.setDefaultButton(QMessageBox.No)
            ret = msg_box.exec()
            if ret == QMessageBox.No:
                return

        with open(save_location, 'w') as f:
            json.dump(self.cache, f)

        self.main_app_manager.print_message("Cache saved to: " + save_location)

    def save_image(self, img, time_stamp):
        """
        Save an image to the save location

        Args:
            img (np.array): Image to save
            time_stamp (float): Time stamp of the image
        """
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

    @pyqtSlot()
    def cache_data_checkbox_changed(self):
        """
        Slot for when the cache data checkbox is changed
        """
        if self.enable_checkbox.isChecked():
            self.cache_data_enabled = True
            self.set_input_disabled(False)
        else:
            self.cache_data_enabled = False
            self.reset_cache()
            self.set_input_disabled(True)

    def set_input_disabled(self, disabled):
        """
        Set the input to disabled or not

        Args:
            disabled (bool): True to disable input, False to enable
        """
        self.save_images_checkbox.setDisabled(disabled)
        self.save_directory_input.setDisabled(disabled)
        self.change_save_directory_button.setDisabled(disabled)
        self.file_name_input.setDisabled(disabled)
        self.save_button.setDisabled(disabled)
        self.reset_cache_button.setDisabled(disabled)
        self.cache_size_label.setDisabled(disabled)
        self.file_name_label.setDisabled(disabled)
        self.save_label.setDisabled(disabled)

class CalibrationDataControls(QWidget):
    """
    Widget for creating/saving calibration data. Where the 'calibration data' is an rgb image, depth image, and ground
    truth width and x position in the image of a tree. Therefore only images of trees with a ground truth width (which
    is stored in the map data) can be saved as calibration data. This data can then be used to tune the trunk width
    estimation algorithm.
    """

    def __init__(self, main_app_manager):
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
        
    @pyqtSlot(dict)
    def save_data(self, current_msg):
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
            current_msg (dict): The current message
        """
        
        x_positions_in_image = current_msg["x_positions_in_image"]

        if x_positions_in_image is None:
            self.main_app_manager.print_message("No trunk data available")
            return

        if self.save_data_checkbox.isChecked() == False:
            self.main_app_manager.print_message("Data not being saved")
            return

        closest_objects, kept_idx = self.find_closest_tree()

        if len(closest_objects) == 0:
            self.main_app_manager.print_message("No tree detected")
            return

        if len(closest_objects) > 1:
            self.main_app_manager.print_message("More than one tree detected ???")
            return

        tree_data = closest_objects[0]
        kept_idx = kept_idx[0]

        rgb_image = current_msg["rgb_image"]
        depth_image = current_msg["depth_image"]
        time_stamp = current_msg["timestamp"]
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

        timestamp_secs = int(time_stamp)
        timestamp_ns = int((time_stamp - timestamp_secs) * 1e9)
        file_name = str(timestamp_secs) + "_" + str(timestamp_ns).zfill(9) + ".png"

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

    def find_closest_tree(self):
        """
        Find the closest tree to the best particle and return the tree data and the index of the tree in the trunk data

        Returns:
            list: List of the closest tree data
            list: List of the index of the tree in the trunk data
        """
        tree_positions = self.main_app_manager.trunk_data_connection.positions
        class_estimates = self.main_app_manager.trunk_data_connection.class_estimates
        best_particle = self.main_app_manager.pf_engine.best_particle

        # make the 3, array a 1,3 array
        best_particle = best_particle.reshape(1, -1)

        tree_global_coords = self.main_app_manager.pf_engine.get_object_global_locations(best_particle, tree_positions)

        closest_objects = []
        kept_idx = []

        for i in range(len(tree_global_coords)):

            if class_estimates[i] == 1:
                continue

            distance, idx = self.main_app_manager.pf_engine.kd_tree.query(tree_global_coords[i, :, :])

            if distance > 0.25:
                continue

            closest_object = copy.deepcopy(self.main_app_manager.map_data.map_data[idx[0]])

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







