from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QCheckBox, QLabel, QLineEdit, QPushButton, QFileDialog, QMessageBox
from PyQt5.QtCore import pyqtSlot
import json
import os
import cv2
from typing import TYPE_CHECKING, List

from pf_orchard_localization.data_managers import data_msgs

if TYPE_CHECKING:
    from pf_orchard_localization import app_managers

import logging
logger = logging.getLogger(__name__)


class CachedDataCreator(QWidget):
    """Widget for creating and managing cached data in the application.
    """
    def __init__(self, main_app_manager: 'app_managers.RosBags'):
        """Initializes the data caching widget.
        
        Args:
            main_app_manager (app_managers.RosBags): Main application manager instance
        """
        super().__init__()

        self.main_app_manager = main_app_manager

        self.cache_data_enabled = False
        self.cache: List[data_msgs.AnyDataMsg] = []

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
        """Opens a directory selection dialog to set the save location for cached data.
        """
        save_location = QFileDialog.getExistingDirectory(self, "Select Save Location") + "/"
        self.save_directory_input.setText(save_location)
        # check for "images" folder, if it doesn't exist, create it

    @pyqtSlot()
    def reset_cache(self):
        """Clears all cached data and resets the cache size display.
        """
        self.cache = []
        self.cache_size_label.setText("Cache Size: 0 messages")
    
    @pyqtSlot(object)
    def cache_data(self, data_msg: data_msgs.AnyDataMsg):
        """Adds received data to the cache if caching is enabled.
        
        Also handles storing images if the image saving option is enabled.
        
        Args:
            data_msg (data_msgs.AnyDataMsg): Data message to cache
        """
        if not self.cache_data_enabled:
            logger.warning("Cache data is not enabled")
            return
            
        if self.save_images_checkbox.isChecked() and data_msg.message_type == data_msgs.MsgType.IMAGE:
            self.save_image(data_msg)

        self.cache.append(data_msg.to_cache_dict())
        self.cache_size_label.setText("Cache Size: " + str(len(self.cache)) + " messages")

    @pyqtSlot()
    def save_cache(self):
        """Saves cached data to a JSON file.
        
        Performs validation checks on save location and filename, and handles file overwrite confirmation.
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
            json.dump(self.cache, f, indent=4)

        self.main_app_manager.print_message("Cache saved to: " + save_location)

    def save_image(self, img_msg: data_msgs.Image):
        """Saves an image from a data message to the selected directory.
        
        Creates an 'images' subdirectory if needed and names files using message timestamps.

        Args:
            img_msg (data_msgs.Image): Image message containing image data to save
        """

        save_directory = self.save_directory_input.text()

        if not os.path.exists(save_directory):
            logger.warning("Save directory does not exist, image not saved")

            return

        save_directory = os.path.join(save_directory, "images")

        if not os.path.exists(save_directory):
            os.makedirs(save_directory)

        save_location = os.path.join(save_directory, str(img_msg.bag_timestamp) + ".png")

        if img_msg.segmented_image is not None:
            img = img_msg.segmented_image
        else:
            img = img_msg.rgb_image

        cv2.imwrite(save_location, img)

    @pyqtSlot()
    def cache_data_checkbox_changed(self):
        """Handles state changes for the cache enabling checkbox.
        
        Enables or disables the caching functionality and UI controls.
        """
        if self.enable_checkbox.isChecked():
            self.cache_data_enabled = True
            self.set_input_disabled(False)
        else:
            self.cache_data_enabled = False
            self.reset_cache()
            self.set_input_disabled(True)

    def set_input_disabled(self, disabled):
        """Enables or disables all UI controls related to caching.

        Args:
            disabled (bool): True to disable all inputs, False to enable them
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
