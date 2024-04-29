from PyQt5.QtWidgets import QWidget, QComboBox, QHBoxLayout, QVBoxLayout, QLabel, QPushButton, QLineEdit
from PyQt5.QtCore import pyqtSignal
import os
from ..recorded_data_loaders import BagDataLoader, CachedDataLoader
import logging

class DataFileControls(QWidget):

    def __init__(self, main_app_manager, data_parameters):
        super().__init__()

        self.main_app_manager = main_app_manager

        #check if data parameters has depth_topic, rgb_topic, and odom_topic
        if hasattr(data_parameters, 'depth_topic'):
            self.depth_topic = data_parameters.depth_topic
        else:
            self.depth_topic = None

        if hasattr(data_parameters, 'rgb_topic'):
            self.rgb_topic = data_parameters.rgb_topic
        else:
            self.rgb_topic = None

        if hasattr(data_parameters, 'odom_topic'):
            self.odom_topic = data_parameters.odom_topic
        else:
            self.odom_topic = None

        if hasattr(data_parameters, 'initial_file_index'):
            self.initial_file_index = data_parameters.initial_bag_index
        else:
            self.initial_file_index = 0

        if hasattr(data_parameters, 'data_file_dir'):
            self.data_file_dir = data_parameters.data_file_dir
        else:
            raise ValueError("Data file directory not provided")

        self.data_file_names = None

        self.data_file_selector = QComboBox()
        self.data_file_selector_layout = QHBoxLayout()
        self.data_file_selector_label = QLabel("Current Data File:")
        self.data_file_selector_label.setFixedWidth(220)
        self.data_file_open_button = QPushButton("Open")
        self.data_file_open_button.setFixedWidth(120)
        self.data_file_selector_layout.addWidget(self.data_file_selector_label)
        self.data_file_selector_layout.addWidget(self.data_file_selector)
        self.data_file_selector_layout.addWidget(self.data_file_open_button)

        self.setLayout(self.data_file_selector_layout)

        # define new button signal that i'll emit when the button is clicked with the current data file name
        self.data_file_open_button.clicked.connect(self.trigger_open_data_file)

        self.set_data_file_names()



    def trigger_open_data_file(self):
        self.main_app_manager.setup_data_manager(self.current_data_file_name)

    def open_data_file(self, data_file_name=None):

        if data_file_name is None:
            data_file_name = self.current_data_file_name

        self.data_file_open_button.setText("---")

        self.data_file_open_button.repaint()

        data_file_path = self.data_file_dir + data_file_name

        if not os.path.isfile(data_file_path):
            self.data_file_open_button.setText("Open")
            self.main_app_manager.print_message("Invalid file name")
            return None

        if self.main_app_manager.using_cached_data and data_file_path.endswith(".json"):
            data_manager = CachedDataLoader(data_file_path)
        elif data_file_path.endswith(".bag"):
            data_manager = BagDataLoader(data_file_path, self.depth_topic, self.rgb_topic, self.odom_topic)
        else:
            self.data_file_open_button.setText("Open")
            self.main_app_manager.print_message("Invalid file type")
            return None

        self.data_file_open_button.setText("Open")

        if data_manager.num_img_msgs == 0:
            self.main_app_manager.print_message("No images found in data file, check topic names")
            return None

        self.main_app_manager.data_file_time_line.data_file_time_line.setText(str(data_manager.current_data_file_time_stamp))

        # Return data manager and list of messages
        self.main_app_manager.print_message("Opened bag file: " + data_file_name)
        self.main_app_manager.print_message("Number of Odom messages: " + str(data_manager.num_odom_msgs))
        self.main_app_manager.print_message("Number of images: " + str(data_manager.num_img_msgs))

        return data_manager

    def load_next_data_file(self):
        current_data_file_name = self.main_app_manager.data_manager.current_data_file_name

        if self.main_app_manager.using_cached_data:
            self.main_app_manager.print_message("Cannot load next data file when using cached data")
            return False

        current_data_file_index = self.data_file_names.index(current_data_file_name)
        next_data_file_index = current_data_file_index + 1

        if next_data_file_index >= len(self.data_file_names):
            self.main_app_manager.print_message("No more data files")
            return False

        new_data_manager = self.open_data_file(self.data_file_names[next_data_file_index])

        if new_data_manager is None:
            return False

        self.main_app_manager.data_manager = new_data_manager

        return True


    def set_data_file_names(self):

        self.data_file_names = os.listdir(self.data_file_dir)

        if self.main_app_manager.using_cached_data:
            self.data_file_names = [file_name for file_name in self.data_file_names if file_name.endswith(".json")]
        else:
            self.data_file_names = [file_name for file_name in self.data_file_names if file_name.endswith(".bag")]

        self.data_file_names.sort()

        self.data_file_selector.clear()
        for data_file_name in self.data_file_names:
            self.data_file_selector.addItem(data_file_name)

        self.data_file_selector.setCurrentIndex(self.initial_file_index)

        logging.debug(f"Found {len(self.data_file_names)} data files in {self.data_file_dir}")

    @property
    def current_data_file_name(self):
        return self.data_file_selector.currentText()

    def disable(self):
        self.data_file_selector.setDisabled(True)
        self.data_file_open_button.setDisabled(True)

    def enable(self):
        self.data_file_selector.setDisabled(False)
        self.data_file_open_button.setDisabled(False)