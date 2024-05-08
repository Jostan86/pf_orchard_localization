from PyQt5.QtWidgets import QWidget, QComboBox, QHBoxLayout, QVBoxLayout, QLabel, QPushButton, QLineEdit
from PyQt5.QtCore import pyqtSignal
import os
from ..recorded_data_loaders import BagDataLoader, CachedDataLoader
import logging

class DataFileControls(QWidget):

    def __init__(self, main_app_manager, data_parameters):
        super().__init__()

        self.main_app_manager = main_app_manager


        if hasattr(data_parameters, 'initial_file_index'):
            self.initial_file_index = data_parameters.initial_bag_index
        else:
            self.initial_file_index = 0

        if hasattr(data_parameters, 'data_file_dir'):
            self.data_file_dir = data_parameters.data_file_dir
        else:
            raise ValueError("Data file directory not provided")

        self.data_file_names = None

        self.data_file_selector_label = QLabel("Current Data File:")
        self.data_file_selector_label.setToolTip("Select the data file to open")
        self.data_file_selector_label.setFixedWidth(150)

        self.data_file_selector = QComboBox()
        self.data_file_selector.setToolTip("Select the data file to open")

        self.data_file_open_button = QPushButton("Open Selected")
        self.data_file_open_button.setToolTip("Open the selected data file")
        self.data_file_open_button.setFixedWidth(150)

        self.data_file_open_next_button = QPushButton("Open Next")
        self.data_file_open_next_button.setToolTip("Open the next data file in the list")
        self.data_file_open_next_button.setFixedWidth(150)

        self.data_file_selector_layout = QHBoxLayout()
        self.data_file_selector_layout.addWidget(self.data_file_selector_label)
        self.data_file_selector_layout.addWidget(self.data_file_selector)
        self.data_file_selector_layout.addWidget(self.data_file_open_button)
        self.data_file_selector_layout.addWidget(self.data_file_open_next_button)

        self.setLayout(self.data_file_selector_layout)

        self.data_file_open_button.clicked.connect(self.trigger_open_data_file)
        self.data_file_open_next_button.clicked.connect(self.trigger_open_next_data_file)

        self.set_data_file_names()
    def trigger_open_data_file(self):
        self.main_app_manager.open_data_file(self.current_data_file_selection)

    def trigger_open_next_data_file(self):
        self.main_app_manager.load_next_data_file()

    def get_next_data_file_name(self, current_data_file_name):
        current_data_file_index = self.data_file_names.index(current_data_file_name)
        next_data_file_index = current_data_file_index + 1

        if next_data_file_index >= len(self.data_file_names):
            return None
        else:
            return self.data_file_names[next_data_file_index]

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
    def current_data_file_selection(self):
        return self.data_file_selector.currentText()

    def set_combo_box_to_current(self):
        current_data_file_name = self.main_app_manager.data_manager.current_data_file_name
        self.data_file_selector.setCurrentIndex(self.data_file_names.index(current_data_file_name))

    def set_opening(self):
        self.data_file_open_button.setText("---")
        self.data_file_open_next_button.setText("---")
        self.data_file_open_button.repaint()

    def set_opened(self, message=None):
        self.data_file_open_button.setText("Open")
        self.data_file_open_next_button.setText("Open Next")
        self.set_combo_box_to_current()
        if message is not None:
            self.main_app_manager.print_message(message)
        self.data_file_open_button.repaint()

    def disable(self):
        self.data_file_selector.setDisabled(True)
        self.data_file_open_button.setDisabled(True)
        self.data_file_open_next_button.setDisabled(True)

    def enable(self):
        self.data_file_selector.setDisabled(False)
        self.data_file_open_button.setDisabled(False)
        self.data_file_open_next_button.setDisabled(False)