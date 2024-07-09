from PyQt5.QtWidgets import QWidget, QComboBox, QHBoxLayout, QVBoxLayout, QLabel, QPushButton, QLineEdit
from PyQt5.QtCore import pyqtSignal, pyqtSlot
import os
import logging
from ..recorded_data_loaders import Bag2DataLoader, CachedDataLoader

class DataFileControls(QWidget):
    
    data_file_controls_message = pyqtSignal(str)
    data_file_loaded = pyqtSignal(bool, object)
    set_img_number_label = pyqtSignal(int, int)
    set_image_display = pyqtSignal(dict)
    reset_pf = pyqtSignal(bool)

    def __init__(self, data_parameters, using_cached_data=False):
        super().__init__()

        self.data_manager = None
        self.using_cached_data = using_cached_data
        self.data_parameters = data_parameters

        if hasattr(data_parameters, 'initial_file_index'):
            self.initial_file_index = data_parameters.initial_bag_index
        else:
            self.initial_file_index = 0

        if hasattr(data_parameters, 'data_file_dir'):
            self.data_file_dir = data_parameters.data_file_dir
        else:
            raise ValueError("Data file directory not given in config file")

        self.data_file_names = None
        
        self.data_file_time_line = QLineEdit()
        self.data_file_time_line.setFixedWidth(75)

        time_line_label = QLabel("Data Time:")
        time_line_label.setFixedWidth(75)

        self.data_file_selector_label = QLabel("Data File:")
        self.data_file_selector_label.setToolTip("Select the data file to open")
        self.data_file_selector_label.setFixedWidth(70)

        self.data_file_selector = QComboBox()
        self.data_file_selector.setToolTip("Select the data file to open")

        self.data_file_open_button = QPushButton("Open")
        self.data_file_open_button.setToolTip("Open the selected data file")
        self.data_file_open_button.setFixedWidth(85)

        self.data_file_open_next_button = QPushButton("Open Next")
        self.data_file_open_next_button.setToolTip("Open the next data file in the list")
        self.data_file_open_next_button.setFixedWidth(85)

        self.data_file_selector_layout = QHBoxLayout()
        self.data_file_selector_layout.addWidget(time_line_label)
        self.data_file_selector_layout.addWidget(self.data_file_time_line)
        self.data_file_selector_layout.addSpacing(15)
        self.data_file_selector_layout.addWidget(self.data_file_selector_label)
        self.data_file_selector_layout.addWidget(self.data_file_selector)
        self.data_file_selector_layout.addWidget(self.data_file_open_button)
        self.data_file_selector_layout.addWidget(self.data_file_open_next_button)

        self.setLayout(self.data_file_selector_layout)

        self.data_file_open_button.clicked.connect(self.trigger_open_data_file)
        self.data_file_open_next_button.clicked.connect(self.trigger_open_next_data_file)
        self.data_file_time_line.returnPressed.connect(self.data_file_time_line_edited)

        self.set_data_file_names()
    
    @pyqtSlot(float)
    def set_time_line(self, time_stamp: float):
        time_stamp = round(time_stamp, 2)
        self.data_file_time_line.setText(str(time_stamp))
        
    def trigger_open_data_file(self):
        self.open_data_file(self.current_data_file_selection)

    def trigger_open_next_data_file(self):
        self.load_next_data_file(True)

    
    def get_next_data_file_name(self):
        current_data_file_index = self.data_file_names.index(self.current_data_file_selection)
        next_data_file_index = current_data_file_index + 1

        if next_data_file_index >= len(self.data_file_names):
            return None
        else:
            return self.data_file_names[next_data_file_index]

    def set_data_file_names(self):

        self.data_file_names = os.listdir(self.data_file_dir)

        if self.using_cached_data:
            self.data_file_names = [file_name for file_name in self.data_file_names if file_name.endswith(".json")]

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
        current_data_file_name = self.data_manager.current_data_file_name
        self.data_file_selector.setCurrentIndex(self.data_file_names.index(current_data_file_name))
    
    def data_file_time_line_edited(self):
        """Change the time the data file is at based on a time entered in the time line edit box by the user"""

        time_stamp = self.data_file_time_line.text()

        # Check if the value entered is a number
        try:
            time_stamp = float(time_stamp)
        except ValueError:
            self.data_file_controls_message.emit("Invalid time stamp")
            return

        # Set the time stamp in the data manager and get the current message
        message, current_msg = self.data_manager.set_time_stamp(time_stamp)

        self.data_file_controls_message.emit(message)

        # Update the app with the message from the new time stamp and set the time line to match the exact time stamp of the message
        if current_msg is not None:
            self.set_image_display.emit({"current_msg": current_msg, "for_display_only": True})
            
            self.set_time_line(self.data_manager.current_data_file_time_stamp)
        
        self.reset_pf.emit(True)

        self.set_img_number_label.emit(self.data_manager.current_img_position, self.data_manager.num_img_msgs)
        
    @pyqtSlot(bool)
    def load_next_data_file(self, load_first_image=True):
        """Load the next data file in the list of data files.

        Args:
            load_first_image (bool): Whether to load the first image in the new data file
        """
        
        if self.using_cached_data:
            self.dispense_data_manager(success=False, message="Cannot load next data file when using cached data")
            return
        
        next_data_file_name = self.get_next_data_file_name()

        if next_data_file_name is None:
            self.dispense_data_manager(success=False, message="Reached the end of the data files")
            return

        self.open_data_file(next_data_file_name, load_first_image)
        
    def open_data_file(self, data_file_name: str = None, load_first_image: bool = True):
        """
        Open a data file using a data manager object

        Args:
            data_file_name (str): Name of the data file to open
            load_first_image (bool): Whether to load the first image in the data file
        """
        # If no data file name is given, use the current data file selection
        if data_file_name is None:
            data_file_name = self.current_data_file_selection

        self.set_opening()

        data_file_path = self.data_file_dir + data_file_name

        valid, message = self.check_data_file_is_valid(data_file_path)
        
        if not valid:
            self.dispense_data_manager(success=False, message=message)
            return
        
        if data_file_path.endswith(".json"):
            self.data_manager = CachedDataLoader(data_file_path)
        else:
            self.data_manager = Bag2DataLoader(data_file_path, self.data_parameters.depth_topic, self.data_parameters.rgb_topic, self.data_parameters.odom_topic)

        if self.data_manager.num_img_msgs == 0:
            self.dispense_data_manager(success=False, message="No images found in data file, check topic names")
            return

        self.set_time_line(self.data_manager.current_data_file_time_stamp)

        msg = ["Opened bag file: " + data_file_name,]
        msg.append("Number of Odom messages: " + str(self.data_manager.num_odom_msgs))
        msg.append("Number of images: " + str(self.data_manager.num_img_msgs))

        self.dispense_data_manager(success=True, message="\n".join(msg))

        if load_first_image:
            current_msg = self.data_manager.get_next_img_msg()
            self.set_image_display.emit({"current_msg": current_msg, "for_display_only": True})
            self.set_img_number_label.emit(self.data_manager.current_img_position, self.data_manager.num_img_msgs)
            
    
    def check_data_file_is_valid(self, data_file_path: str):
        """Check if a data file is valid

        Args:
            data_file_name (str): Name of the data file to check
        """
        if not os.path.isfile(data_file_path) and not os.path.isdir(data_file_path):
            return False, "Invalid file name"
        elif data_file_path.endswith(".json") and not self.using_cached_data:
            return False, "Invalid file type, not using cached data"
        elif data_file_path.endswith(".json") and self.using_cached_data:
            return True, "Valid file"
        elif os.path.isdir(data_file_path):
            files = os.listdir(data_file_path)
            if len(files) == 2:
                file_endings = [file.split(".")[-1] for file in files]
                if "yaml" in file_endings and "db3" in file_endings:
                    return True, "Valid file"
        else:
            return False, "Invalid file"
    
    def dispense_data_manager(self, success: bool, message: str = None):
        if message is not None:
            self.data_file_controls_message.emit(message)
            
        self.set_opened(set_to_current=success)
        
        if not success:
            self.data_manager = None
            self.data_file_loaded.emit(False, None)
        else:
            self.data_file_loaded.emit(True, self.data_manager)

    def set_opening(self):
        self.data_file_open_button.setText("---")
        self.data_file_open_next_button.setText("---")
        self.data_file_open_button.repaint()

    def set_opened(self, set_to_current=True):
        self.data_file_open_button.setText("Open")
        self.data_file_open_next_button.setText("Open Next")
        if set_to_current:
            self.set_combo_box_to_current()