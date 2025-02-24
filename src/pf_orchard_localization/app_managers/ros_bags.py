from PyQt5.QtWidgets import QVBoxLayout, QHBoxLayout
from typing import Union, List, TYPE_CHECKING

from pf_orchard_localization import app_managers, app_modes, img_processing_srv
from pf_orchard_localization.custom_widgets import (DataFileControls, ImageBrowsingControls, ImageNumberLabel, TimeMultiplierSlider,
                                                    CachedDataCreator, CalibrationDataControls)
from pf_orchard_localization.utils.parameters import ParametersBagData

if TYPE_CHECKING:
    ...
    
import logging
logger = logging.getLogger(__name__)

class RosBags(app_managers.Base):
    """
    Application class for the particle filter localization app using pre-recorded ros2 bag files.
    """

    def __init__(self, config_file_path):
        super().__init__(config_file_path)

    def init_data_parameters(self):
        """Initialize the data parameters"""

        self.parameters_data = ParametersBagData()
    
    def init_loaded_data(self):
        """Initialize the loaded data"""

        self.data_manager = None
        self.data_file_controls.open_data_file(data_file_number=self.parameters_data.initial_data_file_index)
        self.data_file_controls.set_time_line(float(self.parameters_data.initial_data_time))
        self.data_file_controls.data_file_time_line_edited()

    def setup_trunk_data_connection(self):
        """Setup the object that connects the app to the trunk segmenter and analyzer"""

        self.trunk_data_connection = img_processing_srv.DirectPkgConnection()
        self.trunk_data_connection.start()

        self.data_file_controls.set_trunk_data_request_func(self.trunk_data_connection.handle_request)

    def init_widgets_unique(self):
        """Initialize the widgets unique to bag data app"""

        self.image_browsing_controls = ImageBrowsingControls()
        self.data_file_controls = DataFileControls(self.parameters_data)
        self.image_number_label = ImageNumberLabel()
        self.image_delay_slider = TimeMultiplierSlider()
        self.cached_data_creator = CachedDataCreator(self)
        self.save_calibration_data_controls = CalibrationDataControls(self)

        self.widget_list += [self.image_browsing_controls, self.data_file_controls,
                                self.image_number_label, self.image_delay_slider, self.cached_data_creator, self.save_calibration_data_controls]

    def draw_ui(self):
        """Draw the user interface"""

        mode_change_button_layout = QHBoxLayout()
        mode_change_button_layout.addWidget(self.mode_selector)
        mode_change_button_layout.addWidget(self.change_parameters_button)

        control_layout = QHBoxLayout()
        control_layout.addWidget(self.control_buttons)
        control_layout.addWidget(self.image_delay_slider)

        self.ui_layout = QVBoxLayout()
        self.ui_layout.addLayout(mode_change_button_layout)
        self.ui_layout.addWidget(self.checkboxes)
        self.ui_layout.addWidget(self.start_location_controls)
        self.ui_layout.addLayout(control_layout)
        self.ui_layout.addWidget(self.image_display)
        self.ui_layout.addWidget(self.image_number_label)
        self.ui_layout.addWidget(self.image_browsing_controls)
        self.ui_layout.addWidget(self.data_file_controls)
        self.ui_layout.addWidget(self.cached_data_creator)
        self.ui_layout.addWidget(self.save_calibration_data_controls)
        self.ui_layout.addWidget(self.console)
        
        self.main_layout.addLayout(self.ui_layout)
        self.main_layout.addWidget(self.plotter)
        
    def connect_slots_unique(self):
        """Connect the app functions unique to the bag data app"""
        self.data_file_controls.data_file_controls_message.connect(self.print_message)
        self.data_file_controls.reset_pf.connect(self.reset_pf)
        self.data_file_controls.set_img_number_label.connect(self.image_number_label.set_img_number_label)        

    def init_modes(self):
        """Initialize the different app modes for the bag data app"""

        self.pf_mode = app_modes.PfRecordedData(self)
        self.modes.append(self.pf_mode)
        self.playback_mode = app_modes.ImagePlayback(self)
        self.modes.append(self.playback_mode)
        self.get_calibration_data_mode = app_modes.PfSaveCalibrationData(self)
        self.modes.append(self.get_calibration_data_mode)
