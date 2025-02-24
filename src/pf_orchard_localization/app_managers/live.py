from PyQt5.QtWidgets import QVBoxLayout, QHBoxLayout, QWidget
from typing import Union, List, TYPE_CHECKING

from pf_orchard_localization import app_managers, app_modes, img_processing_srv
from pf_orchard_localization.custom_widgets import PfQueueSizeLabel
from pf_orchard_localization.utils.parameters import ParametersLiveData

if TYPE_CHECKING:
    ...
    
import logging
logger = logging.getLogger(__name__)

class Live(app_managers.Base):
    """
    Application class for the particle filter localization app using live data from the trunk segmenter and analyzer.
    """

    def __init__(self, config_file_path):
        super().__init__(config_file_path)

    def init_data_parameters(self):
        """
        Initialize the data parameters
        """
        self.parameters_data = ParametersLiveData()

    def setup_trunk_data_connection(self):
        """
        Setup the object that connects the app to the trunk segmenter and analyzer
        """
        # self.trunk_data_connection = TrunkDataConnectionRosService()
        # self.trunk_data_connection.start()
        self.trunk_data_connection = img_processing_srv.Ros2Sub(self.parameters_data)
        self.image_display_checkbox_changed(init=True)
        self.trunk_data_connection.start()
    
    def init_widgets_unique(self):
        """
        Initialize the widgets unique to live data app
        """
        self.queue_size_label = PfQueueSizeLabel()

        self.widget_list += [self.queue_size_label]
    
    def draw_ui(self):
        """
        Draw the user interface for the live data app
        """
        mode_change_button_layout = QHBoxLayout()
        mode_change_button_layout.addWidget(self.mode_selector)
        mode_change_button_layout.addWidget(self.change_parameters_button)

        self.ui_layout = QVBoxLayout()
        
        self.ui_layout.addLayout(mode_change_button_layout)
        self.ui_layout.addWidget(self.checkboxes)
        self.ui_layout.addWidget(self.start_location_controls)
        self.ui_layout.addWidget(self.control_buttons)
        self.ui_layout.addWidget(self.queue_size_label)
        self.ui_layout.addWidget(self.image_display)
        self.ui_layout.addWidget(self.console)
        
        self.main_layout.addLayout(self.ui_layout)
        self.main_layout.addWidget(self.plotter)
        
    def connect_slots_unique(self):
        """
        Connect the app functions unique to the live data app
        """
        # TODO not sure why this is here...
        self.trunk_data_connection.corrected_gnss_data_signal.connect(self.plotter.update_gnss_corrected_estimate)
        self.trunk_data_connection.uncorrected_gnss_data_signal.connect(self.plotter.update_gnss_uncorrected_estimate)
        self.trunk_data_connection.corrected_gnss_data_signal.connect(self.start_location_controls.set_gps_position)
        self.control_buttons.center_on_gps_button.clicked.connect(self.start_location_controls.set_start_location_from_gps)   
    
    def init_modes(self):
        self.pf_live_mode = app_modes.PfLive(self)
        self.modes.append(self.pf_live_mode)

    def init_loaded_data(self):
        pass

    
