from PyQt5.QtCore import pyqtSlot
from PyQt5.QtWidgets import QVBoxLayout, QHBoxLayout, QWidget
from dataclasses import fields
from abc import ABC, abstractmethod
from typing import Union, List, TYPE_CHECKING

from pf_orchard_localization.custom_widgets import (PfControlButtons, PfStartLocationControls, PfCheckBoxes,
                              Console, ImageDisplay, PfModeSelector, PfPlotter, DataFileControls, PfChangeParametersButton)
from pf_orchard_localization.utils.parameters import ParametersPf, ParametersBagData, ParametersCachedData, ParametersLiveData
from pf_orchard_localization.pf_engine import PfEngine
from pf_orchard_localization import img_processing_srv, app_modes
from PyQt5.QtWidgets import QMainWindow, QApplication
from map_data_tools import MapData

if TYPE_CHECKING:
    ...
    
import logging
logger = logging.getLogger(__name__)

class Base(ABC):
    """Base class for the particle filter localization app.
    
    Contains the main structure of the app. Inherited by the PfAppBags, PfAppCached, 
    and PfAppLive classes which are specific implementations for different use cases.
    """
    
    def __init__(self, config_file_path: str):
        """Initialize the app.

        Args:
            config_file_path (str): Path to the configuration file
        """
        super().__init__()

        self.init_main_window()

        # Initialize parameters
        self.parameters_data: Union[ParametersBagData, ParametersCachedData, ParametersLiveData] = None
        self.init_data_parameters()
        self.parameters_data.load_from_yaml(config_file_path)
        self.parameters_pf = ParametersPf()
        self.parameters_pf.load_from_yaml(self.parameters_data.pf_config_file_path)

        self.data_file_controls: DataFileControls = None

        # Load the map data
        self.map_data = MapData(map_data_path=self.parameters_data.map_data_path, move_origin=True, origin_offset=(5, 5))

        # Initialize the particle filter engine
        self.pf_engine = PfEngine(self.map_data)

        self.init_widgets()
        self.draw_ui()

        self.current_msg = None

        self.trunk_data_connection: Union[img_processing_srv.Ros2Sub, img_processing_srv.DirectPkgConnection] = None
        self.setup_trunk_data_connection()
        
        self.connect_slots()
        
        self.modes: List[app_modes.AnyMode] = []
        self.active_mode: app_modes.AnyMode = None
        
        self.init_modes()

        self.mode_selector.set_modes(self.modes)

        self.mode_changed()

        self.init_loaded_data()     

    def init_main_window(self) -> None:
        """Initialize the window display settings."""

        self.main_window = QMainWindow()
        self.main_window.setWindowTitle("Orchard Particle Filter Localization App")

        self.main_window.setGeometry(0, 0, 1700, 900)

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
            self.main_window.move(target_screen.geometry().left(), target_screen.geometry().top())

        logging.debug(f"Target screen number: {target_screen_number}")
        logging.debug(f"Screen size: {target_screen.geometry().width()} x {target_screen.geometry().height()}")
        logging.debug(f"App size: {self.main_window.width()} x {self.main_window.height()}")
        logging.debug(f"App position: {self.main_window.x()} x {self.main_window.y()}")


    @abstractmethod
    def init_data_parameters(self) -> None:
        """Initialize the data parameters."""
        pass

    @abstractmethod
    def init_loaded_data(self) -> None:
        """Initialize the loaded data."""
        pass
    
    @abstractmethod
    def init_modes(self) -> None:
        """Initialize the different app modes."""
        pass
    
    @abstractmethod
    def draw_ui(self) -> None:
        """Draw the user interface."""
        pass

    def init_widgets(self) -> None:
        """Initialize all the widgets used in the app."""

        self.widget_list: List[QWidget] = []

        self.main_layout = QHBoxLayout()

        # Setup widgets shared by all the app types
        self.start_location_controls = PfStartLocationControls(self)
        self.control_buttons = PfControlButtons(self)
        self.change_parameters_button = PfChangeParametersButton(self)
        self.mode_selector = PfModeSelector()
        self.image_display = ImageDisplay(scale_factor=1.0)
        self.console = Console()

        # Setup the checkboxes
        self.checkboxes = PfCheckBoxes()
        self.checkboxes.all_checkbox_info.append(
            ["include_width_checkbox", "Use Width in Weight Calculation", self.parameters_pf.include_width])
        self.checkboxes.all_checkbox_info.append(
            ["stop_when_converged_checkbox", "Stop When Converged", self.parameters_pf.stop_when_converged])
        self.checkboxes.init_checkboxes()

        self.plotter = PfPlotter(self.map_data)

        # Set the central widget of the app
        central_widget = QWidget(self.main_window)
        self.main_window.setCentralWidget(central_widget)
        central_widget.setLayout(self.main_layout)

        # Setup the widgets unique to the app types
        self.init_widgets_unique()

        # Add the widgets to the list of all widgets
        self.widget_list += [self.start_location_controls, self.checkboxes, self.change_parameters_button,
                            self.mode_selector, self.control_buttons, self.image_display, self.console, self.plotter,]

    @abstractmethod
    def init_widgets_unique(self) -> None:
        """Initialize the widgets unique to the app type."""
        pass

    def connect_slots(self) -> None:
        """Connect the UI elements to the app functions."""

        self.control_buttons.reset_button.clicked.connect(lambda x: self.reset_pf(use_ui_parameters=True))

        self.plotter.plot_widget.clicked.connect(self.start_location_controls.set_start_location_from_plot_click)

        self.checkboxes.include_width_checkbox.stateChanged.connect(self.include_width_changed)
        self.checkboxes.stop_when_converged_checkbox.stateChanged.connect(self.stop_when_converged_changed)
        

        self.image_display.imageDisplayChangeSignal.connect(self.trunk_data_connection.set_images_to_include)
        # TODO: Don't love having this here, but it's needed right now
        self.image_display.checkbox_changed()
        self.image_display.imageDisplayChangeSignal.connect(self.image_display_checkbox_changed)
        self.trunk_data_connection.signal_request_processed.connect(self.image_display.set_images)


        self.mode_selector.mode_selector.currentIndexChanged.connect(self.mode_changed)
        
        self.trunk_data_connection.signal_print_message.connect(self.print_message)
        

        self.connect_slots_unique()

    @abstractmethod
    def connect_slots_unique(self) -> None:
        """Connect the app functions unique to the app type."""
        pass

    @abstractmethod
    def setup_trunk_data_connection(self) -> None:
        """Setup the connection to the trunk segmenter and analyzer."""
        pass
    
    def image_display_checkbox_changed(self, checkbox_states: dict = None) -> None:
        """Update the image display when checkboxes change.
        
        Args:
            checkbox_states (dict, optional): States of the checkboxes
        """
        # Update the image display with the current image if there is one
        if self.data_file_controls.data_manager is not None:
            # request = {"current_msg": self.data_file_controls.data_manager.current_msg, "for_display_only": True}
            self.trunk_data_connection.handle_request(self.data_file_controls.data_manager.current_msg)

    def include_width_changed(self) -> None:
        """Update the include width parameter based on the checkbox state."""
        self.parameters_pf.include_width = self.checkboxes.include_width_checkbox.isChecked()

    def stop_when_converged_changed(self) -> None:
        """Update the stop when converged parameter based on the checkbox state."""
        self.parameters_pf.stop_when_converged = self.checkboxes.stop_when_converged_checkbox.isChecked()
    
    def hide_all_widgets(self) -> None:
        """Hide all widgets in the app."""
        for widget in self.widget_list:
            if widget is not None:
                widget.hide()
                
    def mode_changed(self) -> None:
        """Change the active mode based on the current mode selector value."""
        # Find the active mode and deactivate it
        for mode in self.modes:
            if mode.mode_active:
                mode.deactivate_mode()

        self.hide_all_widgets()

        # Find the new active mode and activate it
        for mode in self.modes:
            if mode.mode_name == self.mode_selector.mode:
                mode.activate_mode()
                self.active_mode = mode
                break

    def reset_pf(self, use_ui_parameters: bool = True) -> None:
        """Reset the particle filter.

        Args:
            use_ui_parameters (bool): Whether to use parameters from UI or from app settings
        """
        if use_ui_parameters:
            self.start_location_controls.get_parameters()
        else:
            self.start_location_controls.set_parameters()

        self.pf_engine.reset_pf(self.parameters_pf)

        self.reset_gui()

    def reset_gui(self) -> None:
        """Reset the particle filter elements in the GUI."""

        self.control_buttons.set_num_particles(self.pf_engine.particles.shape[0])
        self.plotter.update_particles(self.pf_engine.downsample_particles())
        self.plotter.update_position_estimate(None)

    def get_pf_active(self) -> bool:
        """Check if the particle filter is currently active.
        
        Returns:
            bool: True if the particle filter is active, False otherwise
        """

        if self.active_mode is not None:
            if hasattr(self.active_mode, "pf_continuous_active"):
                return self.active_mode.pf_continuous_active
        
        return False

    def get_pf_parameters(self) -> ParametersPf:
        """Get the particle filter parameters.
        
        Returns:
            ParametersPf: The current particle filter parameters
        """

        return self.parameters_pf

    def set_pf_parameters(self, parameters: ParametersPf) -> bool:
        """Set the particle filter parameters.

        Args:
            parameters (ParametersPf): New particle filter parameters
            
        Returns:
            bool: True if parameters were set successfully, False otherwise
        """

        if self.get_pf_active():
            self.print_message("Cannot change parameters while the particle filter is running")
            return False
        self.parameters_pf = parameters
        return True

    def print_message(self, message: Union[str, list]) -> None:
        """Print a message to the console.

        Args:
            message (Union[str, list]): Message or list of messages to print
        """

        if isinstance(message, list):
            for msg in message:
                self.console(msg)
        else:
            self.console(message)

    def display_pf_settings(self) -> None:
        """Display the current particle filter settings in the console."""

        for field in fields(self.parameters_pf):
            value = getattr(self.parameters_pf, field.name)
            self.print_message(field.name + ": " + str(value))

    def closeEvent(self, event) -> None:
        """Handle the window close event.
        
        Args:
            event: The close event
        """
        for mode in self.modes:
            if mode.mode_active:
                mode.shutdown_hook()

        event.accept()

