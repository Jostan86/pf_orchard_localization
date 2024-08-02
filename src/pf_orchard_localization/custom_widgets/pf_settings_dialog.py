#!/usr/bin/env python3
from PyQt5.QtWidgets import QDialog, QVBoxLayout, QLabel, QLineEdit, QDialogButtonBox
from ..utils.parameters import ParametersPf

class PfSettingsDialog(QDialog):
    """
    A dialog box that allows the user to change settings for the particle filter
    """

    def __init__(self, current_settings=None, parent=None, ):
        """
        Extends QDialog to create a dialog box for changing particle filter settings
        
        Args:
            current_settings (ParametersPf): The current settings for the particle filter
            parent (QWidget): The parent widget for the dialog box
        """
        super(PfSettingsDialog, self).__init__(parent)
        self.init_ui(current_settings)

    def init_ui(self, current_settings: ParametersPf):
        """
        Initializes the UI for the dialog box

        Args:
            current_settings (ParametersPf): The current settings for the particle filter
        """

        layout = QVBoxLayout()
        self.current_settings = current_settings

        dialog_data = {'particle_density': {'label': 'Particle Density (p/m^2):',
                                            'tool_tip': 'The number of particles to generate per square meter of the map.',
                                            'current_value': self.current_settings.particle_density},
                       'linear_noise': {'label': 'Movement noise - linear (m/s):',
                                        'tool_tip': 'The standard deviation of the noise added to the linear velocity of the robot',
                                        'current_value': self.current_settings.r_dist},
                       'angular_noise': {'label': 'Movement noise - angular (deg/s):',
                                         'tool_tip': 'The standard deviation of the noise added to the angular velocity of the robot.',
                                         'current_value': self.current_settings.r_angle},
                       'width_sensor': {'label': 'Width sensor std dev (m):',
                                        'tool_tip': 'The expected standard deviation of the width sensor.',
                                        'current_value': self.current_settings.width_sd},
                       'tree_bearing': {'label': 'Tree bearing sensor std dev (m):',
                                               'tool_tip': 'The expected standard deviation of the tree bearing sensor.',
                                               'current_value': self.current_settings.bearing_sd},
                       'tree_range': {'label': 'Tree range sensor std dev (m):',
                                        'tool_tip': 'The expected standard deviation of the tree range sensor.',
                                        'current_value': self.current_settings.range_sd},
                       'epsilon': {'label': 'Epsilon:',
                                   'tool_tip': 'The allowable error for the KLD sampling algorithm. A smaller value will result in more particles being generated.',
                                   'current_value': self.current_settings.epsilon},
                       'delta': {'label': 'Delta:',
                                 'tool_tip': 'Another term for the kld sampling. A smaller value will result in more particles being generated. I generally have not changed this value, and use epsilon instead.',
                                 'current_value': self.current_settings.delta},
                       'bin_size': {'label': 'Bin size (m):',
                                    'tool_tip': 'The size of the bins used to discretize the map. A smaller value will result in more particles being generated, but also more computation time, adjusting epsilon is usually a better option.',
                                    'current_value': self.current_settings.bin_size},
                       'bin_angle': {'label': 'Bin angle (deg):',
                                     'tool_tip': 'The angular size of the bins used to discretize the map. A smaller value will result in more particles being generated, but also more computation time, adjusting epsilon is usually a better option.',
                                     'current_value': self.current_settings.bin_angle},
                       }

        for key, value in dialog_data.items():
            label = QLabel(value['label'])
            edit = QLineEdit(str(value['current_value']))
            tool_tip = value['tool_tip']
            edit.setToolTip(tool_tip)
            label.setToolTip(tool_tip)
            layout.addWidget(label)
            layout.addWidget(edit)
            setattr(self, f'{key}_edit', edit)

        # Add OK and Cancel buttons
        # self.buttonBox = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel) # Qt6
        self.buttonBox = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        
        self.buttonBox.accepted.connect(self.accept)
        self.buttonBox.rejected.connect(self.reject)
        layout.addWidget(self.buttonBox)

        self.setLayout(layout)
        self.setWindowTitle("Settings")


    def get_settings(self):
        """
        Returns the settings entered by the user if they are valid

        Returns:
            ParametersPf: The settings entered by the user
        """

        try:
            self.current_settings.r_dist = float(self.linear_noise_edit.text())
            self.current_settings.r_angle = float(self.angular_noise_edit.text())
            self.current_settings.width_sd = float(self.width_sensor_edit.text())
            self.current_settings.bearing_sd = float(self.tree_bearing_edit.text())
            self.current_settings.range_sd = float(self.tree_range_edit.text())
            self.current_settings.epsilon = float(self.epsilon_edit.text())
            self.current_settings.delta = float(self.delta_edit.text())
            self.current_settings.bin_size = float(self.bin_size_edit.text())
            self.current_settings.bin_angle = float(self.bin_angle_edit.text())
            return self.current_settings
        except ValueError:
            return None
            