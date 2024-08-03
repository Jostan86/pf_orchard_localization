#!/usr/bin/env python3
from dataclasses import dataclass, asdict, fields
import yaml
import os
import logging
import os


@dataclass
class Parameters:
    """
    Base class for parameters. The parameters class is used to store the parameters for the different parts of the system.
    The parameters can be loaded from a yaml file, saved to a yaml file, and logged.
    """

    def load_from_yaml(self, file_path):
        """
        Load the parameters from a yaml file
        
        Args:
            file_path (str): The path to the yaml file
        """
        logging.info(f"Loading parameters from {file_path}")

        if not os.path.exists(file_path):
            raise FileNotFoundError(f"Parameter file not found: {file_path}")

        with open(file_path, 'r') as file:
            data = yaml.safe_load(file)

        for field in fields(self):
            if field.name in data:
                setattr(self, field.name, data[field.name])

        self.log_settings()

    def save_to_yaml(self, file_path):
        """
        Save the parameters to a yaml file

        Args:
            file_path (str): The path to the yaml file
        """
        logging.info(f"Saving parameters to {file_path}")

        with open(file_path, 'w') as file:
            yaml.dump(asdict(self), file)

    def log_settings(self):
        """
        Log the current settings
        """
        logging.info("Current settings:")
        for field in fields(self):
            logging.debug(f"{field.name}: {getattr(self, field.name)}")

@dataclass
class ParametersPf(Parameters):
    """
    Parameters for the particle filter
    """
    
    start_pose_center_x: float = None
    start_pose_center_y: float = None
    particle_density: int = None
    start_width: float = None
    start_height: float = None
    start_rotation: float = None
    x_offset: float = None
    y_offset: float = None
    start_orientation_center: float = None
    start_orientation_range: float = None
    spawn_particles_in_both_directions: bool = None

    r_dist: float = None
    r_angle: int = None
    width_sd: float = None
    range_sd: float = None
    bearing_sd: float = None
    epsilon: float = None
    delta: float = None
    bin_size: float = None
    bin_angle: int = None
    include_width: bool = None

    stop_when_converged: bool = None

    @property
    def num_particles(self):
        """
        Calculate the number of particles based on the particle density and the start width and height
        
        Returns:
            int: The number of particles
        """
        return int(self.particle_density * self.start_width * self.start_height)

@dataclass
class ParametersCachedData(Parameters):
    """
    Parameters for the cached data version of the app
    """
    data_file_dir: str = None
    cached_image_dir: str = None
    test_start_info_path: str = None

    initial_data_file_index: int = None
    initial_data_time: float = None

    pf_config_file_path: str = None
    map_data_path: str = None

    image_fps: int = None
    use_visual_odom: bool = False

@dataclass
class ParametersBagData(Parameters):
    """
    Parameters for the bag data version of the app
    """

    data_file_dir: str = None
    depth_topic: str = None
    rgb_topic: str = None
    odom_topic: str = None
    initial_data_time: float = None
    initial_data_file_index: int = None
    image_fps: int = None

    pf_config_file_path: str = None
    map_data_path: str = None
    image_save_dir: str = None

    image_display_scale: float = None

    use_ros_service_for_trunk_width: bool = False
    use_visual_odom: bool = False

@dataclass
class ParametersLiveData(Parameters):
    """
    Parameters for the live data version of the app
    """
    depth_topic: str = os.environ.get("DEPTH_IMAGE_TOPIC")
    rgb_topic: str = os.environ.get("RGB_IMAGE_TOPIC")
    gnss_topic: str = os.environ.get("GNSS_TOPIC")
    image_fps: int = os.environ.get("IMAGE_FPS")
    odom_topic: str = None

    pf_config_file_path: str = None
    map_data_path: str = None

    image_display_scale: float = None


