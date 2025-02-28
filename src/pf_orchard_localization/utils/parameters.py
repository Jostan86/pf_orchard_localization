from dataclasses import dataclass, asdict, fields
import yaml
import os
import logging
import os


@dataclass
class Parameters:
    """Base class for parameters.
    
    Stores parameters for different parts of the system. Parameters can be loaded from a yaml file, 
    saved to a yaml file, and logged.
    """

    def load_from_yaml(self, file_path: str) -> None:
        """Load the parameters from a yaml file.
        
        Args:
            file_path (str): The path to the yaml file
        """
        logging.info(f"Loading parameters from {file_path}")

        if not os.path.exists(file_path):
            raise FileNotFoundError(f"Parameter file not found: {file_path}")

        with open(file_path, 'r') as file:
            data = yaml.safe_load(file)

        for field in fields(self):
            current_value = getattr(self, field.name)
            if field.name in data:
                setattr(self, field.name, data[field.name])
            elif current_value is not None:
                logging.warning(f"Field {field.name} not found in yaml file, keeping current value")
            elif current_value is None:
                logging.warning(f"Field {field.name} not found in yaml file and is not yet set")

        self.log_settings()

    def save_to_yaml(self, file_path: str) -> None:
        """Save the parameters to a yaml file.

        Args:
            file_path (str): The path to the yaml file
        """
        logging.info(f"Saving parameters to {file_path}")

        with open(file_path, 'w') as file:
            yaml.dump(asdict(self), file)

    def log_settings(self) -> None:
        """Log the current parameter settings."""
        logging.debug("Current settings:")
        for field in fields(self):
            logging.debug(f"{field.name}: {getattr(self, field.name)}")

@dataclass
class ParametersPf(Parameters):
    """Parameters for the particle filter."""
    
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
    noise_fps: int = None

    width_sd: float = None
    range_sd: float = None
    bearing_sd: float = None
    epsilon: float = None
    delta: float = None
    bin_size: float = None
    bin_angle: int = None
    include_width: bool = None

    use_orientation_for_angular_velocity: bool = None
    num_readings_for_angular_velocity: int = None
    use_orientation_for_particle_weights: bool = None
    orientation_offset: float = None
    print_orientation_offset: bool = None
    orientation_sd: float = None

    motion_update_max_dt: float = None

    stop_when_converged: bool = None

    @property
    def num_particles(self) -> int:
        """Calculate the number of particles based on density and area.
        
        Returns:
            int: The number of particles
        """
        return int(self.particle_density * self.start_width * self.start_height)

@dataclass
class ParametersCachedData(Parameters):
    """Parameters for the cached data version of the app."""
    data_file_dir: str = None
    cached_image_dir: str = None
    test_start_info_path: str = None
    test_results_save_path: str = None

    initial_data_file_index: int = None
    initial_data_time: float = None

    pf_config_file_path: str = None
    map_data_path: str = None

    use_visual_odom: bool = False

@dataclass
class ParametersBagData(Parameters):
    """Parameters for the bag data version of the app."""

    data_file_dir: str = None
    depth_topic: str = None
    rgb_topic: str = None
    odom_topic: str = None
    orientation_topic: str = None
    gnss_corrected_topic: str = None
    gnss_uncorrected_topic: str = None
    initial_data_time: float = None
    initial_data_file_index: int = None

    pf_config_file_path: str = None
    map_data_path: str = None

    image_display_scale: float = None

    use_ros_service_for_trunk_width: bool = False
    use_visual_odom: bool = True

    every_nth_image: int = 1

@dataclass
class ParametersLiveData(Parameters):
    """Parameters for the live data version of the app."""
    depth_topic: str = None
    rgb_topic: str = None
    orientation_topic: str = None
    gnss_corrected_topic: str = None
    gnss_uncorrected_topic: str = None
    odom_topic: str = None

    pf_config_file_path: str = None
    map_data_path: str = None

    image_display_scale: float = None

    def __post_init__(self):
        self.check_and_set_to_env_var("depth_topic", "DEPTH_IMAGE_TOPIC")
        self.check_and_set_to_env_var("rgb_topic", "RGB_IMAGE_TOPIC")
        self.check_and_set_to_env_var("odom_topic", "ODOM_TOPIC")
        self.check_and_set_to_env_var("orientation_topic", "ORIENTATION_TOPIC")
        self.check_and_set_to_env_var("gnss_corrected_topic", "GNSS_CORRECTED_TOPIC")
        self.check_and_set_to_env_var("gnss_uncorrected_topic", "GNSS_UNCORRECTED_TOPIC")

    def check_and_set_to_env_var(self, field_name: str, env_var: str) -> None:
        """Check if field is None and set it to the environment variable if it exists.
        
        Args:
            field_name (str): The name of the field
            env_var (str): The name of the environment variable
        """
        value = os.environ.get(env_var)
        if value is not None:
            setattr(self, field_name, value)
            logging.info(f"Setting {field_name} to {value} from environment variable {env_var}")
        

        



