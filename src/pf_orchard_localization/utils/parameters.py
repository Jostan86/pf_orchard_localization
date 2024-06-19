#!/usr/bin/env python3
from dataclasses import dataclass, asdict, fields
import yaml
import os
import logging

@dataclass
class Parameters:

    def load_from_yaml(self, file_path):
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

        logging.info(f"Saving parameters to {file_path}")

        with open(file_path, 'w') as file:
            yaml.dump(asdict(self), file)

    def log_settings(self):
        logging.info("Current settings:")
        for field in fields(self):
            logging.debug(f"{field.name}: {getattr(self, field.name)}")

@dataclass
class ParametersPf(Parameters):
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
        return int(self.particle_density * self.start_width * self.start_height)

@dataclass
class ParametersCachedData(Parameters):

    data_file_dir: str = None
    cached_image_dir: str = None
    test_start_info_path: str = None

    initial_data_file_index: int = None
    initial_data_time: float = None

    pf_config_file_path: str = None
    map_data_path: str = None


@dataclass
class ParametersBagData(Parameters):

    data_file_dir: str = None
    depth_topic: str = None
    rgb_topic: str = None
    odom_topic: str = None
    initial_data_time: float = None
    initial_data_file_index: int = None

    width_estimation_config_file_path: str = None
    pf_config_file_path: str = None
    map_data_path: str = None
    image_save_dir: str = None

    image_display_scale: float = None

@dataclass
class ParametersLiveData(Parameters):

    depth_topic: str = None
    rgb_topic: str = None
    odom_topic: str = None

    width_estimation_config_file_path: str = None
    pf_config_file_path: str = None
    map_data_path: str = None

    image_display_scale: float = None


