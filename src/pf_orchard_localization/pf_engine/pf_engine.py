#!/usr/bin/env python3

import numpy as np
from scipy.spatial import KDTree
from scipy.stats import norm
from scipy.ndimage import label
from map_data_tools import MapData
class PFEngine:

    def __init__(self, map_data: MapData, random_seed=None) -> None:
        
        np.random.seed(random_seed)

        # Save the tree positions and widths
        self.map_positions = map_data.all_position_estimates
        self.map_widths = map_data.all_width_estimates

        # Create a KDTree for fast nearest-neighbor lookup of the trees
        self.kd_tree = KDTree(self.map_positions)

    def reset_pf(self, setup_data) -> None:
        """Reset the particle filter with the given setup data."""

        self.start_pose_center = np.array([setup_data.start_pose_center_x, setup_data.start_pose_center_y])
        self.start_pose_width = setup_data.start_width
        self.start_pose_height = setup_data.start_height
        self.rotation = np.deg2rad(setup_data.start_rotation)
        self.orientation_center = np.deg2rad(setup_data.start_orientation_center)
        self.orientation_range = np.deg2rad(setup_data.start_orientation_range)
        num_particles = setup_data.num_particles
        self.R = np.diag([setup_data.r_dist, np.deg2rad(setup_data.r_angle)]) ** 2
        self.bearing_sd = setup_data.bearing_sd
        self.range_sd = setup_data.range_sd
        self.width_sd = setup_data.width_sd
        self.epsilon = setup_data.epsilon
        self.delta = setup_data.delta
        self.bin_size = setup_data.bin_size
        self.bin_angle = np.deg2rad(setup_data.bin_angle)
        self.include_width = setup_data.include_width
        self.spawn_in_both_directions = setup_data.spawn_particles_in_both_directions

        self.particles = self.initialize_particles(num_particles)
        num_particles = self.particles.shape[0]

        self.particle_weights = np.ones(num_particles) / num_particles
        self.best_particle = self.particles[0]

        if hasattr(setup_data, 'max_num_particles'):
            self.max_num_particles = setup_data.max_num_particles
        else:
            self.max_num_particles = 2000000

        if hasattr(setup_data, 'min_num_particles'):
            self.min_num_particles = setup_data.min_num_particles
        else:
            self.min_num_particles = 100

        self.odom_zerod = False
        self.prev_t_odom = None

        self.histogram = None

    def initialize_particles(self, num_particles: int):
        """
        Initialize the particle poses

        Args:
            num_particles (int): The number of particles to initialize with

        Returns:
            np.ndarray: An array of shape (num_particles, 3) containing the initial particle poses as (x, y, theta)

        """

        start_pose_center_x = self.start_pose_center[0]
        start_pose_center_y = self.start_pose_center[1]
        start_pose_width_by_2 = self.start_pose_width / 2
        start_pose_height_by_2 = self.start_pose_height / 2

        orientation_min = -self.orientation_range / 2
        orientation_max = self.orientation_range / 2

        # Initialize the particles
        particles = np.zeros((num_particles, 3))

        # Set the x and y coordinates of the particles to be uniformly distributed around the start pose center
        particles[:, 0] = np.random.uniform(start_pose_center_x - start_pose_width_by_2,
                                            start_pose_center_x + start_pose_width_by_2,
                                            num_particles)
        particles[:, 1] = np.random.uniform(start_pose_center_y - start_pose_height_by_2,
                                            start_pose_center_y + start_pose_height_by_2,
                                            num_particles)

        # Set the orientation of the particles to be uniformly distributed around the orientation center, or put half facing the oposite direction if spawn_in_both_directions is True
        if self.spawn_in_both_directions:
            half_particle_num = int(num_particles / 2)
            particles[:half_particle_num, 2] = np.random.uniform(orientation_min, orientation_max, half_particle_num) + self.orientation_center
            particles[half_particle_num:, 2] = np.random.uniform(orientation_min, orientation_max, half_particle_num) + self.orientation_center - np.pi
        else:
            particles[:, 2] = np.random.uniform(orientation_min, orientation_max, num_particles) + self.orientation_center

        # Rotate the particles around the start pose center by the given rotation
        particles = self.rotate_around_point(particles, self.rotation, self.start_pose_center)

        # Find the closest map tree to each particle
        distances, idx = self.kd_tree.query(particles[:, 0:2])

        # remove particles that are too close to a tree
        particles = np.delete(particles, np.where(distances < 0.8)[0], axis=0)

        return particles

    def handle_odom(self, x_odom: float, theta_odom: float, time_stamp:float, num_readings: int = 1):
        """
        Handle the odom message. This will be called every time an odom message is received.

        Args:
            x_odom (float): The linear velocity of the robot in the forward direction, in meters per second
            theta_odom (float): The angular velocity of the robot, in radians per second
            time_stamp (float): The current time stamp of the odom message, in seconds
            num_readings (int): The number of readings that have been received since the last odom message was processed
        """

        # If this is the first odom message, zero the time and return
        if not self.odom_zerod:
            self.prev_t_odom = time_stamp
            self.odom_zerod = True
            return

        # Calculate the time step size
        dt_odom = time_stamp - self.prev_t_odom

        self.prev_t_odom = time_stamp

        # Set up the control input
        u = np.array([[x_odom], [theta_odom]])

        self.motion_update(u, dt_odom, num_readings)

    def scan_update(self, tree_msg):
        """Handle the tree message. This will be called every time a tree message is received."""


        if tree_msg['positions'] is not None:

            postions_sense = np.array(tree_msg['positions'])
            widths_sense = np.array(tree_msg['widths'])

            # Calculate the position of the tree on the map
            tree_global_coords = self.get_object_global_locations(self.particles, postions_sense)

            # Calculate the weights of the particles
            self.particle_weights = self.get_particle_weight_localize(self.particles, tree_global_coords, widths_sense, postions_sense)

            # Normalize weights
            self.particle_weights /= np.sum(self.particle_weights)

            # Calculate the 'best' particle as the one with the highest weight
            self.best_particle = self.particles[np.argmax(self.particle_weights)]

        # Resample the particles
        self.resample_particles()
        
    def motion_update(self, u: np.ndarray, dt: float, num_readings: int):
        """
        Propagate the particles forward in time using the motion model.

        Args:
            u (np.ndarray): The control input, consisting of the linear velocity in the forward direction and the angular velocity
            dt (float): The time step size
            num_readings (int): The number of readings that have been received since the last odom message was processed
        """


        num_particles = self.particles.shape[0]

        # Make array of noise. Noise is averaged over multiple readings if num_readings > 1
        noise = np.random.randn(num_particles, 2) @ (self.R / np.sqrt(num_readings))

        # Add noise to control/odometry velocities
        ud = u + noise.T

        # Update particles based on control/odometry velocities and time step size
        self.particles.T[2, :] += dt * ud[1, :] * 0.5
        self.particles.T[0, :] += dt * ud[0, :] * np.cos(self.particles.T[2, :])
        self.particles.T[1, :] += dt * ud[0, :] * np.sin(self.particles.T[2, :])
        self.particles.T[2, :] += dt * ud[1, :] * 0.5

        # Wrap angles between -pi and pi
        self.particles.T[2, :] = (self.particles.T[2, :] + np.pi) % (2 * np.pi) - np.pi

        # Update best particle with raw odom velocities
        self.best_particle[0] += dt * u[0] * np.cos(self.best_particle[2])
        self.best_particle[1] += dt * u[0] * np.sin(self.best_particle[2])
        self.best_particle[2] += dt * u[1]
        self.best_particle[2] = (self.best_particle[2] + np.pi) % (2 * np.pi) - np.pi


    def resample_particles(self):
        """Resample the particles according to the particle weights using the low variance sampling algorithm."""

        # Get the number of particles to resample
        num_particles = self.calculate_num_particles(self.particles)

        # Calculate the step size for resampling
        step_size = np.random.uniform(0, 1 / num_particles)

        # Set a starting position for the resampling
        cur_weight = self.particle_weights[0]
        idx_w = 0

        # Initialize the new particles array
        new_particles = np.zeros((num_particles, 3))

        # TODO: i think this can be a numpy operation
        # Use the low variance sampling algorithm to resample the particles
        for idx_m in range(num_particles):
            U = step_size + idx_m / num_particles
            while U > cur_weight:
                idx_w += 1
                cur_weight += self.particle_weights[idx_w]
            new_particles[idx_m, :] = self.particles[idx_w, :]

        self.particles = new_particles

        # Reset the particle weights
        self.particle_weights = np.ones(num_particles) / num_particles

    def get_object_global_locations(self, particle_states: np.ndarray, object_locations: np.ndarray) -> np.ndarray:
        """
        Calculates the location of the given objects in the global frame for each particle by transforming the object
        locations in the particle frames to the global frame.

        Args:
            particle_states (np.ndarray): An array of shape (n, 3) containing the states of the particles
            object_locations (np.ndarray): An array of shape (n, 2) containing the locations of the objects seen by the robot

        Returns

            np.ndarray: A MxNx2 numpy array, with x and y coordinates for each object relative to each particle. Here n
            is the number of particles and m is the number of trees.
        """

        # Calculate sin and cos of particle angles
        s = np.sin(particle_states[:, 2])
        c = np.cos(particle_states[:, 2])

        object_global_location = np.zeros((object_locations.shape[0], particle_states.shape[0], 2))

        for i in range(len(object_locations)):
            # Calculate x and y coordinates of trees in global frame
            object_global_location[i, :, 0] = particle_states[:, 0] + object_locations[i, 0] * c + object_locations[i, 1] * -s
            object_global_location[i, :, 1] = particle_states[:, 1] + object_locations[i, 0] * s + object_locations[i, 1] * c

        return object_global_location

    def object_local_polar_transform(self, particle_states: np.ndarray, object_locs: np.ndarray) -> np.ndarray:
        """
        Calculates an array of the object's location in the local frame.

        Parameters:
        ----------
        particle_states : np.ndarray
            An array of shape (n, 3) containing the states of the particles
        object_locs : np.ndarray
            An array of shape (n, 2) containing the locations of the closest object to the object seen by each particle.

        Returns
         ----------
         np.ndarray
            A Nx2 numpy array, with r and theta coordinates for each object relative to each particle.
        """
        # Calculate differences in x and y coordinates
        dx = object_locs[:, 0] - particle_states[:, 0]
        dy = object_locs[:, 1] - particle_states[:, 1]

        # Calculate range (Euclidean distance)
        ranges = np.sqrt(dx ** 2 + dy ** 2)

        # Calculate bearing, adjusting for particle orientation
        bearings = np.arctan2(dy, dx) - particle_states[:, 2]

        # Combine ranges and bearings into a single array
        polar_coords = np.vstack((ranges, bearings)).T

        return polar_coords

    def get_particle_weight_localize(self, particle_states, sensed_tree_coords, widths_sensed, positions_sensed) -> np.ndarray:
        """

        Parameters
        ----------
        particle_states : np.ndarray
            An array of shape (n, 3) containing the states of the particles
        sensed_tree_coords : np.ndarray
            An array of shape (m, n, 2) containing the locations of the trees in the global frame for each particle. m is the
            number of trees and n is the number of particles.
        widths_sensed : np.ndarray
            An array containing the widths of the trees.

        Returns
        -------
        np.ndarray
            An array of shape (n,) containing the weights of the particles.
        """

        # Initialize scores
        scores = np.ones(particle_states.shape[0], dtype=float)

        # Calculate the distance between the sensed trees and the map trees for each of the sensed tree
        for i in range(len(sensed_tree_coords)):

            # Find the nearest neighbor of each sensed tree in the map
            distances, idx = self.kd_tree.query(sensed_tree_coords[i, :, :])

            # find the range and bearing of the sensed tree relative to the particle
            object_coords = self.map_positions[idx]

            object_relative_particles_rb = self.object_local_polar_transform(particle_states, object_coords)

            seen_object_rb = self.xy_to_polar(positions_sensed[i, :].reshape(1, 2))

            range_diff = np.abs(object_relative_particles_rb[:, 0] - seen_object_rb[:, 0])
            bearing_diff = np.abs(np.arctan2(np.sin(object_relative_particles_rb[:, 1] - seen_object_rb[:, 1]),
                                             np.cos(object_relative_particles_rb[:, 1] - seen_object_rb[:, 1])))

            # for j in range(5):
            #     print("range diff: ", range_diff[j], "bearing diff: ", bearing_diff[j])

            # Calculate the probability of the sensed tree being at the map tree location
            prob_range = self.probability_of_values(range_diff, 0, self.range_sd)
            prob_bearing = self.probability_of_values(bearing_diff, 0, self.bearing_sd)

            # Update the scores
            scores *= prob_range * prob_bearing

            if self.include_width:
                # Calculate the difference between the sensed tree width and the map tree width
                width_diffs = np.abs(widths_sensed[i] - (self.map_widths[idx]))

                # Update the scores based on the width difference
                prob_width = self.probability_of_values(width_diffs, 0, self.width_sd)
                scores *= prob_width

        return scores

    def probability_of_values(self, arr, mean, std_dev):
        """Find the probability of a value in an array given a mean and standard deviation."""

        norm_pdf = (1 / (std_dev * np.sqrt(2 * np.pi))) * np.exp(-(arr - mean) ** 2 / (2 * std_dev ** 2))
        return norm_pdf

    def rotate_around_point(self, particles, angle_rad, center_point):
        """
        Rotate numpy array points (in the first two columns)
        around a given point by a given angle in degrees.

        Parameters:
        - matrix: numpy array where first two columns are x and y coordinates
        - angle_degree: angle to rotate in degrees
        - point: tuple of (x, y) coordinates of rotation center

        Returns:
        - Rotated numpy array
        """

        # Rotation matrix
        rotation_matrix = np.array([
            [np.cos(angle_rad), -np.sin(angle_rad)],
            [np.sin(angle_rad), np.cos(angle_rad)]
        ])

        # Extract x and y columns
        xy_coords = particles[:, 0:2]

        # Translate points to origin based on the provided point
        translated_points = xy_coords - center_point

        # Apply rotation
        rotated_points = np.dot(translated_points, rotation_matrix.T)

        # Translate points back to original place
        rotated_translated_points = rotated_points + center_point

        # Replace original x and y values in the matrix with the rotated values
        particles[:, 0:2] = rotated_translated_points

        return particles

    def xy_to_polar(self, xy_coords: np.ndarray):
        """
        Convert an array of xy coordinates to polar coordinates.

        Parameters:
        - xy_coords: numpy array of shape (n, 2) with columns (x, y)

        Returns:
        - polar_coords: numpy array of shape (n, 2) with columns (r, theta)
        """

        # Calculate r and theta
        r = np.sqrt(xy_coords[:, 0] ** 2 + xy_coords[:, 1] ** 2)
        theta = np.arctan2(xy_coords[:, 1], xy_coords[:, 0])

        # Stack r and theta into a single array
        polar_coords = np.stack((r, theta), axis=1)

        return polar_coords

    def calculate_num_particles(self, particles):
        """
        Calculate the number of particles to use based on KLD-sampling.

        Args:
        - particles (np.array): Array of shape (num_particles, 3) with columns (x, y, theta).
        - epsilon (float): Maximum allowable error in K-L distance.
        - delta (float): Desired confidence in the calculated number of particles.
        - bin_size (tuple): Size of the bins in each dimension (x, y, theta).

        Returns:
        - int: Number of particles for the next timestep.
        """

        # Create multi-dimensional grid
        x_bins = np.arange(particles[:, 0].min(), particles[:, 0].max() + self.bin_size, self.bin_size)
        y_bins = np.arange(particles[:, 1].min(), particles[:, 1].max() + self.bin_size, self.bin_size)
        theta_bins = np.arange(particles[:, 2].min(), particles[:, 2].max() + self.bin_angle, self.bin_angle)

        # Calculate histogram to determine number of non-empty bins (k)
        self.histogram, _ = np.histogramdd(particles, bins=(x_bins, y_bins, theta_bins))
        k = np.sum(self.histogram > 0)

        if k == 1:
            return self.min_num_particles

        # Calculate z_1-delta (upper 1-delta quantile of the standard normal distribution)
        z_1_delta = norm.ppf(1 - self.delta)

        # Calculate n using the derived formula
        first_term = (k - 1) / (2 * self.epsilon)
        second_term = (1 - (2 / (9 * (k - 1))) + np.sqrt(2 * z_1_delta / (9 * (k - 1)))) ** 3
        n = first_term * second_term

        if n < self.min_num_particles:
            n = self.min_num_particles

        if n > self.max_num_particles:
            n = self.max_num_particles

        return int(np.ceil(n))

    def check_convergence(self):
        """Check if the particles have converged to a single cluster using the histogram created for kld sampling."""

        hist = self.histogram

        if hist is None:
            return False

        # Convert the histogram to binary where bins with any count are considered as occupied.
        binary_mask = (hist > 0).astype(int)

        # Define a structure for direct connectivity in 3D.
        structure = np.array([[[0, 0, 0],
                               [0, 1, 0],
                               [0, 0, 0]],

                              [[0, 1, 0],
                               [1, 1, 1],
                               [0, 1, 0]],

                              [[0, 0, 0],
                               [0, 1, 0],
                               [0, 0, 0]]])

        # Label connected components. The structure defines what is considered "connected".
        labeled_array, num_features = label(binary_mask, structure=structure)

        # If there is only one feature, then the particles have converged
        if num_features == 1:
            # Get the size of each feature
            feature_sizes = np.bincount(labeled_array.ravel())[1:]
            # Calculate the maximum feature size that is allowed, this is somewhat arbitrary but is based on the
            # bin linear and angular sizes, it has worked well in testing.
            feature_size_max = int(((1 / self.bin_size) ** 2) * ((0.25 * np.pi) / self.bin_angle))
            if feature_sizes[0] < feature_size_max:
                return True
            else:
                return False
        else:
            return False

    def downsample_particles(self, max_samples=10000):
        """
        Downsample a 2D array of particles to a maximum number of samples.

        Parameters:
        - max_samples: int, maximum number of samples after downsampling

        Returns:
        - Downsampled 2D numpy array of particles
        """
        num_particles = self.particles.shape[0]
        if num_particles <= max_samples:
            return self.particles

        indices = np.random.choice(num_particles, max_samples, replace=False)
        return self.particles[indices]
