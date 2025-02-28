import numpy as np
from scipy.spatial import KDTree
from scipy.stats import norm
from scipy.ndimage import label
from map_data_tools import MapData
from typing import Union

from pf_orchard_localization.utils import ParametersPf
# from pf_orchard_localization.recorded_data_loaders import MessageType, Timestamp, ImageData, OdomData, ImuData, Gnss, PoseEstimate
from pf_orchard_localization.data_managers import data_msgs

class PfEngine:

    def __init__(self, map_data: MapData, random_seed=None) -> None:
        
        np.random.seed(random_seed)

        # Save the tree positions and widths
        self.map_positions = map_data.all_position_estimates
        self.map_widths = map_data.all_width_estimates

        # Create a KDTree for fast nearest-neighbor lookup of the trees
        self.kd_tree = KDTree(self.map_positions)

        self.best_particle: np.ndarray = None

    def reset_pf(self, setup_data: ParametersPf) -> None:
        """Reset the particle filter with the given setup data.
        
        Args:
            setup_data (ParametersPf): Configuration parameters for the particle filter
            
        Returns:
            None
        """
        self.start_pose_center = np.array([setup_data.start_pose_center_x, setup_data.start_pose_center_y])
        self.start_pose_width = setup_data.start_width
        self.start_pose_height = setup_data.start_height
        self.rotation = np.deg2rad(setup_data.start_rotation)
        self.orientation_center = np.deg2rad(setup_data.start_orientation_center)
        self.orientation_range = np.deg2rad(setup_data.start_orientation_range)
        num_particles = setup_data.num_particles

        self.R = np.diag([setup_data.r_dist, np.deg2rad(setup_data.r_angle)]) ** 2
        self.noise_fps = setup_data.noise_fps

        self.bearing_sd = setup_data.bearing_sd
        self.range_sd = setup_data.range_sd
        self.width_sd = setup_data.width_sd

        self.epsilon = setup_data.epsilon
        self.delta = setup_data.delta
        self.bin_size = setup_data.bin_size
        self.bin_angle = np.deg2rad(setup_data.bin_angle)

        self.include_width = setup_data.include_width

        self.spawn_in_both_directions = setup_data.spawn_particles_in_both_directions

        self.use_orientation_for_angular_velocity = setup_data.use_orientation_for_angular_velocity
        self.num_readings_for_angular_velocity = setup_data.num_readings_for_angular_velocity
        self.use_orientation_for_particle_weights = setup_data.use_orientation_for_particle_weights
        self.orientation_offset = setup_data.orientation_offset
        self.print_orientation_offset = setup_data.print_orientation_offset
        self.orientation_sd = setup_data.orientation_sd

        self.motion_update_max_dt = setup_data.motion_update_max_dt

        self.orientation_prev = None
        self.orientation_current = None
        self.orientation_prev_time = None

        self.gyro_readings = np.zeros(self.num_readings_for_angular_velocity)
        self.gyro_readings_idx = 0

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
        self.prev_dt_odom = None

        self.histogram = None
    
    def reset_timestamps(self) -> None:
        """Reset all timestamps and related flags used for tracking motion."""
        self.prev_t_odom = None
        self.odom_zerod = False
        self.orientation_prev = None
        self.orientation_current = None
        self.orientation_prev_time = None

    def initialize_particles(self, num_particles: int) -> np.ndarray:
        """Initialize the particle poses.
        
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
        particles = np.delete(particles, np.where(distances < 0.6)[0], axis=0)

        return particles

    # def handle_odom(self, x_odom: float, theta_odom: float, timestamp:float, num_readings: int = 1):
    #     """
    #     Handle the odom message. This will be called every time an odom message is received.

    #     Args:
    #         x_odom (float): The linear velocity of the robot in the forward direction, in meters per second
    #         theta_odom (float): The angular velocity of the robot, in radians per second
    #         timestamp (float): The current time stamp of the odom message, in seconds
    #         num_readings (int): The number of readings that have been received since the last odom message was processed
    #     """

    #     # If this is the first odom message, zero the time and return
    #     if not self.odom_zerod:
    #         self.prev_t_odom = timestamp
    #         self.odom_zerod = True
    #         return

    #     # Calculate the time step size
    #     dt_odom = timestamp - self.prev_t_odom

        
    #     self.prev_t_odom = timestamp

    #     # Set up the control input
    #     u = np.array([[x_odom], [theta_odom]])

    #     self.motion_update(u, dt_odom, num_readings)

    def motion_update(self, odom_data: data_msgs.Odom) -> None:
        """Propagate the particles forward in time using the motion model.

        Args:
            odom_data (data_msgs.Odom): The odometry data message, either visual or wheel odometry
        """

        timestamp = odom_data.msg_timestamp.to_sec()
        
        # If this is the first odom message, zero the time and return
        if not self.odom_zerod:
            self.prev_t_odom = timestamp
            self.odom_zerod = True
            return
        
        # Calculate the time step size
        dt_odom = timestamp - self.prev_t_odom
        self.prev_t_odom = timestamp

        if dt_odom > self.motion_update_max_dt:
            if self.prev_dt_odom is not None:
                print(f"Large time step detected: {dt_odom}, using previous delta t instead")
                dt_odom = self.prev_dt_odom
            else:
                print(f"Large time step detected and no previous delta t, skipping update")
                return

        self.prev_dt_odom = dt_odom

        if odom_data.message_type == data_msgs.MsgType.VISUAL_ODOM:
            linear_velocity = odom_data.linear_displacement / dt_odom
        else:
            linear_velocity = odom_data.linear_velocity

        if self.use_orientation_for_angular_velocity:
            angular_velocity = self.angular_velocity
        else:
            angular_velocity = odom_data.angular_velocity

        # Set up the control input
        u = np.array([[linear_velocity], [angular_velocity]])

        num_particles = self.particles.shape[0]

        # This is needed to make the noise independent of the time step size
        noise_multiplier = 1 / np.sqrt(dt_odom * self.noise_fps)

        # Make array of noise to add to the control/odometry velocities
        noise = np.random.randn(num_particles, 2) @ (self.R * noise_multiplier)

        # Add noise to control/odometry velocities
        ud = u + noise.T

        # Update particles based on control/odometry velocities and time step size
        self.particles.T[2, :] += dt_odom * ud[1, :] * 0.5
        self.particles.T[0, :] += dt_odom * ud[0, :] * np.cos(self.particles.T[2, :])
        self.particles.T[1, :] += dt_odom * ud[0, :] * np.sin(self.particles.T[2, :])
        self.particles.T[2, :] += dt_odom * ud[1, :] * 0.5

        # Wrap angles between -pi and pi
        self.particles.T[2, :] = self.wrap_angle(self.particles.T[2, :])

        # Update best particle with raw odom velocities
        self.best_particle[0] += dt_odom * u[0] * np.cos(self.best_particle[2])
        self.best_particle[1] += dt_odom * u[0] * np.sin(self.best_particle[2])
        self.best_particle[2] += dt_odom * u[1]
        self.best_particle[2] = self.wrap_angle(self.best_particle[2])


        if self.orientation_current is not None and self.print_orientation_offset:
            # get the difference between the best particles yaw and the imu reading
            orientation_delta = self.best_particle[2] - self.orientation_current
            orientation_delta = self.wrap_angle(orientation_delta)
            print("Orientation diff: ", orientation_delta)  
    
    def orientation_update(self, imu_msg: data_msgs.Imu) -> None:
        """Handle the IMU message. This will be called every time an IMU message is received.
        
        Args:
            imu_msg (data_msgs.Imu): The IMU data message containing orientation information
        """
        if not (self.use_orientation_for_angular_velocity or self.use_orientation_for_particle_weights):
            return

        self.orientation_current = self.yaw_from_quaternion(imu_msg)
        
        self.orientation_current += self.orientation_offset
        self.orientation_current = self.wrap_angle(self.orientation_current)

        if self.orientation_prev is not None and self.use_orientation_for_angular_velocity:
            delta_yaw = self.orientation_current - self.orientation_prev
            delta_yaw = self.wrap_angle(delta_yaw)
            
            orientation_current_time = imu_msg.msg_timestamp.to_sec()
            delta_time = orientation_current_time - self.orientation_prev_time

            # print(f"Delta time: {delta_time}")
            # print(f"current time: {orientation_current_time}")
            # print(f"Delta yaw: {delta_yaw}")
            angular_velocity = delta_yaw / delta_time

            self.gyro_readings[self.gyro_readings_idx] = angular_velocity
            self.gyro_readings_idx += 1
            if self.gyro_readings_idx == self.num_readings_for_angular_velocity:
                self.gyro_readings_idx = 0

        if self.use_orientation_for_angular_velocity:
            self.orientation_prev = self.orientation_current
            self.orientation_prev_time = imu_msg.msg_timestamp.to_sec()

    def sensor_update(self, image_data_msg: data_msgs.Image) -> None:
        """Does a sensor update based on detected objects and IMU orientation.
        
        This will be called every time a tree message is received. Updates particle weights
        based on the objects in the message and the orientation from the IMU if it is being used.

        Args:
            image_data_msg (data_msgs.Image): The image data message containing detected object locations and widths
        """

        orientation_used_in_weight = False
        tree_position_used_in_weight = False

        if self.orientation_current is not None and self.use_orientation_for_particle_weights:
            orientation_used_in_weight = True
            orientation_weights = self.particle_weight_update_orientation(self.particles, self.orientation_current)

        if image_data_msg.object_locations is not None:

            # Calculate the position of the tree on the map
            tree_global_coords = self.get_object_global_locations(self.particles, image_data_msg.object_locations)

            tree_position_weights = self.particle_weight_update_tree(self.particles, tree_global_coords, image_data_msg.object_widths, image_data_msg.object_locations)
            tree_position_used_in_weight = True

        if orientation_used_in_weight and tree_position_used_in_weight:
            self.particle_weights = orientation_weights * tree_position_weights
        elif orientation_used_in_weight:
            self.particle_weights = orientation_weights
        elif tree_position_used_in_weight:
            self.particle_weights = tree_position_weights
        
        if orientation_used_in_weight or tree_position_used_in_weight:
            # Normalize weights
            self.particle_weights /= np.sum(self.particle_weights)

            # Calculate the 'best' particle as the one with the highest weight
            self.best_particle = self.particles[np.argmax(self.particle_weights)]

        # Resample the particles
        self.resample_particles()
    

    
    @property
    def angular_velocity(self) -> float:
        """Calculate average angular velocity from stored gyro readings.
        
        Returns:
            float: Mean angular velocity in radians per second
        """
        return np.mean(self.gyro_readings)

    def yaw_from_quaternion(self, imu_msg: data_msgs.Imu) -> float:
        """Extract yaw angle from quaternion in IMU message.
        
        Args:
            imu_msg (data_msgs.Imu): The IMU message containing orientation as a quaternion
            
        Returns:
            float: The yaw angle in radians
        """
        x = imu_msg.orientation_x
        y = imu_msg.orientation_y
        z = imu_msg.orientation_z
        w = imu_msg.orientation_w
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = np.arctan2(t3, t4)
        return yaw_z

    def resample_particles(self) -> None:
        """Resample the particles according to the particle weights using the low variance sampling algorithm."""

        # Get the number of particles to resample
        num_particles = self.calculate_num_particles(self.particles)

        # Calculate the step size for resampling
        step_size = np.random.uniform(0, 1 / num_particles)

        # Initialize the new particles array
        new_particles = np.zeros((num_particles, 3))



        # # Set a starting position for the resampling
        # cur_weight = self.particle_weights[0]
        # idx_w = 0

        # # TODO: i think this can be a numpy operation
        # # Use the low variance sampling algorithm to resample the particles
        # for idx_m in range(num_particles):
        #     U = step_size + idx_m / num_particles
        #     while U > cur_weight:
        #         idx_w += 1
        #         cur_weight += self.particle_weights[idx_w]
        #     new_particles[idx_m, :] = self.particles[idx_w, :]

        # Cumulative sum of weights
        cumulative_weights = np.cumsum(self.particle_weights)

        # Generate U values
        U = step_size + np.arange(num_particles) / num_particles

        # Find indices using searchsorted
        indices = np.searchsorted(cumulative_weights, U, side='right')

        # Assign new particles
        new_particles = self.particles[indices]



        self.particles = new_particles

        # Reset the particle weights
        self.particle_weights = np.ones(num_particles) / num_particles

    def get_object_global_locations(self, particle_states: np.ndarray, object_locations: np.ndarray) -> np.ndarray:
        """Calculate the location of objects in the global frame for each particle.
        
        Transforms the object locations from the particle frames to the global frame.

        Args:
            particle_states (np.ndarray): An array of shape (n, 3) containing the states of the particles
            object_locations (np.ndarray): An array of shape (m, 2) containing the locations of the objects seen by the robot

        Returns:
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
        """Calculate object locations in local polar coordinates relative to particles.

        Args:
            particle_states (np.ndarray): An array of shape (n, 3) containing the states of the particles
            object_locs (np.ndarray): An array of shape (n, 2) containing the locations of the objects

        Returns:
            np.ndarray: A Nx2 numpy array, with r and theta coordinates for each object relative to each particle
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
    
    def particle_weight_update_tree(self, particle_states: np.ndarray, sensed_tree_coords: np.ndarray, widths_sensed: np.ndarray, positions_sensed: np.ndarray) -> np.ndarray:
        """Calculate particle weights based on sensed tree locations and widths.

        Args:
            particle_states (np.ndarray): Array of shape (n, 3) containing the states of the particles
            sensed_tree_coords (np.ndarray): Array of shape (m, n, 2) containing the locations of the trees 
                                           in the global frame for each particle, where m is the number 
                                           of trees and n is the number of particles
            widths_sensed (np.ndarray): Array containing the widths of the trees
            positions_sensed (np.ndarray): Array of shape (m, 2) containing the locations of the trees in the robot frame
        
        Returns:
            np.ndarray: Array of shape (n,) containing the weights of the particles
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

            # Calculate the probability of the sensed tree being at the map tree location
            prob_range = self.probability_of_values(range_diff, self.range_sd)
            prob_bearing = self.probability_of_values(bearing_diff, self.bearing_sd)

            # Update the scores
            scores *= prob_range * prob_bearing
            
            if self.include_width:
                # Calculate the difference between the sensed tree width and the map tree width
                width_diffs = np.abs(widths_sensed[i] - (self.map_widths[idx]))

                # Update the scores based on the width difference
                prob_width = self.probability_of_values(width_diffs, self.width_sd)
                scores *= prob_width

        return scores
    
    def particle_weight_update_orientation(self, particle_states: np.ndarray, orientation_sensed: np.ndarray) -> np.ndarray:
        """Calculate particle weights based on the sensed orientation.

        Args:
            particle_states (np.ndarray): Array of shape (n, 3) containing the states of the particles
            orientation_sensed (np.ndarray): Array containing the sensed orientation
        
        Returns:
            np.ndarray: Array of shape (n,) containing the weights of the particles
        """

        # Calculate the difference between the sensed orientation and the particle orientation
        orientation_diffs = np.abs(orientation_sensed - particle_states[:, 2])

        # Calculate the probability of the sensed orientation being the particle orientation
        prob_orientation = self.probability_of_values(orientation_diffs, self.orientation_sd)

        return prob_orientation
    
    def wrap_angle(self, angle: Union[float, np.ndarray]) -> Union[float, np.ndarray]:
        """Wrap an angle to be between -pi and pi.

        Args:
            angle (Union[float, np.ndarray]): The angle(s) to wrap

        Returns:
            Union[float, np.ndarray]: The wrapped angle(s)
        """
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def probability_of_values(self, measurement_discrepancy: np.ndarray, std_dev: float) -> np.ndarray:
        """Calculate probabilities using normal distribution based on measurement discrepancies.
        
        Finds the probability of each particle using a normal distribution given the discrepancy
        between the expected sensor value and the actual sensor value, and the standard deviation.
        
        Args:
            measurement_discrepancy (np.ndarray): Array of differences between expected and actual values
            std_dev (float): Standard deviation of the measurement

        Returns:
            np.ndarray: Probability of each value in the array
        """

        norm_pdf = (1 / (std_dev * np.sqrt(2 * np.pi))) * np.exp(-measurement_discrepancy ** 2 / (2 * std_dev ** 2))
        return norm_pdf

    def rotate_around_point(self, particles: np.ndarray, angle_rad: float, center_point: tuple) -> np.ndarray:
        """Rotate points around a given center point by a given angle.

        Args:
            particles (np.ndarray): Array where first two columns are x and y coordinates
            angle_rad (float): Angle to rotate in radians
            center_point (tuple): Tuple of (x, y) coordinates of rotation center
        
        Returns:
            np.ndarray: Rotated array of particles
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

    def xy_to_polar(self, xy_coords: np.ndarray) -> np.ndarray:
        """Convert an array of xy coordinates to polar coordinates.

        Args:
            xy_coords (np.ndarray): Array of shape (n, 2) with columns (x, y)

        Returns:
            np.ndarray: Array of shape (n, 2) with columns (r, theta)
        """

        # Calculate r and theta
        r = np.sqrt(xy_coords[:, 0] ** 2 + xy_coords[:, 1] ** 2)
        theta = np.arctan2(xy_coords[:, 1], xy_coords[:, 0])

        # Stack r and theta into a single array
        polar_coords = np.stack((r, theta), axis=1)

        return polar_coords

    def calculate_num_particles(self, particles: np.ndarray) -> int:
        """Calculate the optimal number of particles to use based on KLD-sampling.

        Args:
            particles (np.ndarray): Array of shape (num_particles, 3) with columns (x, y, theta)
        
        Returns:
            int: Number of particles for the next timestep
        """
        # Create 2-dimensional grid of bins
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

    def check_convergence(self) -> bool:
        """Check if the particles have converged to a single cluster.
        
        Uses the histogram created for KLD sampling to determine if particles have converged
        to a single cluster of appropriate size.
        
        Returns:
            bool: True if the particles have converged, False otherwise
        """

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

    def downsample_particles(self, max_samples: int = 10000) -> np.ndarray:
        """Downsample particles to a maximum number of samples.

        Args:
            max_samples (int): The maximum number of samples to downsample to

        Returns:
            np.ndarray: Downsampled array of particles
        """
        num_particles = self.particles.shape[0]
        if num_particles <= max_samples:
            return self.particles

        indices = np.random.choice(num_particles, max_samples, replace=False)
        return self.particles[indices]
