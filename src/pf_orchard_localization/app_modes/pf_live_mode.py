from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QHBoxLayout, QVBoxLayout, QApplication
import rospy
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np

class PfLiveMode:
    def __init__(self, main_app_manager):
        self.main_app_manager = main_app_manager

        self.converged = False
        self.is_processing = False

        self.mode_active = False

        self.pf_continuous_active = False
        self.timer = None

        self.ros_connected = False

        self.previous_image_timestamp = None

        self.mode_name = "PF - Live Data"

        self.bridge = CvBridge()

        self.odom_msgs = []
        self.rgb_image_msgs = []
        self.depth_image_msgs = []

        self.odom_times = []
        self.rgb_image_timestamps = []
        self.depth_image_timestamps = []

    def connect_to_ros(self):
        rospy.init_node('pf_live_mode', anonymous=True)
        self.rgb_image_sub = rospy.Subscriber('/registered/rgb/image', Image, self.rgb_image_callback)
        self.depth_image_sub = rospy.Subscriber('/registered/depth/image', Image, self.depth_image_callback)
        self.odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback)
        # self.tree_data_pub = rospy.Publisher('/tree_data', TreeData, queue_size=10)

        self.ros_connected = True
        self.prev_tree_data_time = None

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_pf)
        self.timer.start(0.01)

        self.main_app_manager.ros_connect_button.set_connected()

        self.main_app_manager.print_message("Connected to ROS")

    def disconnect_from_ros(self):
        if not self.ros_connected:
            return

        self.timer.stop()

        self.rgb_image_sub.unregister()
        self.depth_image_sub.unregister()
        self.odom_sub.unregister()

        self.ros_connected = False

        self.main_app_manager.ros_connect_button.set_disconnected()

        self.main_app_manager.print_message("Disconnected from ROS")


    def rgb_image_callback(self, image_msg):
        self.rgb_image_msgs.append(image_msg)
        self.rgb_image_timestamps.append(image_msg.header.stamp.to_sec())

        # if not self.pf_active and len(self.rgb_image_msgs) > 1:
        #     image_msg = self.rgb_image_msgs.pop(0)
        #     self.rgb_image_timestamps.pop(0)
        #     image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        #     self.qt_window.load_image(image)

    def depth_image_callback(self, image_msg):
        self.depth_image_msgs.append(image_msg)
        self.depth_image_timestamps.append(image_msg.header.stamp.to_sec())
    def odom_callback(self, odom_msg):
        self.odom_msgs.append(odom_msg)
        self.odom_times.append(odom_msg.header.stamp.to_sec())

    def ensure_pf_stopped(self):
        if self.pf_continuous_active:
            self.stop_pf_continuous()

    def update_pf(self):

        if self.is_processing or not self.ros_connected:
            return

        self.is_processing = True

        positions, widths, class_estimates, image_timestamp = self.get_tree_data()

        if image_timestamp is None:
            self.is_processing = False
            return

        if self.previous_image_timestamp is None:
            self.previous_image_timestamp = image_timestamp
            self.is_processing = False
            return

        if not self.pf_continuous_active:
            self.is_processing = False
            return

        self.main_app_manager.queue_size_label.set_queue_size(len(self.rgb_image_msgs))

        tree_data = {'positions': positions, 'widths': widths, 'classes': class_estimates}

        # Get the odom messages that are between the previous tree data time and the current tree data time
        start_index = next((i for i, v in enumerate(self.odom_times) if v >= self.previous_image_timestamp), 0)
        end_index = next((i for i, v in enumerate(self.odom_times) if v >= image_timestamp), len(self.odom_times))
        odom_msgs_cur = self.odom_msgs[start_index:end_index]
        odom_times_cur = self.odom_times[start_index:end_index]

        # if self.prev_odom_x is not None and len(odom_msgs_cur) > 0:
        #     self.distance_traveled += np.linalg.norm(np.array([odom_msgs_cur[-1].pose.pose.position.x - self.prev_odom_x, odom_msgs_cur[-1].pose.pose.position.y - self.prev_odom_y]))
        # if len(odom_msgs_cur) > 0:
        #     self.prev_odom_x = odom_msgs_cur[-1].pose.pose.position.x
        #     self.prev_odom_y = odom_msgs_cur[-1].pose.pose.position.y

        # Get rid of the odom messages that have been used
        self.odom_msgs = self.odom_msgs[end_index:]
        self.odom_times = self.odom_times[end_index:]

        # Save the current tree data time for the next iteration
        self.previous_image_timestamp = image_timestamp

        # Get the average linear and angular velocities, the number of odom readings, and the time of the last odom
        # reading from the odom messages
        if len(odom_msgs_cur) > 0:
            x_odom = np.mean(np.array([odom_msg.twist.twist.linear.x for odom_msg in odom_msgs_cur]))
            theta_odom = np.mean(np.array([odom_msg.twist.twist.angular.z for odom_msg in odom_msgs_cur]))
            time_stamp_odom = np.max(np.array([odom_times_cur]))
            num_readings = len(odom_msgs_cur)
            self.main_app_manager.pf_engine.handle_odom(x_odom, theta_odom, time_stamp_odom, num_readings=num_readings)

        self.main_app_manager.pf_engine.scan_update(tree_data)

        best_guess = self.main_app_manager.pf_engine.best_particle
        self.main_app_manager.plotter.update_particles(self.main_app_manager.pf_engine.downsample_particles())
        self.main_app_manager.plotter.update_position_estimate(best_guess)
        #

        self.converged = self.main_app_manager.pf_engine.check_convergence()

        # TODO: update num particle label
        self.main_app_manager.control_buttons.set_num_particles(self.main_app_manager.pf_engine.particles.shape[0])

        # # Update whether the particle filter has converged once
        # if self.converged:
        #     self.pf_active = False
        #     self.converge_location = self.pf_engine.best_particle
        #     print("Particle filter has converged")
        #     print("Converged at: " + str(self.converge_location))
        #     pf_converge_time = round(rospy.Time.now().to_sec() - self.pf_start_time, 3)
        #     print("Time to converge: " + str(pf_converge_time) + "s")
        #     # convergence_dist = round(np.linalg.norm(self.pf_engine.best_particle[:2] - self.start_location_estimate), 2)
        #     convergence_dist = round(self.distance_traveled, 2)
        #     print("Distance traveled: " + str(convergence_dist) + "m")
        #     self.qt_window.plotter.update_actual_end_tree(self.converge_location)
        #
        #     converge_loc_str = str(round(self.converge_location[0], 2)) + ", " + str(
        #         round(self.converge_location[1], 2))
        #     file_info_msg = [str(pf_converge_time), str(round(convergence_dist, 2)),
        #                      converge_loc_str]
        #     file_info_msg = '\t'.join(file_info_msg)
        #     pyperclip.copy(file_info_msg)
        #     # print('\t'.join(file_info_msg))
        #     # self.qt_window.console.appendPlainText('\t'.join(file_info_msg))
        #     #             self.qt_window.console.appendPlainText("Converged at: " + str(converge_location))
        #     self.start_stop_button_clicked()

        self.is_processing = False

    def get_tree_data(self):
        if len(self.rgb_image_msgs) == 0 or len(self.depth_image_msgs) == 0:
            return None, None, None, None

        if not np.isclose(self.rgb_image_timestamps[0], self.depth_image_timestamps[0], atol=0.001):
            # get rid of the older of the two messages
            if self.rgb_image_timestamps[0] < self.depth_image_timestamps[0]:
                self.rgb_image_msgs.pop(0)
                self.rgb_image_timestamps.pop(0)
            else:
                self.depth_image_msgs.pop(0)
                self.depth_image_timestamps.pop(0)
            return None, None, None, None
        else:
            rgb_msg = self.rgb_image_msgs.pop(0)
            depth_msg = self.depth_image_msgs.pop(0)
            time_stamp = self.rgb_image_timestamps.pop(0)
            self.depth_image_timestamps.pop(0)

        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
        rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")

        current_msg = {'rgb_image': rgb_image, 'depth_image': depth_image}

        positions, widths, class_estimates = self.main_app_manager.trunk_data_connection.get_trunk_data(current_msg)

        return positions, widths, class_estimates, time_stamp

    def start_pf_continuous(self):
        self.main_app_manager.control_buttons.set_stop()
        self.main_app_manager.change_parameters_button.disable()

        self.main_app_manager.start_location_controls.disable()

        self.pf_continuous_active = True

    def stop_pf_continuous(self):
        self.pf_continuous_active = False
        self.is_processing = False

        self.main_app_manager.control_buttons.set_start()
        self.main_app_manager.change_parameters_button.enable()
        self.main_app_manager.start_location_controls.enable()

    def activate_mode(self):
        self.mode_active = True

        mode_change_button_layout = QHBoxLayout()
        mode_change_button_layout.addWidget(self.main_app_manager.mode_selector)
        mode_change_button_layout.addWidget(self.main_app_manager.change_parameters_button)

        self.main_app_manager.ui_layout.addLayout(mode_change_button_layout)
        self.main_app_manager.ui_layout.addWidget(self.main_app_manager.checkboxes)
        self.main_app_manager.ui_layout.addWidget(self.main_app_manager.start_location_controls)
        self.main_app_manager.ui_layout.addWidget(self.main_app_manager.ros_connect_button)
        self.main_app_manager.ui_layout.addWidget(self.main_app_manager.control_buttons)
        self.main_app_manager.ui_layout.addWidget(self.main_app_manager.queue_size_label)
        self.main_app_manager.ui_layout.addWidget(self.main_app_manager.image_display)
        self.main_app_manager.ui_layout.addWidget(self.main_app_manager.console)

        self.main_app_manager.main_layout.addLayout(self.main_app_manager.ui_layout)
        self.main_app_manager.main_layout.addWidget(self.main_app_manager.plotter)

        self.main_app_manager.control_buttons.startButtonClicked.connect(self.start_pf_continuous)
        self.main_app_manager.control_buttons.stopButtonClicked.connect(self.stop_pf_continuous)
        self.main_app_manager.ros_connect_button.connectButtonClicked.connect(self.connect_to_ros)
        self.main_app_manager.ros_connect_button.disconnectButtonClicked.connect(self.disconnect_from_ros)

        self.main_app_manager.control_buttons.single_step_button.setDisabled(True)

        self.main_app_manager.reset_pf()

    def deactivate_mode(self):
        self.main_app_manager.control_buttons.startButtonClicked.disconnect(self.start_pf_continuous)
        self.main_app_manager.control_buttons.stopButtonClicked.disconnect(self.stop_pf_continuous)
        self.main_app_manager.ros_connect_button.connectButtonClicked.disconnect(self.connect_to_ros)
        self.main_app_manager.ros_connect_button.disconnectButtonClicked.disconnect(self.disconnect_from_ros)

        self.main_app_manager.control_buttons.single_step_button.setDisabled(False)

        self.mode_active = False
        self.ensure_pf_stopped()

    def shutdown_hook(self):
        self.stop_pf_continuous()
        self.disconnect_from_ros()