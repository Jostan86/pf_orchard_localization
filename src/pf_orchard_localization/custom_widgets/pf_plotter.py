#!/usr/bin/env python3

import pyqtgraph as pg
from PyQt5.QtCore import pyqtSignal, Qt
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QPushButton
import numpy as np

class ClickablePlotWidget(pg.PlotWidget):
    # This class is for a plot widget that emits a signal when clicked about where it was clicked. It also distinguishes
    # between a normal click and a shift-click

    # Define a custom signal
    clicked = pyqtSignal(float, float, bool)  # Signal to emit x and y coordinates

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.is_repainting = False

    def mousePressEvent(self, event):
        mouse_point = self.plotItem.vb.mapSceneToView(event.pos())
        x = mouse_point.x()
        y = mouse_point.y()

        # Check if Shift key is pressed
        if event.modifiers() & Qt.ShiftModifier:
            self.clicked.emit(x, y, True)
        else:
            self.clicked.emit(x, y, False)

        super().mousePressEvent(event)

    def paintEvent(self, event):
        self.is_repainting = True
        super().paintEvent(event)
        self.is_repainting = False

class PFPlotter(QWidget):
    # Class to handle all the plotting for the particle filter app
    def __init__(self, map_data):
        super().__init__()

        # Indicates whether or not to show tree numbers on the plot
        self.show_nums = False

        # Create a pyqtgraph plot widget with some added functionality
        self.plot_widget = ClickablePlotWidget()

        self.plot_nums_toggle_button = QPushButton("Toggle Numbers")

        main_layout = QVBoxLayout()
        main_layout.addWidget(self.plot_widget)
        main_layout.addWidget(self.plot_nums_toggle_button)
        self.setLayout(main_layout)

        # Set background to white and lock the aspect ratio
        self.plot_widget.setAspectLocked(True, ratio=1)
        self.plot_widget.setBackground('w')

        # Set the map data
        self.all_position_estimates = np.array(map_data['all_position_estimates'])
        self.all_class_estimates = np.array(map_data['all_class_estimates'])
        self.all_object_numbers = map_data['object_numbers']
        self.test_tree_numbers = map_data['test_tree_numbers']
        # Set class to 2 for test trees
        self.all_class_estimates[map_data['test_tree_indexes']] = 2

        # Draw the map
        self.draw_plot()

        self.plot_nums_toggle_button.clicked.connect(self.toggle_show_nums)

        self.particles_save = None

    def toggle_show_nums(self):
        self.show_nums = not self.show_nums
        self.draw_plot(particles=self.particles_save)

    def draw_plot(self, particles=None):
        """Method to draw the map on the plot widget"""

        # Hard-coded row number positions
        row_num_xs = [4.9, 5.5, 6.05, 6.65, 7.4, 7.45, 7.9, 8.65, 9.2, 9.65, 10.25, 10.65, 11.05, 11.6, 12.1, 12.65,
                      13.2]
        row_num_xs = [x - .75 for x in row_num_xs]
        row_num_ys = [4.9, 10.8, 16.5, 22.35, 28.1, 33.3, 39.05, 45.05, 51.05, 56.5, 62.6, 68.25, 73.9, 79.55, 85.6,
                      91.5, 97.3]
        row_num_ys = [y - 1 for y in row_num_ys]
        row_nums = [96+i for i in range(len(row_num_xs))]

        # Clear the plot widget
        self.plot_widget.clear()

        # Get the positions of the trees, posts, and test trees
        tree_positions = self.all_position_estimates[self.all_class_estimates == 0]
        post_positions = self.all_position_estimates[self.all_class_estimates == 1]
        test_tree_positions = self.all_position_estimates[self.all_class_estimates == 2]

        # Set size of dots
        self.dot_size = 12

        # Add data to the plot widget
        self.plot_widget.plot(tree_positions[:, 0], tree_positions[:, 1], pen=None, symbol='o', symbolBrush=(0, 158, 115),
                              symbolSize=self.dot_size, name='Trees')
        self.plot_widget.plot(post_positions[:, 0], post_positions[:, 1], pen=None, symbol='o', symbolBrush=(230, 159, 0),
                              symbolSize=self.dot_size, name='Posts')
        self.plot_widget.plot(test_tree_positions[:, 0], test_tree_positions[:, 1], pen=None, symbol='o',
                              symbolBrush=(6, 180, 233), symbolSize=self.dot_size, name='Test Trees')

        # Add numbers to the trees if show_nums is True, which is toggled by a button in the app
        test_tree_idx = 0
        if self.show_nums:
            for i, (x, y) in enumerate(self.all_position_estimates):
                # tree_num_text = pg.TextItem(
                #     html='<div style="text-align: center"><span style="color: #000000; font-size: 8pt;">{}</span></div>'.format(
                #             self.all_object_numbers[i]), anchor=(1.1, 0.5))
                # tree_num_text.setPos(x, y)
                # self.plot_widget.addItem(tree_num_text)


                # Add test tree numbers
                if self.all_class_estimates[i] == 2:
                    tree_num_text = pg.TextItem(
                        html = '<div style="text-align: center"><span style="color: #000000; font-size: 8pt;">{}</span></div>'.format(
                            self.test_tree_numbers[test_tree_idx]), anchor = (-0.1, 0.5))
                    tree_num_text.setPos(x, y)
                    self.plot_widget.addItem(tree_num_text)
                    test_tree_idx += 1

            # Add row numbers
            for i, (x, y) in enumerate(zip(row_num_xs, row_num_ys)):
                row_num_text = pg.TextItem(
                    html='<div style="text-align: center"><span style="color: #000000; font-size: 15pt;">{}</span></div>'.format(
                        row_nums[i]), anchor=(0.5, 0.5))
                row_num_text.setPos(x, y)
                self.plot_widget.addItem(row_num_text)

        self.particle_plot_item = self.plot_widget.plot([], [], pen=None, symbol='o',
                                                        symbolBrush=(0, 0, 0), symbolSize=2, name='Particles')

        self.actual_position_plot_item = self.plot_widget.plot([], [],
                                                               pen=None,
                                                               symbol='o',
                                                               symbolBrush=(213, 94, 0),
                                                               symbolSize=0.75*self.dot_size,
                                                               name='Actual Position')
        self.update_particles(particles)

    def update_particles(self, particles):
        # Method to update the particles on the plot
        if particles is not None:
            # particles = self.downsample_particles(particles)
            self.particle_plot_item.setData(particles[:, 0], particles[:, 1])
        else:
            self.particle_plot_item.setData([], [])

        self.particles_save = particles

    def downsample_particles(self, particles, max_samples=10000):
        """
        Downsample a 2D array of particles to a maximum number of samples. This is useful for plotting large numbers of
        particles without slowing down the GUI.

        Parameters:
        - particles: 2D numpy array of shape (n, 2)
        - max_samples: int, maximum number of samples after downsampling

        Returns:
        - Downsampled 2D numpy array of particles
        """
        num_particles = particles.shape[0]
        if num_particles <= max_samples:
            return particles

        indices = np.random.choice(num_particles, max_samples, replace=False)
        return particles[indices]

    def update_actual_position(self, actual_position):
        if actual_position is not None:
            self.actual_position_plot_item.setData([actual_position[0]], [actual_position[1]])
        else:
            self.actual_position_plot_item.setData([], [])


class TreatingPFPlotter(PFPlotter):
    def __init__(self, map_data):
        super().__init__(map_data)
    def draw_plot(self, particles=None):
        super().draw_plot(particles)

        self.position_estimate_plot_item = self.plot_widget.plot([], [],
                                                                 pen=None,
                                                                 symbol='o',
                                                                 symbolBrush=(213, 94, 0),
                                                                 symbolSize=self.dot_size,
                                                                 name='Pose Estimate')

        self.in_progress_tree_plot_item = self.plot_widget.plot([], [],
                                                                pen=None,
                                                                symbol='o',
                                                                symbolBrush=(211, 0, 255),
                                                                symbolSize=self.dot_size,
                                                                name='In Progress Tree')

        self.treated_trees_plot_item = self.plot_widget.plot([], [],
                                                             pen=None,
                                                             symbol='o',
                                                             symbolBrush=(138, 187, 248),
                                                             symbolSize=self.dot_size,
                                                             name='Treated Trees')




    def update_position_estimate(self, position_estimate):
        pass
        # if position_estimate is not None:
        #     self.position_estimate_plot_item.setData([position_estimate[0]], [position_estimate[1]])
        # else:
        #     self.position_estimate_plot_item.setData([], [])

    def update_in_progress_tree(self, in_progress_tree_position):
        if in_progress_tree_position is not None:
            self.in_progress_tree_plot_item.setData([in_progress_tree_position[0]], [in_progress_tree_position[1]])
        else:
            self.in_progress_tree_plot_item.setData([], [])

    def update_treated_trees(self, complete_position):
        if complete_position is None:
            self.treated_trees_plot_item.setData([], [])
        elif len(complete_position) == 1:
            complete_position = complete_position[0]
            self.treated_trees_plot_item.setData([complete_position[0]], [complete_position[1]])
        else:
            self.treated_trees_plot_item.setData(complete_position[:, 0], complete_position[:, 1])