#!/usr/bin/env python3

import pyqtgraph as pg
import time
from PyQt6.QtCore import pyqtSignal, Qt, QPointF, QThread, pyqtSlot
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QPushButton
from PyQt6.QtGui import QMouseEvent
import numpy as np
from map_data_tools import MapData

class ClickablePlotWidget(pg.PlotWidget):
    # This class is for a plot widget that emits a signal when clicked about where it was clicked. It also distinguishes
    # between a normal click and a shift-click

    # Define a custom signal
    clicked = pyqtSignal(float, float, bool)  # Signal to emit x and y coordinates

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.is_repainting = False

    def mousePressEvent(self, event: QMouseEvent):
        # Convert QPoint to QPointF
        mouse_point = self.plotItem.vb.mapSceneToView(QPointF(event.pos()))
        x = mouse_point.x()
        y = mouse_point.y()

        # Check if Shift key is pressed
        if event.modifiers() & Qt.KeyboardModifier.ShiftModifier:
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
    def __init__(self, map_data: MapData):
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
        self.all_position_estimates = map_data.all_position_estimates
        self.all_class_estimates = map_data.all_class_estimates
        self.all_object_numbers = map_data.object_numbers
        self.test_tree_numbers = map_data.test_tree_numbers
        # Set class to 2 for test trees
        self.all_class_estimates[map_data.test_tree_indexes] = 2
        
        self.add_nums_thread = None

        # Draw the map
        self.draw_plot()

        self.plot_nums_toggle_button.clicked.connect(self.toggle_show_nums)

        self.particles_save = None

    def toggle_show_nums(self):
        # Check if the thread is running
        
        
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
        
        # Add numbers to the trees if show_nums is True, which is toggled by a button in the app
        if self.show_nums:
            self.add_nums_thread = TextItemWorker(self.all_position_estimates, self.all_object_numbers, self.all_class_estimates, self.test_tree_numbers, row_num_xs, row_num_ys, row_nums)
            self.add_nums_thread.text_item_ready.connect(self.add_text_item)
            self.add_nums_thread.start()
        else:
            if self.add_nums_thread is not None and self.add_nums_thread.is_running:
                self.add_nums_thread.stop()

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

        

        self.particle_plot_item = self.plot_widget.plot([], [], pen=None, symbol='o',
                                                        symbolBrush=(0, 0, 0), symbolSize=2, name='Particles')

        self.actual_position_plot_item = self.plot_widget.plot([], [],
                                                               pen=None,
                                                               symbol='o',
                                                               symbolBrush=(213, 94, 0),
                                                               symbolSize=0.75*self.dot_size,
                                                               name='Actual Position')
        self.update_particles(particles)
        
    def add_text_item(self, item):
        if self.show_nums:
            text_item = pg.TextItem(html=item['html'], anchor=item['anchor'])
            text_item.setPos(item['x'], item['y'])
            self.plot_widget.addItem(text_item)
    
    @pyqtSlot(np.ndarray)
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
    
    @pyqtSlot(np.ndarray)
    def update_actual_position(self, actual_position):
        if actual_position is not None:
            self.actual_position_plot_item.setData([actual_position[0]], [actual_position[1]])
        else:
            self.actual_position_plot_item.setData([], [])


class TreatingPFPlotter(PFPlotter):
    def __init__(self, map_data: MapData):
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

            
class TextItemWorker(QThread):
    text_item_ready = pyqtSignal(dict)

    def __init__(self, all_position_estimates, all_object_numbers, all_class_estimates, test_tree_numbers, row_num_xs, row_num_ys, row_nums):
        super().__init__()
        self.all_position_estimates = all_position_estimates
        self.all_object_numbers = all_object_numbers
        self.all_class_estimates = all_class_estimates
        self.test_tree_numbers = test_tree_numbers
        self.row_num_xs = row_num_xs
        self.row_num_ys = row_num_ys
        self.row_nums = row_nums
        self.is_running = False

    @pyqtSlot()
    def run(self):
        self.is_running = True
        test_tree_idx = 0
        delay_time = 0.005

        for i, (x, y) in enumerate(self.all_position_estimates):
            if not self.is_running:
                break
            
            tree_num_text = {
                'html': '<div style="text-align: center"><span style="color: #000000; font-size: 8pt;">{}</span></div>'.format(self.all_object_numbers[i]),
                'x': x,
                'y': y,
                'anchor': (1.1, 0.5)
            }
            # items.append(tree_num_text)
            self.text_item_ready.emit(tree_num_text)
            

            if self.all_class_estimates[i] == 2:
                test_tree_text = {
                    'html': '<div style="text-align: center"><span style="color: #000000; font-size: 8pt;">{}</span></div>'.format(self.test_tree_numbers[test_tree_idx]),
                    'x': x,
                    'y': y,
                    'anchor': (-0.1, 0.5)
                }
                # items.append(test_tree_text)
                self.text_item_ready.emit(test_tree_text)
                test_tree_idx += 1
            
            time.sleep(delay_time)

        for i, (x, y) in enumerate(zip(self.row_num_xs, self.row_num_ys)):
            if not self.is_running:
                break
            
            row_num_text = {
                'html': '<div style="text-align: center"><span style="color: #000000; font-size: 15pt;">{}</span></div>'.format(self.row_nums[i]),
                'x': x,
                'y': y,
                'anchor': (0.5, 0.5)
            }
            # items.append(row_num_text)
            self.text_item_ready.emit(row_num_text)
            time.sleep(delay_time)
        
        self.is_running = False
    
    def stop(self):
        self.is_running = False
        self.wait()
    
