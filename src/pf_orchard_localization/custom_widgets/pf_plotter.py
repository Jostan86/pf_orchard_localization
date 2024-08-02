#!/usr/bin/env python3

import pyqtgraph as pg
import time
from PyQt5.QtCore import pyqtSignal, Qt, QPointF, QThread, pyqtSlot
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QPushButton
from PyQt5.QtGui import QMouseEvent
import numpy as np
from map_data_tools import MapData

class ClickablePlotWidget(pg.PlotWidget):
    """
    This is plot widget that emits a signal when clicked about where it was clicked. It also distinguishes
    between a normal click and a shift-click
    """
    
    clicked = pyqtSignal(float, float, bool)  # Signal to emit x and y coordinates

    # def mousePressEvent(self, event: QMouseEvent): # Qt6
    #     # Convert QPoint to QPointF
    #     mouse_point = self.plotItem.vb.mapSceneToView(QPointF(event.pos()))
    #     x = mouse_point.x()
    #     y = mouse_point.y()

    #     # Check if Shift key is pressed
    #     if event.modifiers() & Qt.KeyboardModifier.ShiftModifier:
    #         self.clicked.emit(x, y, True)
    #     else:
    #         self.clicked.emit(x, y, False)

    #     super().mousePressEvent(event)

    def mousePressEvent(self, event):
        """
        Extends the mousePressEvent method to emit a signal when the plot is clicked with the x and y coordinates
        """
        mouse_point = self.plotItem.vb.mapSceneToView(event.pos())
        x = mouse_point.x()
        y = mouse_point.y()

        # Check if Shift key is pressed
        if event.modifiers() & Qt.ShiftModifier:
            self.clicked.emit(x, y, True)
        else:
            self.clicked.emit(x, y, False)

        super().mousePressEvent(event)

class PfPlotter(QWidget):
    """
    Class to handle all the plotting for the particle filter app
    """

    def __init__(self, map_data: MapData):
        """
        Constructor for the PFPlotter class
        
        Args:
            map_data (MapData): An instance of the MapData class that contains all the data for the map
        """
        
        super().__init__()

        # State of whether or not to show tree numbers on the plot
        self.show_nums = False

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

    @pyqtSlot()
    def toggle_show_nums(self):
        """
        Slot to toggle whether or not to show tree numbers on the plot when the button is clicked
        """        
        self.show_nums = not self.show_nums
        self.draw_plot(particles=self.particles_save)

    def draw_plot(self, particles=None):
        """
        Method to draw the map on the plot widget, called when the plot is first created and when the toggle numbers button is clicked. 
        Otherwise the update functions are used to update the data in the plot.
        
        Args:
            particles (np.ndarray): An array of particles to plot on the map
        """
        self.plot_widget.clear()

        # Hard-coded row number positions
        row_num_xs = [4.9, 5.5, 6.05, 6.65, 7.4, 7.45, 7.9, 8.65, 9.2, 9.65, 10.25, 10.65, 11.05, 11.6, 12.1, 12.65,
                      13.2]
        row_num_xs = [x - .75 for x in row_num_xs]
        row_num_ys = [4.9, 10.8, 16.5, 22.35, 28.1, 33.3, 39.05, 45.05, 51.05, 56.5, 62.6, 68.25, 73.9, 79.55, 85.6,
                      91.5, 97.3]
        row_num_ys = [y - 1 for y in row_num_ys]
        row_nums = [96+i for i in range(len(row_num_xs))]
        
        # Start a thread to add tree numbers to the plot if show_nums is True. Put in a thread because if it's a lot of numbers it'll freeze the GUI
        if self.show_nums:
            self.add_nums_thread = TextItemWorker(self.all_position_estimates, self.all_object_numbers, self.all_class_estimates, self.test_tree_numbers, row_num_xs, row_num_ys, row_nums)
            self.add_nums_thread.text_item_ready.connect(self.add_text_item)
            self.add_nums_thread.start()
        else:
            if self.add_nums_thread is not None and self.add_nums_thread.is_running:
                self.add_nums_thread.stop()

        # Get the positions of the trees, posts, and test trees
        # TODO: Make these classes a setting somewhere
        tree_positions = self.all_position_estimates[self.all_class_estimates == 0]
        post_positions = self.all_position_estimates[self.all_class_estimates == 1]
        test_tree_positions = self.all_position_estimates[self.all_class_estimates == 2]

        # Set size of dots
        # TODO: Make this a setting somewhere
        self.dot_size = 12

        # Add data to the plot widget
        # TODO: Make these colors settings somewhere
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
                                                               symbolBrush=(211, 0, 255),
                                                               symbolSize=0.75*self.dot_size,
                                                               name='Actual Position')
        
        self.position_estimate_plot_item = self.plot_widget.plot([], [],
                                                                 pen=None,
                                                                 symbol='o',
                                                                 symbolBrush=(213, 94, 0),
                                                                 symbolSize=self.dot_size,
                                                                 name='Pose Estimate')
        
        self.gnss_estimate_plot_item = self.plot_widget.plot([], [],
                                                                 pen=None,
                                                                 symbol='o',
                                                                 symbolBrush=(0, 0, 0),
                                                                 symbolSize=self.dot_size * 2,
                                                                 name='Pose Estimate')

        
        self.update_particles(particles)
    
    @pyqtSlot(dict)
    def add_text_item(self, item):
        """
        Slot to add a text item to the plot widget
        
        Args:
            item (dict): A dictionary containing the html, x, y, and anchor values for the text item
        """
        if self.show_nums:
            text_item = pg.TextItem(html=item['html'], anchor=item['anchor'])
            text_item.setPos(item['x'], item['y'])
            self.plot_widget.addItem(text_item)
    
    @pyqtSlot(np.ndarray)
    def update_particles(self, particles):
        """
        Slot to update the particles on the plot

        Args:
            particles (np.ndarray): An array of particles to plot on the map
        """
        if particles is not None:
            self.particle_plot_item.setData(particles[:, 0], particles[:, 1])
        else:
            self.particle_plot_item.setData([], [])

        self.particles_save = particles
   
    @pyqtSlot(np.ndarray)
    def update_actual_position(self, actual_position):
        """
        Slot to update the dot representing the actual ground position on the plot (for when using cached data)

        Args:
            actual_position (np.ndarray): An array containing the x and y coordinates of the actual position estimate
        """
        if actual_position is not None:
            self.actual_position_plot_item.setData([actual_position[0]], [actual_position[1]])
        else:
            self.actual_position_plot_item.setData([], [])

    @pyqtSlot(dict)
    def update_gnss_estimate(self, gnss_data):
        """
        Slot to update the dot representing the GNSS estimate of the position on the plot

        Args:
            gnss_data (dict): A dictionary containing the easting and northing values of the GNSS estimate
        """
        if gnss_data is not None:
            easting = gnss_data["easting"]
            northing = gnss_data["northing"]
            self.gnss_estimate_plot_item.setData([easting], [northing])
        else:
            self.gnss_estimate_plot_item.setData([], [])

    @pyqtSlot(np.ndarray)
    def update_position_estimate(self, position_estimate):
        """
        Slot to update the dot representing the best estimate of the position on the plot

        Args:
            position_estimate (np.ndarray): An array containing the x and y coordinates of the position estimate
        """
        if position_estimate is not None:
            self.position_estimate_plot_item.setData([position_estimate[0]], [position_estimate[1]])
        else:
            self.position_estimate_plot_item.setData([], [])


class TextItemWorker(QThread):
    """
    Worker thread to add tree and row number text items to the plot widget
    """

    text_item_ready = pyqtSignal(dict)

    def __init__(self, all_position_estimates, all_object_numbers, all_class_estimates, test_tree_numbers, row_num_xs, row_num_ys, row_nums):
        """
        Constructor for the TextItemWorker class

        Args:
            all_position_estimates (np.ndarray): An array of all the tree positions
            all_object_numbers (np.ndarray): An array of all the tree numbers
            all_class_estimates (np.ndarray): An array of all the tree classifications
            test_tree_numbers (np.ndarray): An array of the test tree numbers
            row_num_xs (list): A list of the x positions for the row numbers
            row_num_ys (list): A list of the y positions for the row numbers
            row_nums (list): A list of the row numbers
        """
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
            
            #TODO: Make these numbers a toggleable setting somewhere
            # tree_num_text = {
            #     'html': '<div style="text-align: center"><span style="color: #000000; font-size: 8pt;">{}</span></div>'.format(self.all_object_numbers[i]),
            #     'x': x,
            #     'y': y,
            #     'anchor': (1.1, 0.5)
            # }
            # # items.append(tree_num_text)
            # self.text_item_ready.emit(tree_num_text)
            

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
    
