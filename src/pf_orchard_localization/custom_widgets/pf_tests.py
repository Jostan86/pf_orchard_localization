import csv
import os
import numpy as np
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QComboBox, QSpinBox, QFileDialog, QMessageBox
from PyQt5.QtCore import pyqtSignal, pyqtSlot
from ..utils.pf_evaluation import PfTestRegimen

class PfTestControls(QWidget):
    """
    A widget that allows the user to control the particle filter tests, which run a series of tests to evaluate the
    performance of the particle filter. Used to try out different parameters and see how they affect the filter.
    """
    runAllTestsClicked = pyqtSignal()
    abortAllTestsClicked = pyqtSignal()
    runSelectedTestClicked = pyqtSignal()
    abortSelectedTestClicked = pyqtSignal()

    def __init__(self, main_app_manager):
        """
        Extends QWidget to create a widget that allows the user to control the particle filter tests

        Args:
            main_app_manager (PfAppBags or PfAppCached): The main application manager 
        """
        super().__init__()

        self.main_app_manager = main_app_manager

        self.run_all_button = QPushButton("Run All Tests")
        self.run_all_button.setToolTip("Start the test regimen")
        self.run_all_button.setMinimumWidth(120)

        self.run_selected_button = QPushButton("Run Selected Test")
        self.run_selected_button.setToolTip("Run the selected test")
        self.run_selected_button.setMinimumWidth(120)

        # self.reset_button = QPushButton("Reset")
        # self.reset_button.setToolTip("Reset the test regimen")
        # self.reset_button.setMinimumWidth(120)

        # self.save_button = QPushButton("Save Results")
        # self.save_button.setToolTip("Save the test results to a file")
        # self.save_button.setMinimumWidth(120)
        
        #TODO i think i can just add a row with a save location edit, a choose location button, and a save data checkbox

        self.test_selection_label = QLabel("Test Selection:")
        self.test_selection_label.setToolTip("Select the test to run")

        self.test_selection_combobox = QComboBox()
        self.test_selection_combobox.setToolTip("Select the test to run")

        self.load_tests_button = QPushButton("Load Test Regimen")
        self.load_tests_button.setToolTip("Load a test regimen from a file")
        self.load_tests_button.setFixedWidth(200)

        self.tests_per_location_label = QLabel("Tests Per Location:")
        self.tests_per_location_label.setToolTip("Number of tests to run at each location")

        self.tests_per_location_spinbox = QSpinBox()
        self.tests_per_location_spinbox.setToolTip("Number of tests to run at each location")
        self.tests_per_location_spinbox.setMinimum(1)
        self.tests_per_location_spinbox.setMaximum(100)
        self.tests_per_location_spinbox.setValue(1)
        self.tests_per_location_spinbox.setFixedWidth(50)

        self.test_number_label = QLabel("Test:")
        self.test_number_label.setToolTip("Current test number")
        self.test_number_label.setFixedWidth(80)

        self.trial_number_label = QLabel("Trial:")
        self.trial_number_label.setToolTip("Current trial number")
        self.trial_number_label.setFixedWidth(80)

        self.time_elapsed_label = QLabel("Time Elapsed:")
        self.time_elapsed_label.setToolTip("Time elapsed since trial start")
        self.time_elapsed_label.setFixedWidth(150)

        self.particle_number_label = QLabel("Particles Count:")
        self.particle_number_label.setToolTip("Number of particles currently in the filter")
        self.particle_number_label.setFixedWidth(180)

        self.convergence_rate_label = QLabel("Convergence Rate: Current: 0.0, All: 0.0")
        self.convergence_rate_label.setToolTip("Convergence rate of all the tests")
        self.convergence_rate_label.setFixedWidth(300)

        self.test_time_label = QLabel("Average Time: Current: 0.0, All: 0.0")
        self.test_time_label.setToolTip("Average time of all the tests")
        self.test_time_label.setFixedWidth(300)

        self.main_layout = QVBoxLayout()

        self.layer_1_layout = QHBoxLayout()
        self.layer_1_layout.addWidget(self.run_all_button)
        self.layer_1_layout.addWidget(self.run_selected_button)

        self.layer_2_layout = QHBoxLayout()
        self.layer_2_layout.addWidget(self.test_selection_label)
        self.layer_2_layout.addWidget(self.test_selection_combobox)
        self.layer_2_layout.addWidget(self.load_tests_button)
        self.layer_2_layout.addWidget(self.tests_per_location_label)
        self.layer_2_layout.addWidget(self.tests_per_location_spinbox)

        self.layer_3_layout = QHBoxLayout()
        self.layer_3_layout.addWidget(self.test_number_label)
        self.layer_3_layout.addWidget(self.trial_number_label)
        self.layer_3_layout.addWidget(self.time_elapsed_label)
        self.layer_3_layout.addWidget(self.particle_number_label)

        self.layer_4_layout = QHBoxLayout()
        self.layer_4_layout.addWidget(self.convergence_rate_label)
        self.layer_4_layout.addWidget(self.test_time_label)

        self.main_layout.addLayout(self.layer_1_layout)
        self.main_layout.addLayout(self.layer_2_layout)
        self.main_layout.addLayout(self.layer_3_layout)
        self.main_layout.addLayout(self.layer_4_layout)

        self.setLayout(self.main_layout)

        self.load_tests_button.clicked.connect(self.load_tests_button_clicked)
        self.run_all_button.clicked.connect(self.run_all_button_clicked)
        self.run_selected_button.clicked.connect(self.run_selected_button_clicked)


    def load_pf_test_names(self):
        """
        Load the names of the particle filter tests from the test regimen file, which is a csv file with the test data and the 
        path to it is set in the parameters file.
        """
        pf_test_regimen = PfTestRegimen(test_info_file_path=self.main_app_manager.parameters_data.test_start_info_path)
        
        if len(pf_test_regimen.pf_tests) == 0:
            raise FileNotFoundError("No tests found in the test regimen file or idk something wack")
        
        for pf_test in pf_test_regimen.pf_tests:
            self.test_selection_combobox.addItem(pf_test.test_name)

    @pyqtSlot()
    def run_all_button_clicked(self):
        """
        Slot that is called when the run all button is clicked. Emits the runAllTestsClicked signal if the button text is "Run All Tests",
        or the abortAllTestsClicked signal if the button text is "Abort Tests
        """
        if self.run_all_button.text() == "Run All Tests":
            self.runAllTestsClicked.emit()
        elif self.run_all_button.text() == "Abort Tests":
            self.abortAllTestsClicked.emit()

    @pyqtSlot()
    def run_selected_button_clicked(self):
        """
        Slot that is called when the run selected button is clicked. Emits the runSelectedTestClicked signal if the button text is "Run Selected Test",
        or the abortSelectedTestClicked signal if the button text is "Abort Test
        """
        if self.run_selected_button.text() == "Run Selected Test":
            self.runSelectedTestClicked.emit()
        elif self.run_selected_button.text() == "Abort Test":
            self.abortSelectedTestClicked.emit()


    def set_running_all_tests(self, running_all_tests: bool):
        """
        Set the state of the widget to reflect if the tests are currently running or not
        
        Args:
            running_all_tests (bool): True if the tests are currently running, False otherwise
        """
        if running_all_tests:
            self.run_all_button.setText("Abort Tests")
            self.run_all_button.setToolTip("Abort the test regimen")
        else:
            self.run_all_button.setText("Run All Tests")
            self.run_all_button.setToolTip("Start the test regimen")
        self.run_selected_button.setDisabled(running_all_tests)
        self.load_tests_button.setDisabled(running_all_tests)
        self.tests_per_location_spinbox.setDisabled(running_all_tests)
        self.test_selection_combobox.setDisabled(running_all_tests)


    def set_running_selected_test(self, running_selected_test: bool):
        """
        Set the state of the widget to reflect if the selected test is currently running or not

        Args:
            running_selected_test (bool): True if the selected test is currently running, False otherwise
        """
        if running_selected_test:
            self.run_selected_button.setText("Abort Test")
            self.run_selected_button.setToolTip("Abort the selected test")
        else:
            self.run_selected_button.setText("Run Selected Test")
            self.run_selected_button.setToolTip("Run the selected test")
        self.run_all_button.setDisabled(running_selected_test)
        self.load_tests_button.setDisabled(running_selected_test)
        self.tests_per_location_spinbox.setDisabled(running_selected_test)
        self.test_selection_combobox.setDisabled(running_selected_test)

    def get_selected_test(self):
        """
        Get the index of the selected test in the test selection combobox
        """
        return self.test_selection_combobox.currentIndex()

    def get_num_trials_per_location(self):
        """
        Get the number of trials to run at each location
        """
        return self.tests_per_location_spinbox.value()

    @pyqtSlot()
    def load_tests_button_clicked(self):
        """
        Slot that is called when the load tests button is clicked. Opens a file dialog to select the test regimen file
        """
        file_name = QFileDialog.getOpenFileName(self, "Select Test Regimen", filter="*.csv")
        if file_name == "":
            return
        else:
            self.load_test(file_name[0])

    def set_save_path(self):
        """
        Opens a file dialog to select the path to save the test results 
        """
        save_path = QFileDialog.getSaveFileName(self, "Save Results", filter="*.csv")
        if save_path == "":
            return
        
        save_path = save_path[0]
        
        # check if the file already exists, if so, have popup to ask if they want to overwrite
        if os.path.exists(save_path):
            msg_box = QMessageBox()
            msg_box.setIcon(QMessageBox.Icon.Warning)
            msg_box.setText("File already exists")
            msg_box.setInformativeText("Do you want to overwrite the file?")
            msg_box.setStandardButtons(QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
            msg_box.setDefaultButton(QMessageBox.StandardButton.No)
            ret = msg_box.exec()
            if ret == QMessageBox.StandardButton.No:
                return
        
        return save_path
    
    # TODO: implement this
    def get_save_path(self):
        return None
    
    @pyqtSlot(float, int)
    def update_trial_info(self, trial_time_elapsed, num_particles):
        """
        Update the trial information labels with the current trial time elapsed and number of particles

        Args:
            trial_time_elapsed (float): The time elapsed since the start of the trial
            num_particles (int): The number of particles currently in the filter
        """
        self.time_elapsed_label.setText("Time Elapsed: " + str(round(trial_time_elapsed, 1)) + "s")
        self.particle_number_label.setText("Particles Count: " + str(num_particles))

    @pyqtSlot(int)
    def update_trial_number(self, trial_num):
        """
        Update the trial number label with the current trial number

        Args:
            trial_num (int): The current trial number
        """
        self.trial_number_label.setText("Trial: " + str(trial_num))

    @pyqtSlot(int)
    def update_test_number(self, test_num):
        """
        Update the test number label with the current test number

        Args:
            test_num (int): The current test number
        """
        self.test_number_label.setText("Test: " + str(test_num))

    @pyqtSlot(float, float)
    def update_convergence_rate(self, convergence_rate_current, convergence_rate_all):
        """
        Update the convergence rate labels with the current convergence rate and the average convergence rate

        Args:
            convergence_rate_current (float): The current convergence rate
            convergence_rate_all (float): The average convergence rate
        """
        self.convergence_rate_label.setText("Convergence Rate: Current: " + str(round(convergence_rate_current, 2)) + ", All: " + str(round(convergence_rate_all, 2)))

    def update_test_time(self, test_time_current, test_time_all):
        """
        Update the test time labels with the current test time and the average test time

        Args:
            test_time_current (float): The current test time
            test_time_all (float): The average test time
        """
        self.test_time_label.setText("Average Time: Current: " + str(round(test_time_current, 2)) + ", All: " + str(round(test_time_all, 2)))


    @pyqtSlot(float, float, float)
    def update_trial_results(self, trial_convergence_rate, trial_avg_time_all, trial_avg_time_converged):
        """
        Update the trial results with the convergence rate and average time for the trial

        Args:
            trial_convergence_rate (float): The convergence rate for the trial
            trial_avg_time_all (float): The average time for the trial
            trial_avg_time_converged (float): The average time for the trial for the converged tests
        """
        self.update_convergence_rate(trial_convergence_rate, 0)
        self.update_test_time(trial_avg_time_all, trial_avg_time_converged)
