import csv
import os
import numpy as np
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QComboBox, QSpinBox, QFileDialog, QMessageBox
from PyQt5.QtCore import pyqtSignal


class PfTestControls(QWidget):
    runAllTestsClicked = pyqtSignal()
    abortAllTestsClicked = pyqtSignal()
    runSelectedTestClicked = pyqtSignal()
    abortSelectedTestClicked = pyqtSignal()

    def __init__(self, main_app_manager):
        super().__init__()

        self.main_app_manager = main_app_manager

        self.run_all_button = QPushButton("Run All Tests")
        self.run_all_button.setToolTip("Start the test regimen")
        self.run_all_button.setMinimumWidth(120)

        self.run_selected_button = QPushButton("Run Selected Test")
        self.run_selected_button.setToolTip("Run the selected test")
        self.run_selected_button.setMinimumWidth(120)

        self.reset_button = QPushButton("Reset")
        self.reset_button.setToolTip("Reset the test regimen")
        self.reset_button.setMinimumWidth(120)

        self.save_button = QPushButton("Save Results")
        self.save_button.setToolTip("Save the test results to a file")
        self.save_button.setMinimumWidth(120)

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
        self.layer_1_layout.addWidget(self.reset_button)
        self.layer_1_layout.addWidget(self.save_button)

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
        self.reset_button.clicked.connect(self.reset_tests)
        self.save_button.clicked.connect(self.save_results)

        self.test_regimen = None

    def run_all_button_clicked(self):
        if not self.tests_loaded:
            return
        if self.run_all_button.text() == "Run All Tests":
            self.runAllTestsClicked.emit()
        else:
            self.abortAllTestsClicked.emit()

    def run_selected_button_clicked(self):
        if not self.tests_loaded:
            return
        if self.run_selected_button.text() == "Run Selected Test":
            self.runSelectedTestClicked.emit()
        else:
            self.abortSelectedTestClicked.emit()

    @property
    def tests_loaded(self):
        if self.test_regimen is None:
            self.main_app_manager.print_message("No tests loaded")
            return False
        if len(self.test_regimen.pf_tests) == 0:
            self.main_app_manager.print_message("No tests loaded")
            return False
        return True

    def set_running_all_tests(self, running_all_tests: bool):
        if running_all_tests:
            self.run_all_button.setText("Abort Tests")
            self.run_all_button.setToolTip("Abort the test regimen")
        else:
            self.run_all_button.setText("Run All Tests")
            self.run_all_button.setToolTip("Start the test regimen")
        self.run_selected_button.setDisabled(running_all_tests)
        self.reset_button.setDisabled(running_all_tests)
        self.save_button.setDisabled(running_all_tests)
        self.load_tests_button.setDisabled(running_all_tests)
        self.tests_per_location_spinbox.setDisabled(running_all_tests)
        self.test_selection_combobox.setDisabled(running_all_tests)


    def set_running_selected_test(self, running_selected_test: bool):
        if running_selected_test:
            self.run_selected_button.setText("Abort Test")
            self.run_selected_button.setToolTip("Abort the selected test")
        else:
            self.run_selected_button.setText("Run Selected Test")
            self.run_selected_button.setToolTip("Run the selected test")
        self.run_all_button.setDisabled(running_selected_test)
        self.reset_button.setDisabled(running_selected_test)
        self.save_button.setDisabled(running_selected_test)
        self.load_tests_button.setDisabled(running_selected_test)
        self.tests_per_location_spinbox.setDisabled(running_selected_test)
        self.test_selection_combobox.setDisabled(running_selected_test)

    def get_selected_test(self):
        return self.test_selection_combobox.currentIndex()

    def get_num_trials_per_location(self):
        return self.tests_per_location_spinbox.value()

    def load_tests_button_clicked(self):
        file_name = QFileDialog.getOpenFileName(self, "Select Test Regimen", filter="*.csv")
        if file_name == "":
            return
        else:
            self.load_test(file_name[0])

    def load_test(self, file_name):
        self.test_regimen = PfTestRegimen(file_name, self.main_app_manager.print_message)

        self.test_selection_combobox.clear()
        for test_name in self.test_regimen.test_names:
            self.test_selection_combobox.addItem(test_name)

    def save_results(self):
        save_path = QFileDialog.getSaveFileName(self, "Save Results", filter="*.csv")
        if save_path == "":
            return
        self.test_regimen.process_results(save_path[0])

    def reset_tests(self):
        if not self.tests_loaded:
            return

        self.test_regimen.reset_tests()

    def update_trial_info(self, trial_time_elapsed, num_particles):
        self.time_elapsed_label.setText("Time Elapsed: " + str(round(trial_time_elapsed, 1)) + "s")
        self.particle_number_label.setText("Particles Count: " + str(num_particles))

    def update_trial_number(self, trial_num):
        self.trial_number_label.setText("Trial: " + str(trial_num))

    def update_test_number(self, test_num):
        self.test_number_label.setText("Test: " + str(test_num))

    def update_convergence_rate(self, convergence_rate_current, convergence_rate_all):
        self.convergence_rate_label.setText("Convergence Rate: Current: " + str(round(convergence_rate_current, 2)) + ", All: " + str(round(convergence_rate_all, 2)))

    def update_test_time(self, test_time_current, test_time_all):
        self.test_time_label.setText("Average Time: Current: " + str(round(test_time_current, 2)) + ", All: " + str(round(test_time_all, 2)))





class PfTestRegimen:
    def __init__(self, file_name, print_message_func):

        self.print_message_func = print_message_func

        self.file_name = file_name
        self.test_names = []
        self.pf_tests = []

        with open(file_name, newline='') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                self.test_names.append(row['test_name'])
                pf_test = PfTest(row['test_name'],
                                 float(row['start_x']),
                                 float(row['start_y']),
                                 float(row['start_width']),
                                 float(row['start_length']),
                                 float(row['start_rotation']),
                                 float(row['orientation_center']),
                                 float(row['orientation_range']),
                                 row['data_file_name'],
                                 float(row['start_time']))

                self.pf_tests.append(pf_test)

        self.num_tests = len(self.test_names)
        self.current_test = 0

        self.num_trials_per_location = 1

    def process_results(self, save_path=None):
        completed_tests = []
        for pf_test in self.pf_tests:
            if pf_test.test_completed:
                completed_tests.append(pf_test)

        avg_convergence_rates = np.zeros(len(completed_tests))
        avg_times_all = np.zeros(len(completed_tests))
        avg_times_converged = np.zeros(len(completed_tests))

        for i in range(len(completed_tests)):
            convergence_rate, avg_time_all, avg_time_converged = completed_tests[i].get_results()
            avg_convergence_rates[i] = convergence_rate
            avg_times_all[i] = avg_time_all
            avg_times_converged[i] = avg_time_converged

        # Calculate the overall average time for trials that converged, removing nan values
        overall_avg_time_converged = np.nanmean(avg_times_converged)

        # Calculate the overall average convergence rate
        overall_avg_convergence_rate = np.mean(avg_convergence_rates)

        # Save the results to a csv file
        if save_path is not None:
            if not save_path.endswith(".csv"):
                save_path = save_path + ".csv"

            # check if the file already exists, if so, have popup to ask if they want to overwrite
            if os.path.exists(save_path):
                msg_box = QMessageBox()
                msg_box.setIcon(QMessageBox.Warning)
                msg_box.setText("File already exists")
                msg_box.setInformativeText("Do you want to overwrite the file?")
                msg_box.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
                msg_box.setDefaultButton(QMessageBox.No)
                ret = msg_box.exec_()
                if ret == QMessageBox.No:
                    return

            with open(save_path, "w") as f:
                writer = csv.writer(f)
                writer.writerow(["Start Location", "Convergence Rate", "Average Time", "Average Time Converged"])
                for i in range(len(completed_tests)):
                    writer.writerow([i, avg_convergence_rates[i], avg_times_all[i], avg_times_converged[i]])

                writer.writerow(["Overall Average Time Converged", overall_avg_time_converged])
                writer.writerow(["Overall Average Convergence Rate", overall_avg_convergence_rate])

        # Print the results to the console
        self.print_message_func("Results:")
        for i in range(len(completed_tests)):
            self.print_message_func("Start Location: {}   Convergence Rate: {}   Average Time: {}   Average Time Converged: {}".format(
                i, avg_convergence_rates[i], avg_times_all[i], avg_times_converged[i]
            ))
        self.print_message_func("Overall Average Time Converged: {}".format(overall_avg_time_converged))
        self.print_message_func("Overall Average Convergence Rate: {}".format(overall_avg_convergence_rate))

    def reset_tests(self):
        for pf_test in self.pf_tests:
            pf_test.reset_results()

    def reset_test(self, test_num):
        self.pf_tests[test_num].reset_results()



class PfTest:
    def __init__(self, test_name, start_x, start_y, start_width, start_length, start_rotation, orientation_center,
                 orientation_range, data_file_name, start_time):
        self.test_name = test_name
        self.start_x = start_x
        self.start_y = start_y
        self.start_width = start_width
        self.start_length = start_length
        self.start_rotation = start_rotation
        self.orientation_center = orientation_center
        self.orientation_range = orientation_range
        self.data_file_name = data_file_name
        self.start_time = start_time

        self.results_distances = []
        self.results_convergence_accuracy = []
        self.results_run_times = []

        self.test_completed = False

    def __repr__(self):
        return f"Test Name: {self.test_name}, Start X: {self.start_x}, Start Y: {self.start_y}, Start Width: {self.start_width}, Start Length: {self.start_length}, Start Rotation: {self.start_rotation}, Orientation Center: {self.orientation_center}, Orientation Range: {self.orientation_range}, Data File Name: {self.data_file_name}, Start Time: {self.start_time}"

    def reset_results(self):
        self.test_completed = False
        self.results_distances = []
        self.results_convergence_accuracy = []
        self.results_run_times = []

    def add_results(self, run_time, correct_convergence, distance_to_converge):
        self.results_distances.append(distance_to_converge)
        self.results_convergence_accuracy.append(correct_convergence)
        self.results_run_times.append(run_time)

    def get_results(self):
        convergences = np.array(self.results_convergence_accuracy)
        run_times = np.array(self.results_run_times)

        # Calculate the convergence rate
        convergence_rate = np.sum(convergences) / len(convergences)

        # Calculate the average time to convergence
        avg_time_all = np.sum(run_times) / len(run_times)

        # Calculate the average time for trials that converged
        avg_time_converged = np.sum(run_times * convergences) / np.sum(convergences)

        return convergence_rate, avg_time_all, avg_time_converged

    def set_completed(self):
        self.test_completed = True
