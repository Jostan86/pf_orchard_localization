import numpy as np
import csv
import time
from collections import defaultdict
from typing import Callable, List, Tuple, Dict


class PfTest:
    """Class to store and process information for a single test"""

    def __init__(self, test_name: str, start_x: float, start_y: float, start_width: float, start_length: float,
                 start_rotation: float, orientation_center: float, orientation_range: float, data_file_name: str, start_time: float):
        self.test_name = test_name
        self.start_x, self.start_y = start_x, start_y
        self.start_width, self.start_length = start_width, start_length
        self.start_rotation, self.orientation_center = start_rotation, orientation_center
        self.orientation_range, self.data_file_name = orientation_range, data_file_name
        self.start_time = start_time
        self.save_file_path = None

        self.reset_results()

    def __repr__(self):
        """Returns a string representation of the test data"""
        return (f"Test Name: {self.test_name}, Start X: {self.start_x}, Start Y: {self.start_y}, "
                f"Start Width: {self.start_width}, Start Length: {self.start_length}, Start Rotation: {self.start_rotation}, "
                f"Orientation Center: {self.orientation_center}, Orientation Range: {self.orientation_range}, "
                f"Data File Name: {self.data_file_name}, Start Time: {self.start_time}")

    def reset_results(self):
        """Reset the results for the test"""
        self.test_completed = False
        self.results_location_errors = []
        self.results_distances_traveled = []
        self.results_convergence_accuracy = []
        self.results_run_times = []

    def add_results(self, run_time: float, correct_convergence: bool, location_error: float, distance_traveled: float):
        """Add the results of a trial to the test"""
        self.results_location_errors.append(location_error)
        self.results_distances_traveled.append(distance_traveled)
        self.results_convergence_accuracy.append(correct_convergence)
        self.results_run_times.append(run_time)

        if self.save_file_path:
            self.add_results_to_file()

    def add_results_to_file(self):
        """Append the latest result to the results file"""
        with open(self.save_file_path, "a", newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                self.test_name, round(self.results_location_errors[-1], 3),
                round(self.results_distances_traveled[-1], 3),
                self.results_convergence_accuracy[-1],
                round(self.results_run_times[-1], 3)
            ])

    def get_results(self):
        """Calculate test results statistics"""
        convergences = np.array(self.results_convergence_accuracy, dtype=bool)
        convergence_rate = np.mean(convergences)

        run_times = np.array(self.results_run_times, dtype=float)
        distances = np.array(self.results_distances_traveled, dtype=float)

        avg_time_all = np.mean(run_times)
        avg_distance_all = np.mean(distances)

        if np.any(convergences):
            converged_times = run_times[convergences]
            converged_distances = distances[convergences]

            return (convergence_rate, avg_time_all, np.mean(converged_times), np.std(converged_times, ddof=1),
                    avg_distance_all, np.mean(converged_distances), np.std(converged_distances, ddof=1))
        return (convergence_rate, avg_time_all, np.nan, np.nan, avg_distance_all, np.nan, np.nan)

    def set_completed(self):
        """Mark the test as completed"""
        self.test_completed = True


class PfTestRegimen:
    """Class to store and process information for a set of tests"""

    def __init__(self, test_info_file_path: str, print_message_func: Callable[[str], None] = print, save_path_base: str = None):
        self.print_message_func = print_message_func
        self.pf_tests: List[PfTest] = []
        self.save_path_base = save_path_base

        with open(test_info_file_path, newline='') as csvfile:
            reader = csv.DictReader(csvfile)
            self.pf_tests = [
                PfTest(row['test_name'], float(row['start_x']), float(row['start_y']), float(row['start_width']),
                       float(row['start_length']), float(row['start_rotation']), float(row['orientation_center']),
                       float(row['orientation_range']), row['data_file_name'], float(row['start_time']))
                for row in reader
            ]

        self.num_tests = len(self.pf_tests)
        self.save_path_all = None

    def initialize_save_files(self, save_path_base: str):
        """Initialize result files with headers"""
        self.save_path_base = save_path_base.rstrip(".csv")
        timestamp = time.strftime("%Y-%m-%d--%H-%M-%S")
        self.save_path_all = f"{self.save_path_base}_{timestamp}_all.csv"
        self.save_path_avg = f"{self.save_path_base}_{timestamp}_avg.csv"

        with open(self.save_path_all, "w", newline='') as f:
            csv.writer(f).writerow(["Test Name", "Location Error", "Distance Traveled", "Convergence Accurate", "Run Time"])

        for test in self.pf_tests:
            test.save_file_path = self.save_path_all

    def process_results(self):
        """Process results and save summarized statistics"""
        completed_tests = [test for test in self.pf_tests if test.test_completed]
        if not completed_tests:
            self.print_message_func("No completed tests to process.")
            return

        results = np.array([test.get_results() for test in completed_tests])

        with open(self.save_path_avg, "w", newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["Start Location", "Convergence Rate", "Average Time", "Average Time (Converged)",
                             "STD Time (Converged)", "Average Distance", "Average Distance (Converged)",
                             "STD Distance (Converged)"])

            for i, result in enumerate(results):
                writer.writerow([i+1] + list(np.round(result, 3)))

            overall_avg_time_converged = np.nanmean(results[:, 2])
            overall_avg_convergence_rate = np.mean(results[:, 0])
            overall_avg_distance_converged = np.nanmean(results[:, 5])

            writer.writerow(["Overall Average Convergence Rate", overall_avg_convergence_rate])
            writer.writerow(["Overall Average Time Converged", overall_avg_time_converged])
            writer.writerow(["Overall Average Distance Converged", overall_avg_distance_converged])
            writer.writerow(["Number of Trials per Test", len(completed_tests[0].results_distances_traveled)])

        self.print_message_func(f"Processed {len(completed_tests)} tests.")
        self.print_message_func(f"Overall Avg Time Converged: {overall_avg_time_converged}")
        self.print_message_func(f"Overall Avg Convergence Rate: {overall_avg_convergence_rate}")
        self.print_message_func(f"Overall Avg Distance Converged: {overall_avg_distance_converged}")

    def reset_tests(self):
        """Reset all tests"""
        for test in self.pf_tests:
            test.reset_results()

    def reset_test(self, test_num: int):
        """Reset a specific test"""
        if 0 <= test_num < len(self.pf_tests):
            self.pf_tests[test_num].reset_results()

class PfTestResultsProcessor:
    """Class to process test results from an 'all' data file and generate an 'avg' summary file."""

    def __init__(self, all_file_path: str):
        """
        Args:
            all_file_path (str): Path to the 'all' CSV file containing individual trial results.
        """
        self.all_file_path = all_file_path
        self.avg_file_path = all_file_path.replace("_all.csv", "_avg.csv")
        self.test_data: Dict[str, List[Tuple[float, float, bool, float]]] = defaultdict(list)

    def load_data(self):
        """Loads data from the 'all' CSV file and stores it in a dictionary grouped by test name."""
        with open(self.all_file_path, newline='') as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    test_name = row["Test Name"]
                    location_error = float(row["Location Error"])
                    distance_traveled = float(row["Distance Traveled"])
                    convergence_accurate = row["Convergence Accurate"].strip().lower() in ("true", "1")
                    run_time = float(row["Run Time"])

                    self.test_data[test_name].append((location_error, distance_traveled, convergence_accurate, run_time))
                except ValueError as e:
                    print(f"Skipping row due to error: {e}")

    def compute_statistics(self):
        """Computes summary statistics for each test and writes to an 'avg' CSV file."""
        with open(self.avg_file_path, "w", newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["Test Name", "Convergence Rate", "Average Time", "Average Time (Converged)", 
                             "STD Time (Converged)", "Average Distance", "Average Distance (Converged)", 
                             "STD Distance (Converged)"])

            for test_name, results in self.test_data.items():
                results_array = np.array(results, dtype=[("location_error", float), 
                                                         ("distance_traveled", float), 
                                                         ("convergence_accurate", bool), 
                                                         ("run_time", float)])

                convergences = results_array["convergence_accurate"]
                convergence_rate = np.mean(convergences)  # Ratio of successful trials

                avg_time_all = np.mean(results_array["run_time"])
                avg_distance_all = np.mean(results_array["distance_traveled"])

                if np.any(convergences):
                    converged_times = results_array["run_time"][convergences]
                    converged_distances = results_array["distance_traveled"][convergences]

                    avg_time_converged = np.mean(converged_times)
                    std_time_converged = np.std(converged_times, ddof=1)

                    avg_distance_converged = np.mean(converged_distances)
                    std_distance_converged = np.std(converged_distances, ddof=1)
                else:
                    avg_time_converged, std_time_converged = np.nan, np.nan
                    avg_distance_converged, std_distance_converged = np.nan, np.nan

                writer.writerow([test_name, round(convergence_rate, 3), round(avg_time_all, 3),
                                 round(avg_time_converged, 3), round(std_time_converged, 5),
                                 round(avg_distance_all, 3), round(avg_distance_converged, 3), 
                                 round(std_distance_converged, 5)])

        print(f"Summary written to: {self.avg_file_path}")

    def process(self):
        """Runs the full processing pipeline."""
        self.load_data()
        self.compute_statistics()


def calculate_overall_stats(avg_file_path: str):
    """Computes overall test statistics from an 'avg' file and appends the results.

    Args:
        avg_file_path (str): Path to the 'avg' CSV file.
    """
    convergence_rates = []
    avg_times_all = []
    avg_times_converged = []
    std_times_converged = []
    avg_distances_all = []
    avg_distances_converged = []
    std_distances_converged = []

    with open(avg_file_path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                convergence_rates.append(float(row["Convergence Rate"]))
                avg_times_all.append(float(row["Average Time"]))
                avg_times_converged.append(float(row["Average Time (Converged)"]) if row["Average Time (Converged)"] else np.nan)
                std_times_converged.append(float(row["STD Time (Converged)"]) if row["STD Time (Converged)"] else np.nan)
                avg_distances_all.append(float(row["Average Distance"]))
                avg_distances_converged.append(float(row["Average Distance (Converged)"]) if row["Average Distance (Converged)"] else np.nan)
                std_distances_converged.append(float(row["STD Distance (Converged)"]) if row["STD Distance (Converged)"] else np.nan)
            except ValueError as e:
                print(f"Skipping row due to error: {e}")

    # Convert to NumPy arrays for efficient computation
    convergence_rates = np.array(convergence_rates)
    avg_times_all = np.array(avg_times_all)
    avg_times_converged = np.array(avg_times_converged, dtype=np.float64)
    std_times_converged = np.array(std_times_converged, dtype=np.float64)
    avg_distances_all = np.array(avg_distances_all)
    avg_distances_converged = np.array(avg_distances_converged, dtype=np.float64)
    std_distances_converged = np.array(std_distances_converged, dtype=np.float64)

    # Compute overall statistics
    overall_avg_convergence_rate = np.mean(convergence_rates)
    overall_avg_time_converged = np.nanmean(avg_times_converged)  # Ignore NaNs
    overall_avg_distance_converged = np.nanmean(avg_distances_converged)

    # Compute number of trials per test from the first test
    num_trials_per_test = len(convergence_rates)  # Assuming each test has the same number of trials

    # Append results to the avg file
    with open(avg_file_path, "a", newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["Overall Average Convergence Rate", round(overall_avg_convergence_rate, 3)])
        writer.writerow(["Overall Average Time Converged", round(overall_avg_time_converged, 3)])
        writer.writerow(["Overall Average Distance Converged", round(overall_avg_distance_converged, 3)])
        writer.writerow(["Number of Trials per Test", num_trials_per_test])

    print(f"Overall stats appended to {avg_file_path}")

if __name__ == "__main__":
    processor = PfTestResultsProcessor("/home/jostan/Documents/2023-Oct-Cached_data/results/test_results_2025-02-18--15-01-51_all.csv")
    processor.process()

    calculate_overall_stats("/home/jostan/Documents/2023-Oct-Cached_data/results/test_results_2025-02-18--15-01-51_avg.csv")
