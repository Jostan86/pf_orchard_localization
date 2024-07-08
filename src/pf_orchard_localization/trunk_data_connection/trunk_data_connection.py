import numpy as np
import cv2
from typing import Callable, Optional, List
import time
from PyQt6.QtCore import QThread, pyqtSignal, pyqtSlot, QObject, QMutex, QWaitCondition
from PyQt6.QtWidgets import QApplication
import socket
import struct
import pickle
import copy

# Function to only import these if they're needed
def import_trunk_analyzer(width_estimation_config_file_path):
    from trunk_width_estimation import TrunkAnalyzer, TrunkSegmenter, PackagePaths
    config_file = "width_estimation_config_apple.yaml"
    return TrunkAnalyzer(PackagePaths(config_file), combine_segmenter=False), TrunkSegmenter(PackagePaths(config_file))

# class for trunk data connection
class TrunkDataConnection(QThread):
    
    signal_save_calibration_data = pyqtSignal(dict)
    signal_request_processed = pyqtSignal(object)
    signal_segmented_image = pyqtSignal(object, int)
    signal_unfiltered_image = pyqtSignal(object, int)
    signal_original_image = pyqtSignal(object, int)
    signal_print_message = pyqtSignal(str)
    
    def __init__(self,
                 width_estimation_config_file_path: str = None,
                #  seg_image_display_func: Callable[[Optional[np.ndarray]], None] = None,
                #  pre_filtered_segmentation_display_func: Callable[[Optional[np.ndarray]], None] = None,
                #  original_image_display_func: Callable[[Optional[np.ndarray]], None] = None,
                 class_mapping=(1, 2, 0),
                 offset=(0, 0),
                #  message_printer: Callable[[List[str]], None] = None,
                #  emitting_save_calibration_data=False
                 ):

        self.init_trunk_analyzer(width_estimation_config_file_path)

        self.class_mapping = class_mapping
        # self.seg_image_display_func = seg_image_display_func
        # self.pre_filtered_segmentation_display_func = pre_filtered_segmentation_display_func
        # self.original_image_display_func = original_image_display_func
        self.offset = offset
        # self.message_printer = message_printer
        
        self.original_image_display_num = -1
        self.unfiltered_image_display_num = -1
        self.segmented_image_display_num = -1
        
        self.emitting_save_calibration_data = False
        
        self.positions = None
        self.widths = None
        self.class_estimates = None
        self.seg_img = None
        self.x_positions_in_image = None
        self.results_kept = None
        
        self.current_msg = None
        
        self.wait_condition = QWaitCondition()
        self.mutex = QMutex()
        
        super().__init__()
    
    def run(self):
        while True:
            self.mutex.lock()
            self.wait_condition.wait(self.mutex)
            if self.current_msg is not None:
                result = self.get_trunk_data(self.current_msg, return_seg_img=True)
                
                if not self.for_display_only:
                    self.signal_request_processed.emit(result)
                    
                self.current_msg = None
            self.mutex.unlock()
    
    @pyqtSlot(dict)
    def handle_request(self, request_data):
        self.mutex.lock()
        self.current_msg = request_data["current_msg"]
        
        if "for_display_only" in request_data:
            self.for_display_only = request_data["for_display_only"]
        else:
            self.for_display_only = False
        
        self.wait_condition.wakeAll()
        self.mutex.unlock()
        
    def init_trunk_analyzer(self, width_estimation_config_file_path):
        if width_estimation_config_file_path is not None:
            self.trunk_analyzer, self.trunk_segmenter = import_trunk_analyzer(width_estimation_config_file_path)


    def get_trunk_data(self, current_msg, return_seg_img=False):
        results_dict, results = self.trunk_segmenter.get_results(current_msg['rgb_image'])

        if self.unfiltered_image_display_num != -1:
            seg_img_og = results.plot()
        else:
            seg_img_og = None

        self.get_results(current_msg, results_dict, results)

        if self.seg_img is None:
            self.seg_img = current_msg['rgb_image']

        if self.original_image_display_num != -1:
            self.signal_original_image.emit(current_msg['rgb_image'], self.original_image_display_num)

        if self.unfiltered_image_display_num != -1:
            self.signal_unfiltered_image.emit(seg_img_og, self.unfiltered_image_display_num)
            
        if self.segmented_image_display_num != -1:
            self.signal_segmented_image.emit(self.seg_img, self.segmented_image_display_num)

        if not return_seg_img:
            return self.positions, self.widths, self.class_estimates
        else:
            return self.positions, self.widths, self.class_estimates, self.seg_img

    def get_results(self, current_msg, results_dict, results):

        self.positions, self.widths, self.class_estimates, self.x_positions_in_image, self.results_kept = (
            self.trunk_analyzer.get_width_estimation_pf(current_msg['depth_image'], results_dict=results_dict))

        if self.emitting_save_calibration_data:
            calibration_data = copy.deepcopy(current_msg)
            calibration_data['x_positions_in_image'] = copy.deepcopy(self.x_positions_in_image)
            self.signal_save_calibration_data.emit(calibration_data)
                                            
        if self.results_kept is not None:
            self.seg_img = results[self.results_kept].plot()
        else:
            self.seg_img = current_msg['rgb_image']

        if self.class_estimates is not None:
            self.class_estimates = self.remap_classes(self.class_estimates)
    
    def remap_classes(self, class_estimates):
        class_estimates_copy = class_estimates.copy()
        for i, class_num in enumerate(self.class_mapping):
            class_estimates_copy[class_estimates == i] = class_num

        return class_estimates_copy

    def print_messages(self, positions, widths):
        messages = []

        msg_str = "Widths: "
        for width in widths:
            width *= 100
            msg_str += str(round(width, 2)) + "cm,  "
        messages.append(msg_str)
        msg_str = "Positions: "
        for position in positions:
            msg_str += "(" + str(round(position[0], 3)) + ", " + str(round(position[1], 3)) + ") "
        messages.append(msg_str)
        messages.append("---")
        
        message = "\n".join(messages)
        self.signal_print_message.emit(message)
            
    
    def set_emitting_save_calibration_data(self, emitting_save_calibration_data):
        self.emitting_save_calibration_data = emitting_save_calibration_data

    @pyqtSlot(int)
    def set_segmented_image_display_num(self, segmented_image_display_num):
        self.segmented_image_display_num = segmented_image_display_num
    
    @pyqtSlot(int)
    def set_unfiltered_image_display_num(self, unfiltered_image_display_num):
        self.unfiltered_image_display_num = unfiltered_image_display_num
    
    @pyqtSlot(int)
    def set_original_image_display_num(self, original_image_display_num):
        self.original_image_display_num = original_image_display_num

    


class TrunkDataConnectionCachedData(TrunkDataConnection):
    def __init__(self,
                 cached_img_directory,
                 class_mapping=(1, 2, 0),
                 offset=(0, 0),
                ):

        super().__init__(class_mapping=class_mapping, offset=offset)

        self.cached_img_directory = cached_img_directory
    
    def get_trunk_data(self, current_msg, return_seg_img=False):
        
        # print('hi1')
        # print(current_msg)

        if current_msg['data'] is None:
            return None, None, None

        msg_data = current_msg['data']['tree_data']
        self.positions = np.array(msg_data['positions'])
        self.widths = np.array(msg_data['widths'])
        self.class_estimates = np.array(msg_data['classes'], dtype=np.int32)

        self.seg_img = self.load_cached_img(current_msg['timestamp'])

        if self.segmented_image_display_num != -1:
            self.signal_segmented_image.emit(self.seg_img, self.segmented_image_display_num)

        self.class_estimates = self.remap_classes(self.class_estimates)
        self.print_messages(self.positions, self.widths)

        return self.positions, self.widths, self.class_estimates

    def load_cached_img(self, time_stamp):
        time_stamp = str(int(1000*time_stamp))
        file_path = self.cached_img_directory + "/" + time_stamp + ".png"
        img = cv2.imread(file_path)
        return img


class TrunkDataConnectionJetson(TrunkDataConnection):
    def __init__(self,
                 width_estimation_config_file_path: str = None,
                 seg_image_display_func: Callable[[Optional[np.ndarray]], None] = None,
                 pre_filtered_segmentation_display_func: Callable[[Optional[np.ndarray]], None] = None,
                 original_image_display_func: Callable[[Optional[np.ndarray]], None] = None,
                 class_mapping=(1, 2, 0),
                 offset=(0, 0),
                 message_printer: Callable[[List[str]], None] = None):

        super().__init__(width_estimation_config_file_path=None,
                         seg_image_display_func=seg_image_display_func,
                         pre_filtered_segmentation_display_func=pre_filtered_segmentation_display_func,
                         original_image_display_func=original_image_display_func,
                         class_mapping=class_mapping,
                         offset=offset,
                         message_printer=message_printer)

        self.producer_thread = ProducerThread()
        self.producer_thread.image_completed.connect(self.image_data_received)
        self.producer_thread.start()

        self.waiting_for_response = False

        self.positions = None
        self.widths = None
        self.class_estimates = None
        self.seg_img = None

    def get_trunk_data(self, current_msg, return_seg_img=False):
        
        

        self.waiting_for_response = True
        self.producer_thread.send_images(current_msg['rgb_image'], current_msg['depth_image'])

        start_time = time.time()

        while self.waiting_for_response:
            time.sleep(0.005)
            QApplication.processEvents()
            if time.time() - start_time > 2:
                print("Timeout waiting for response")
                if return_seg_img:
                    return None, None, None, None
                else:
                    return None, None, None

        print("Time to get response: ", time.time() - start_time)
        if self.positions is not None:
            self.print_messages(self.positions, self.widths)

        if self.seg_img is None:
            self.seg_img = current_msg['rgb_image']

        if self.original_image_display_func is not None:
            self.original_image_display_func(current_msg['rgb_image'])

        if self.seg_image_display_func is not None:
            self.seg_image_display_func(self.seg_img)

        if return_seg_img:
            return self.positions, self.widths, self.class_estimates, self.seg_img
        else:
            return self.positions, self.widths, self.class_estimates

    def image_data_received(self, positions, widths, class_estimates, seg_img):
        self.positions = positions
        self.widths = widths
        self.class_estimates = class_estimates
        self.seg_img = seg_img

        if self.class_estimates is not None:
            self.class_estimates = self.remap_classes(self.class_estimates)

        # print("Received data from producer thread")

        self.waiting_for_response = False


class ProducerThread(QThread):
    image_completed = pyqtSignal(object, object, object, object)

    def __init__(self, server_address=('localhost', 65432), parent=None):
        super(ProducerThread, self).__init__(parent)
        self.server_address = server_address
        self.running = True
        self.sock = None
        self.queue = []
        self.connected_to_consumer = False

    def run(self):
        while self.running:
            try:
                # Create a socket object
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.connect(self.server_address)

                self.connected_to_consumer = True

                while self.running:
                    if self.queue:
                        rgb_image, depth_image = self.queue.pop(0)

                        # Prepare data for sending
                        data = {
                            'rgb_image': rgb_image,
                            'depth_image': depth_image
                        }
                        serialized_data = pickle.dumps(data)
                        data_length = len(serialized_data)

                        # Send the length of the serialized data first
                        self.sock.sendall(struct.pack("I", data_length))
                        # Send the serialized data
                        self.sock.sendall(serialized_data)

                        # Receive data from the consumer
                        ack_length = struct.unpack("I", self.sock.recv(4))[0]
                        ack_data = b''
                        while len(ack_data) < ack_length:
                            ack_data += self.sock.recv(ack_length - len(ack_data))

                        if ack_data:
                            response = pickle.loads(ack_data)
                            positions = response["positions"]
                            widths = response["widths"]
                            class_estimates = response["class_estimates"]
                            seg_img = response["seg_img"]

                            if positions is not None:
                                self.image_completed.emit(positions, widths, class_estimates, seg_img)
                            else:
                                self.image_completed.emit(None, None, None, None)
                    else:
                        self.msleep(5)  # Sleep for a short period to avoid busy waiting
            except (ConnectionRefusedError, BrokenPipeError):
                print("Connection error, retrying in 5 seconds...")
                self.msleep(5000)
            finally:
                self.connected_to_consumer = False
                if self.sock:
                    self.sock.close()

    @pyqtSlot(np.ndarray, np.ndarray)
    def send_images(self, rgb_image, depth_image):
        self.queue.append((rgb_image, depth_image))

    def stop(self):
        self.connected_to_consumer = False
        self.running = False
        self.quit()
        self.wait()