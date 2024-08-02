import socket
import struct
import pickle
import time
import numpy as np
from typing import List, Callable, Optional
from PyQt5.QtCore import QThread, pyqtSignal, pyqtSlot, QApplication
from .trunk_data_connection import TrunkDataConnection

class TrunkDataConnectionJetson(TrunkDataConnection):
    """
    Class for connecting to the trunk data producer from the trunk_width_estimation package. This isn't currenltly used
    in the system, but is left here in case it is needed in the future.
    """

    def __init__(self,
                 width_estimation_config_file_path: str = None,
                 seg_image_display_func: Callable[[Optional[np.ndarray]], None] = None,
                 pre_filtered_segmentation_display_func: Callable[[Optional[np.ndarray]], None] = None,
                 original_image_display_func: Callable[[Optional[np.ndarray]], None] = None,
                 class_mapping=(1, 2, 0),
                 offset=(0, 0),
                 message_printer: Callable[[List[str]], None] = None):
        """
        Args:
            width_estimation_config_file_path (str, optional): The path to the width estimation config file. Defaults to None.
            seg_image_display_func (Callable[[Optional[np.ndarray]], None], optional): The function to display the segmented image. Defaults to None.
            pre_filtered_segmentation_display_func (Callable[[Optional[np.ndarray]], None], optional): The function to display the pre-filtered segmentation image. Defaults to None.
            original_image_display_func (Callable[[Optional[np.ndarray]], None], optional): The function to display the original image. Defaults to None.
            class_mapping (tuple, optional): The mapping of classes from the trunk width estimation package to this one. Defaults to (1, 2, 0).
            offset (tuple, optional): The offset to apply to the positions. Defaults to (0, 0).
            message_printer (Callable[[List[str]], None], optional): The function to print messages. Defaults to None.
        """
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