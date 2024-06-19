import socket
import cv2
import numpy as np
import struct
import time
import pickle
from PyQt5.QtCore import QThread, pyqtSignal, pyqtSlot
import os

class ProducerThread(QThread):
    acknowledgment_received = pyqtSignal(str)
    image_received = pyqtSignal(np.ndarray)
    image_completed = pyqtSignal(int)

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
                        rgb_image, depth_image, frame_number = self.queue.pop(0)

                        start_time = time.time()
                        
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
                        print("Images sent")

                        # Receive data from the consumer
                        ack_length = struct.unpack("I", self.sock.recv(4))[0]
                        ack_data = b''
                        while len(ack_data) < ack_length:
                            ack_data += self.sock.recv(ack_length - len(ack_data))

                        if ack_data:
                            response = pickle.loads(ack_data)
                            print("time taken", time.time() - start_time)
                            print("response1", response["positions"])
                            print("response2", response["widths"])
                            response_img = response["seg_img"]
                            if response_img is not None:
                                # cv2.imwrite("response_img.png", response_img)
                                # cv2.imshow("response_img", response_img)
                                # cv2.waitKey(1000)
                                self.image_received.emit(response_img)
                            else:
                                self.image_received.emit(rgb_image)
                            self.image_completed.emit(frame_number)
                    else:
                        self.msleep(10)  # Sleep for a short period to avoid busy waiting
            except (ConnectionRefusedError, BrokenPipeError):
                print("Connection error, retrying in 5 seconds...")
                self.msleep(5000)
            finally:
                self.connected_to_consumer = False
                if self.sock:
                    self.sock.close()
                
    @pyqtSlot(np.ndarray, np.ndarray, int)
    def send_images(self, rgb_image, depth_image, frame_number):
        self.queue.append((rgb_image, depth_image, frame_number))           

    def stop(self):
        self.connected_to_consumer = False
        self.running = False
        self.quit()
        self.wait()

    
class ImageSenderThread(QThread):

    def __init__(self, producer_thread, rgb_image_dir, depth_image_dir):
        super(ImageSenderThread, self).__init__()
        self.producer_thread = producer_thread
        self.rgb_image_dir = rgb_image_dir
        self.depth_image_dir = depth_image_dir
        self.sending_images = False
        self.waiting_for_response = False

    def run(self):
        self.sending_images = False
        self.waiting_for_response = False
        
        self.producer_thread.image_completed.connect(self.response_received)
        
        rgb_images = sorted([f for f in os.listdir(self.rgb_image_dir) if f.endswith('.png')])
        depth_images = sorted([f for f in os.listdir(self.depth_image_dir) if f.endswith('.png')])

        if len(rgb_images) == 0 or len(depth_images) == 0:
            print("Failed to load images")
            return

        self.sending_images = True

        for i, (rgb_image_name, depth_image_name) in enumerate(zip(rgb_images, depth_images)):
            if not self.sending_images:
                break

            rgb_image = cv2.imread(os.path.join(self.rgb_image_dir, rgb_image_name))
            depth_image = cv2.imread(os.path.join(self.depth_image_dir, depth_image_name), cv2.IMREAD_UNCHANGED)

            if rgb_image is None or depth_image is None:
                print("Failed to load images")
                break
            
            self.producer_thread.send_images(rgb_image, depth_image, i)
            self.frame_number = i
            self.waiting_for_response = True
            
            while self.waiting_for_response:
                self.msleep(10)
    
    @pyqtSlot(int)
    def response_received(self, frame_number):
        if frame_number == self.frame_number:
            self.waiting_for_response = False
        else:
            self.sending_images = False
            print("Response received for wrong frame")
            for j in range(20):
                print("Response received for wrong frame")
                # time.sleep(1)
            self.waiting_for_response = False
            
    
    def stop(self):
        self.producer_thread.image_completed.disconnect(self.response_received)
        # self.requestInterruption()
        self.sending_images = False
        self.waiting_for_response = False
        self.quit()
        self.wait()