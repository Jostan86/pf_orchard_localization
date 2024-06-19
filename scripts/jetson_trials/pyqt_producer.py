import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QLabel, QVBoxLayout, QWidget
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtCore import Qt

from pyqt_producer2 import ProducerThread, ImageSenderThread
import numpy as np
import cv2
import os

class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()

        self.initUI()

    def initUI(self):
        self.setWindowTitle("PyQt5 Producer App")
        self.setGeometry(100, 100, 400, 200)

        self.label = QLabel("Waiting for acknowledgment...", self)
        self.label.setAlignment(Qt.AlignCenter)
        
        self.img_label = QLabel(self)
        self.img_label.setAlignment(Qt.AlignCenter)
        place_holder_image = np.zeros((480, 640, 3), dtype=np.uint8)
        self.update_img_label(place_holder_image)

        self.button = QPushButton("Start Sending Images", self)
        self.button.clicked.connect(self.start_sending_images)

        self.stop_button = QPushButton("Stop Sending Images", self)
        self.stop_button.clicked.connect(self.stop_sending_images)

        layout = QVBoxLayout()
        layout.addWidget(self.label)
        layout.addWidget(self.img_label)
        layout.addWidget(self.button)
        layout.addWidget(self.stop_button)

        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

        self.producer_thread = ProducerThread()
        self.producer_thread.acknowledgment_received.connect(self.update_label)
        self.producer_thread.image_received.connect(self.update_img_label)
        self.producer_thread.start()
        
        self.image_sender_thread = None
        
        #     self.producer_thread.start()
    def start_sending_images(self):
        if self.producer_thread is None or not self.producer_thread.isRunning() or not self.producer_thread.connected_to_consumer:
            print("Producer thread not running or not connected to consumer")
            return
        if self.image_sender_thread is not None:
            if self.image_sender_thread.isRunning():
                print("Image sender thread already running")
                return
            else:
                self.image_sender_thread.start()
                return
            
        rgb_image_dir = '/pf_orchard_localization/apple_images/rgb'
        depth_image_dir = '/pf_orchard_localization/apple_images/depth'
        self.image_sender_thread = ImageSenderThread(self.producer_thread, rgb_image_dir, depth_image_dir)
        self.image_sender_thread.start()
            
    def stop_sending_images(self):
        # self.image_sender_thread.stop()
        if self.image_sender_thread and self.image_sender_thread.isRunning():
            self.image_sender_thread.stop()
        
    def stop_thread(self):
        if self.producer_thread and self.producer_thread.isRunning():
            self.producer_thread.stop()
        self.stop_sending_images()

    def update_label(self, message):
        self.label.setText(f"Received acknowledgment: {message}")
    
    def update_img_label(self, image):
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image_qt = QImage(image_rgb.data, image_rgb.shape[1], image_rgb.shape[0], QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(image_qt)
        self.img_label.setPixmap(pixmap)

    def closeEvent(self, event):
        self.stop_thread()
        event.accept()


        
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
