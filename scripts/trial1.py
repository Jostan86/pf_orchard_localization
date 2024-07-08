
# import sys
# from PyQt6.QtCore import QThread, QMutex, QWaitCondition, pyqtSignal, QObject
# from PyQt6.QtWidgets import QApplication, QLabel, QVBoxLayout, QPushButton, QWidget
import time
# # Worker thread class
# class WorkerThread(QThread):
#     def __init__(self, mutex, condition):
#         super().__init__()
#         self.mutex = mutex
#         self.condition = condition

#     def run(self):
#         print('hi1')
#         # self.mutex.lock()
#         print('hi2')
#         print("Worker: Waiting for main thread to complete the task.")
#         self.condition.wait(self.mutex)  # Wait for the main thread
#         print('hi5')
#         print("Worker: Main thread has completed the task. Continuing work.")
#         self.mutex.unlock()

# # Main application class
# class AppDemo(QWidget):
#     def __init__(self):
#         super().__init__()
#         self.init_ui()

#     def init_ui(self):
#         self.setWindowTitle('PyQt6 Main Thread as Server')
#         self.setGeometry(100, 100, 300, 200)

#         layout = QVBoxLayout()
#         self.label = QLabel('Press the button to simulate main thread action')
#         layout.addWidget(self.label)

#         self.button = QPushButton('Start Action')
#         self.button.clicked.connect(self.perform_action)
#         layout.addWidget(self.button)

#         self.setLayout(layout)

#         # Setup for threading
#         self.mutex = QMutex()
#         self.condition = QWaitCondition()
#         self.worker_thread = WorkerThread(self.mutex, self.condition)

#     def perform_action(self):
#         # Start worker thread
#         self.worker_thread.start()
#         time.sleep(0)
#         print('hi3')
#         # Main thread locks the mutex
#         self.mutex.lock()
#         print('hi4')
#         print("Main Thread: Performing the action...")
        
#         # Simulate some work in the main thread
#         time.sleep(2)
        
#         # Action completed, wake up the worker thread
#         print("Main Thread: Action completed. Waking up the worker thread.")
#         self.condition.wakeOne()
        
#         # Unlock the mutex
#         self.mutex.unlock()

# if __name__ == '__main__':
#     app = QApplication(sys.argv)
#     demo = AppDemo()
#     demo.show()
#     sys.exit(app.exec())

import sys
from PyQt6.QtCore import QThread, QMutex, QWaitCondition, pyqtSignal, QObject, QTimer
from PyQt6.QtWidgets import QApplication, QLabel, QVBoxLayout, QPushButton, QWidget

# Worker thread class
class WorkerThread(QThread):
    def __init__(self, mutex, condition):
        super().__init__()
        self.mutex = mutex
        self.condition = condition

    def run(self):
        self.mutex.lock()
        print("Worker: Waiting for main thread to complete the task.")
        self.condition.wait(self.mutex)  # Wait for the main thread
        print("hi2")
        print("Worker: Main thread has completed the task. Continuing work.")
        self.mutex.unlock()
        print("hi3")

# Main application class
class AppDemo(QWidget):
    def __init__(self):
        super().__init__()
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle('PyQt6 Main Thread as Server')
        self.setGeometry(100, 100, 300, 200)

        layout = QVBoxLayout()
        self.label = QLabel('Press the button to simulate main thread action')
        layout.addWidget(self.label)

        self.button = QPushButton('Start Action')
        self.button.clicked.connect(self.perform_action)
        layout.addWidget(self.button)

        self.setLayout(layout)

        # Setup for threading
        self.mutex = QMutex()
        self.condition = QWaitCondition()
        self.worker_thread = WorkerThread(self.mutex, self.condition)

    def perform_action(self):
        # Start worker thread
        self.worker_thread.start()
        
        # Simulate some work in the main thread
        print("Main Thread: Performing the action...")
        QTimer.singleShot(1000, self.complete_action)

    def complete_action(self):
        # Main thread locks the mutex
        self.mutex.lock()
        print("Main Thread: Action completed. Waking up the worker thread.")
        print('hi1')
        # Wake up the worker thread
        self.condition.wakeAll()
        time.sleep(0.5)
        print("hi4")
        # Unlock the mutex
        self.mutex.unlock()
        # time.sleep(0.5)
        print("hi5")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    demo = AppDemo()
    demo.show()
    sys.exit(app.exec())
