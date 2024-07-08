import sys
from PyQt6.QtCore import QThread, pyqtSignal, QMutex, QWaitCondition, QObject
from PyQt6.QtWidgets import QApplication, QLabel, QVBoxLayout, QPushButton, QWidget

# The server thread class
class ServerThread(QThread):
    request_processed = pyqtSignal(int)

    def __init__(self):
        super().__init__()
        self.mutex = QMutex()
        self.condition = QWaitCondition()
        self.request_data = None

    def run(self):
        while True:
            self.mutex.lock()
            self.condition.wait(self.mutex)
            if self.request_data is not None:
                result = self.process_request(self.request_data)
                self.request_processed.emit(result)
                self.request_data = None
            self.mutex.unlock()

    def process_request(self, data):
        # Simulate processing time
        self.sleep(2)
        return data * 2

    def handle_request(self, data):
        self.mutex.lock()
        self.request_data = data
        self.condition.wakeAll()
        self.mutex.unlock()

# The client thread class
class ClientThread(QThread):
    def __init__(self, server):
        super().__init__()
        self.server = server
        self.response = None
        self.mutex = QMutex()
        self.condition = QWaitCondition()
        self.server.request_processed.connect(self.on_request_processed)

    def run(self):
        # Start a request to the server
        self.server.handle_request(10)
        # self.server.request_processed.connect(self.on_request_processed)
        self.wait_for_response()

    def on_request_processed(self, result):
        self.mutex.lock()
        self.response = result
        self.condition.wakeAll()
        self.mutex.unlock()

    def wait_for_response(self):
        self.mutex.lock()
        if self.response is None:
            self.condition.wait(self.mutex)
        response = self.response
        self.mutex.unlock()
        print(f"Client received response: {response}")

# The main application class
class AppDemo(QWidget):
    def __init__(self):
        super().__init__()
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle('PyQt6 Thread Communication')
        self.setGeometry(100, 100, 300, 200)

        layout = QVBoxLayout()
        self.label = QLabel('Press the button to start client-server communication')
        layout.addWidget(self.label)

        self.button = QPushButton('Start')
        self.button.clicked.connect(self.start_threads)
        layout.addWidget(self.button)

        self.setLayout(layout)

    def start_threads(self):
        self.server_thread = ServerThread()
        self.client_thread = ClientThread(self.server_thread)

        self.server_thread.start()
        self.client_thread.start()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    demo = AppDemo()
    demo.show()
    sys.exit(app.exec())

