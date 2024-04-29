from ..custom_widgets import PfMainWindow, ImageDisplay, ImageBrowsingControls, DataFileControls, Console
from PyQt5.QtWidgets import QHBoxLayout, QVBoxLayout, QWidget, QPushButton
import logging

class ImageSelectWindowConstructor(PfMainWindow):
    def __init__(self, parameters_image_select):
        super().__init__()

        self.parameters_image_select = parameters_image_select
        self.init_main_layout()

        logging.debug("Constructed ImageSelectWindowConstructor successfully")
    def init_main_layout(self):

        self.setWindowTitle("Image Select App")

        self.init_window_display_settings()

        self.main_layout = QHBoxLayout()

        self.ui_layout = QVBoxLayout()

        self.image_display = ImageDisplay(num_camera_feeds=2)

        self.image_browsing_controls = ImageBrowsingControls()

        self.data_file_controls = DataFileControls(self.parameters_image_select)

        self.console = Console()

        self.data_file_controls.printConsoleMessage.connect(self.console.print_message)

        self.ui_layout.addWidget(self.image_display)
        self.ui_layout.addWidget(self.image_browsing_controls)
        self.ui_layout.addWidget(self.data_file_controls)
        self.ui_layout.addWidget(self.console)

        self.main_layout.addLayout(self.ui_layout)

        # Create central widget
        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)
        central_widget.setLayout(self.main_layout)

if __name__ == "__main__":
    from PyQt5.QtWidgets import QApplication
    import sys

    app = QApplication(sys.argv)
    window = ImageSelectWindowConstructor()
    window.show()
    sys.exit(app.exec_())