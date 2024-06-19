import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QComboBox, QVBoxLayout, QWidget
from PyQt5.QtGui import QSurfaceFormat

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PyQt App")
        self.setGeometry(100, 100, 400, 300)

        label = QLabel("Hello, PyQt!", self)
        label.move(150, 150)
        
        combo_box = QComboBox(self)
        combo_box.addItem("Python")
        combo_box.addItem("Java")
        combo_box.addItem("C++")
        
        layout = QVBoxLayout()
        layout.addWidget(label)
        layout.addWidget(combo_box)
        
        surface_format = QSurfaceFormat()
        print(surface_format)
        
        central_widget = QWidget(self)
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)
        

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())