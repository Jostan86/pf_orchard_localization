from PyQt5.QtWidgets import QApplication
import sys
from pf_orchard_localization.app_managers import ImageSelectAppManager


image_select_config_file_path = "/home/jostan/OneDrive/Docs/Grad_school/Research/code_projects/pf_orchard_localization/config/parameters_image_select.yaml"
app = QApplication(sys.argv)
# window = MyMainWindow()
image_select_app = ImageSelectAppManager(image_select_config_file_path)
image_select_app.qt_window.show()

sys.exit(app.exec_())