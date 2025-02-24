from PyQt5.QtWidgets import QApplication
import sys
from pf_orchard_localization import app_managers
import warnings
warnings.simplefilter("error", RuntimeWarning)

import os

import logging
logging.basicConfig(level=logging.INFO)

# package_path = "/home/jostan/OneDrive/Docs/Grad_school/Research/code_projects/trunk_width_estimation"
# os.environ['WIDTH_ESTIMATION_PACKAGE_PATH'] = package_path

# Docker
# bag_config_file_path = "/home/vscode/pf_orchard_localization/config/parameters_pf_app_bags.yaml"
# live_config_file_path = "/home/vscode/pf_orchard_localization/config/parameters_pf_app_live.yaml"
# cached_config_file_path = "/home/vscode/pf_orchard_localization/config/parameters_pf_app_cached.yaml"

# Local Desktop
bag_config_file_path = "/home/jostan/OneDrive/Docs/Grad_school/Research/code_projects/pf_orchard_localization/config/parameters_pf_app_bags.yaml"
live_config_file_path = "/home/jostan/OneDrive/Docs/Grad_school/Research/code_projects/pf_orchard_localization/config/parameters_pf_app_live.yaml"
cached_config_file_path = "/home/jostan/OneDrive/Docs/Grad_school/Research/code_projects/pf_orchard_localization/config/parameters_pf_app_cached.yaml"

app = QApplication(sys.argv)

# ----- Choose the app version to run -----
# pf_app = app_managers.RosBags(bag_config_file_path)
# pf_app = app_managers.Live(live_config_file_path)
pf_app = app_managers.Cached(cached_config_file_path)

pf_app.main_window.show()

sys.exit(app.exec_())
