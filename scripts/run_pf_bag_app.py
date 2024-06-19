from PyQt5.QtWidgets import QApplication
import sys
from pf_orchard_localization.app_managers import PfAppBags, PfAppCached, PfAppLive


bag_config_file_path = "/home/jostan/OneDrive/Docs/Grad_school/Research/code_projects/pf_orchard_localization/config/parameters_pf_app_bags.yaml"
live_config_file_path = "/home/jostan/OneDrive/Docs/Grad_school/Research/code_projects/pf_orchard_localization/config/parameters_pf_app_live.yaml"
cached_config_file_path = "/home/jostan/OneDrive/Docs/Grad_school/Research/code_projects/pf_orchard_localization/config/parameters_pf_app_cached.yaml"

# Jetson
# bag_config_file_path = "/pf_orchard_localization/particle-filter-app/config/parameters_pf_app_bags_jetson.yaml"
# live_config_file_path = "/home/jostan/OneDrive/Docs/Grad_school/Research/code_projects/pf_orchard_localization/config/parameters_pf_app_live.yaml"
# cached_config_file_path = "/pf_orchard_localization/particle-filter-app/config/parameters_pf_app_cached.yaml"

app = QApplication(sys.argv)
# window = MyMainWindow()
pf_app = PfAppBags(bag_config_file_path)
# pf_app = PfAppLive(live_config_file_path)
# pf_app = PfAppCached(cached_config_file_path)

pf_app.show()

sys.exit(app.exec_())

# 1.4cm/sec