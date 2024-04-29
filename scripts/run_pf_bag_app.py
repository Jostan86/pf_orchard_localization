from PyQt5.QtWidgets import QApplication
import sys
from pf_orchard_localization.app_managers import PfAppBags


pf_app_bag_config_file_path = "/home/jostan/OneDrive/Docs/Grad_school/Research/code_projects/pf_orchard_localization/config/parameters_pf_app_bags.yaml"
# pf_app_cached_data_config_file_path = "/home/jostan/OneDrive/Docs/Grad_school/Research/code_projects/pf_orchard_localization/config/parameters_pf_app_cached.yaml"
app = QApplication(sys.argv)
# window = MyMainWindow()
pf_bag_app = PfAppBags(pf_app_bag_config_file_path, use_cached_data=False)
pf_bag_app.show()

sys.exit(app.exec_())