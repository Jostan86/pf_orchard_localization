from pf_orchard_localization.pf_engine import PFEngine
from map_data_tools import MapData
from pf_orchard_localization.utils import ParametersPf
from pf_orchard_localization.utils.pf_evaluation import PfTestExecutor


pf_parameters_path = "/home/jostan/OneDrive/Docs/Grad_school/Research/code_projects/pf_orchard_localization/config/parameters_pf.yaml"
pf_parameters = ParametersPf()
pf_parameters.load_from_yaml(pf_parameters_path)

test_start_info_path = "/media/jostan/portabits/pf_app_data/test_starts2.csv"
cached_data_files_directory = "/media/jostan/portabits/pf_app_data/pf_cached_data/pf_data"
    
map_data_path = "/media/jostan/portabits/pf_app_data/map_data_jazz_new.json"
map_data = MapData(map_data_path=map_data_path, move_origin=True, origin_offset=(5, 5))

pf_engine = PFEngine(map_data=map_data)

class_mapping = (1, 2, 0)

pf_test_runner = PfTestExecutor(pf_engine=pf_engine,
                                parameters_pf=pf_parameters,
                                test_info_path=test_start_info_path,
                                cached_data_files_dir=cached_data_files_directory,
                                num_trials=2,
                                class_mapping=class_mapping)

pf_test_runner.run_all_tests()

pf_test_runner.process_results(save_path="/media/jostan/portabits/pf_app_data/pf_test_results4.csv")
    