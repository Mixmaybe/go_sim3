import importlib.util
from pathlib import Path


def generate_launch_description():
    launch_path = Path(__file__).with_name("collect_city_cv_dataset.launch.py")
    spec = importlib.util.spec_from_file_location("collect_city_cv_dataset_launch", launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()
