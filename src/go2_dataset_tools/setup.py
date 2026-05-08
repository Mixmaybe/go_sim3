from glob import glob
import os

from setuptools import find_packages, setup


package_name = "go2_dataset_tools"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [os.path.join("resource", package_name)]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob(os.path.join("launch", "*.launch.py"))),
        (os.path.join("share", package_name, "config"), glob(os.path.join("config", "*.yaml"))),
        (
            os.path.join("share", package_name, "models", "dataset_camera"),
            glob(os.path.join("models", "dataset_camera", "*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ros",
    maintainer_email="kaisar.abutalipovv@gmail.com",
    description="Dataset collection tools for Gazebo Sim city_cv YOLO images.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "collect_city_cv_dataset = go2_dataset_tools.collect_city_cv_dataset:main",
        ],
    },
)
