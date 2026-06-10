from setuptools import find_packages, setup
import os
from glob import glob

package_name = "fleet_manager"

setup(
    name=package_name,
    version="2.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"),  glob("launch/*.py")),
        (os.path.join("share", package_name, "config"),  glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Warehouse Team",
    maintainer_email="warehouse@example.com",
    description="Multi-AGV Dynamic Scheduling System for Warehouse Fleet Management — Adaptive task allocation, load-aware strategy switching, and simplified conflict avoidance",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fleet_manager_node = fleet_manager.fleet_manager_node:main",
        ],
    },
)
