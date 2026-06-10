from setuptools import find_packages, setup
import os
from glob import glob

package_name = "warehouse_dashboard"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"),  glob("launch/*.py")),
        (os.path.join("share", package_name, "static"),  glob("static/*.html") + glob("static/*.ico") + glob("static/*.png")),
        (os.path.join("share", package_name, "static/js"), glob("static/js/*")),
        (os.path.join("share", package_name, "static/css"), glob("static/css/*")),
    ],
    install_requires=["setuptools", "flask", "flask-socketio", "eventlet"],
    zip_safe=True,
    maintainer="Warehouse Team",
    maintainer_email="warehouse@example.com",
    description="Web monitoring dashboard for multi-robot warehouse",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dashboard_node = warehouse_dashboard.dashboard_node:main",
        ],
    },
)
