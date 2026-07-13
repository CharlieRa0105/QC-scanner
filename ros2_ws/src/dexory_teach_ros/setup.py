import os
from glob import glob
from setuptools import find_packages, setup

package_name = "dexory_teach_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "images"), glob("images/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Leon Trowsdale",
    maintainer_email="leon.trowsdale@dexory.com",
    description="Dexory Robot Teach GUI (ROS 2 node) for SR5 arm + linear slider.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "teach = dexory_teach_ros.teach_app:main",
        ],
    },
)
