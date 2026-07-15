from setuptools import find_packages, setup

package_name = "rail_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Leon Trowsdale",
    maintainer_email="leon.trowsdale@dexory.com",
    description="ROS 2 driver for the linear rail / floor track (mock + Roboteq backends).",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "rail_driver = rail_driver.rail_driver_node:main",
        ],
    },
)
