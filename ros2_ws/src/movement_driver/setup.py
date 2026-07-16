from setuptools import setup

package_name = "movement_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/movement_driver.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Leon Trowsdale",
    maintainer_email="leon.trowsdale@dexory.com",
    description="MovementDriver node — plays a planned trajectory to ArmDriver; serves /execute_path (execution only).",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "movement_driver = movement_driver.movement_driver_node:main",
        ],
    },
)
