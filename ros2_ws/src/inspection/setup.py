from setuptools import setup

package_name = "inspection"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/inspection.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Leon Trowsdale",
    maintainer_email="leon.trowsdale@dexory.com",
    description="InspectionNode (Phase 2) — thin ROS wrapper over the pure-Python inspection lib; serves /inspect.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "inspection = inspection.inspection_node:main",
        ],
    },
)
