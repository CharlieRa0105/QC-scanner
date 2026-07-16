from setuptools import setup

package_name = "path_planner"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/path_planner.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Leon Trowsdale",
    maintainer_email="leon.trowsdale@dexory.com",
    description="PathPlanner node: coverage planning + frame transform + (MoveIt) trajectory.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "path_planner = path_planner.path_planner_node:main",
        ],
    },
)
