from setuptools import setup

package_name = "task_manager"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/task_manager.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Leon Trowsdale",
    maintainer_email="leon.trowsdale@dexory.com",
    description="TaskManager node — mission orchestrator; owns /mission/state + the rescan loop; serves /mission/{plan,execute,abort}.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "task_manager = task_manager.task_manager_node:main",
        ],
    },
)
