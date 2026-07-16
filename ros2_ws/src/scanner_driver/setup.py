from setuptools import setup

package_name = "scanner_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/scanner_driver.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Leon Trowsdale",
    maintainer_email="leon.trowsdale@dexory.com",
    description="ScanningDriver node — MIRACO Plus bridge (interface only until the scanner hardware/SDK lands); serves /scan/{start,stop}, publishes /scan/state.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "scanner_driver = scanner_driver.scanner_driver_node:main",
        ],
    },
)
