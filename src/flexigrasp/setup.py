from glob import glob

from setuptools import find_packages, setup

package_name = "flexigrasp"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/udev", glob("udev/*")),
    ],
    install_requires=["setuptools", "numpy", "pyserial"],
    zip_safe=True,
    maintainer="FlexiGrasp",
    maintainer_email="gilsocojp@gmail.com",
    description="Two-finger FlexiTac tactile sensing node and live viewer.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "flexigrasp_node = flexigrasp.node:main",
            "flexigrasp_gui = flexigrasp.gui:main",
        ],
    },
)
