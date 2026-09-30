import os
from glob import glob

from setuptools import setup

package_name = "moz1_sim_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"),
            glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Moz1 Sim",
    maintainer_email="asr.mobilemanipulation@gmail.com",
    description="Host-side ROS2 glue nodes for the Moz1 Isaac Sim integration.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "joint_name_remap = moz1_sim_bridge.joint_name_remap:main",
            "pc2_to_livox = moz1_sim_bridge.pc2_to_livox:main",
        ],
    },
)
