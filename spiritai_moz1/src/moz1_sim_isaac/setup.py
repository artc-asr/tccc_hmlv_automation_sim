import os
from glob import glob

from setuptools import setup

package_name = "moz1_sim_isaac"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Paste-into-Isaac-Script-Editor scripts, shipped for reference.
        (os.path.join("share", package_name, "isaac_scripts"),
            glob("isaac_scripts/*.py")),
        (os.path.join("share", package_name, "config"),
            glob("config/*.json")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Moz1 Sim",
    maintainer_email="asr.mobilemanipulation@gmail.com",
    description="Isaac Sim OmniGraph attach scripts for the Moz1 sim ROS2 contract.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={"console_scripts": []},
)
