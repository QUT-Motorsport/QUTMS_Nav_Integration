import os
from glob import glob

from setuptools import setup

package_name = "nav_interfaces"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Alastair Bradford",
    maintainer_email="team@qutmotorsport.com",
    description="Mapping, Planning, Control helper interfaces with Nav2",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "cone_placement = nav_interfaces.node_cone_placement:main",
            "boundary_interpolation = nav_interfaces.node_boundary_interpolation:main",
            "nav_commands = nav_interfaces.node_nav_commands:main",
            "trackdrive_handler = nav_interfaces.node_trackdrive_handler:main",
            "trackdrive_handler_multiple = nav_interfaces.node_trackdrive_handler_multiple:main",
            "trackdrive_handler_single = nav_interfaces.node_trackdrive_handler_single:main",
            "trackdrive_handler_plan = nav_interfaces.node_trackdrive_handler_plan:main",
        ],
    },
)
