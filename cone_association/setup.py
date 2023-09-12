import os
from glob import glob

from setuptools import setup

package_name = "cone_association"

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
    description="Python approach to SLAM with known car position",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "cone_placement = cone_association.node_cone_placement:main",
            "map_interpolation = cone_association.node_map_interpolation:main",
            "nav_commands = cone_association.node_nav_commands:main",
            "trackdrive_handler = cone_association.node_trackdrive_handler:main",
            "trackdrive_handler_multiple = cone_association.node_trackdrive_handler_multiple:main",
            "trackdrive_handler_single = cone_association.node_trackdrive_handler_single:main",
        ],
    },
)
