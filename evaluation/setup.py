import os
from glob import glob

from setuptools import setup

package_name = "evaluation"

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
            "map_comparison = evaluation.node_map_comparison:main",
            "pose_history = evaluation.node_pose_history:main",
            "pose_to_tf = evaluation.node_pose_to_tf:main",
        ],
    },
)
