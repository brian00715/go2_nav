from setuptools import setup
from glob import glob
import os

package_name = "go2_nav"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob(os.path.join("launch", "*launch.[pxy][yma]*"))),
        (os.path.join("share", package_name, "config"), glob(os.path.join("config", "**"))),
        (os.path.join("share", package_name, "urdf"), glob(os.path.join("urdf", "**"))),
        (os.path.join("share", package_name, "scripts"), glob(os.path.join("scripts", "*.py"))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="unitree",
    maintainer_email="unitree@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dds_forward = go2_nav.dds_forward:main",
            "teleop_relay = go2_nav.teleop_relay:main",
            "odom2tf = go2_nav.odom2tf:main",
            "cmd_vel_to_sport = go2_nav.cmd_vel_to_sport:main",
            "waypoint2nav2 = go2_nav.waypoint2nav2:main",
        ],
    },
)
