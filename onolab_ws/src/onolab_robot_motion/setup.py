from setuptools import setup
import os
from glob import glob

package_name = "onolab_robot_motion"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ubuntu",
    maintainer_email="yusaku_nakajima@ap.eng.osaka-u.ac.jp",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "marker_display = onolab_robot_motion.marker_display:main",
            "grinding_demo = onolab_robot_motion.grinding_demo:main",
        ],
    },
)
