from glob import glob
from setuptools import setup

package_name = "robot_patrol"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Dev",
    maintainer_email="dev@example.com",
    description="ROS2 patrol demo application",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "patrol_node = robot_patrol.patrol_node:main",
        ],
    },
)
