from setuptools import find_packages, setup

package_name = "mighty_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/mighty_ros2.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Mighty Camera",
    maintainer_email="hello@mightycamera.com",
    description="ROS 2 publisher wrapper for Mighty Camera SDK streams.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "mighty_ros2_publisher = mighty_ros2.publisher:main",
        ],
    },
)
