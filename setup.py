from setuptools import find_packages, setup

package_name = "stream_repub"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=[
        "setuptools",
        "aiohttp>=3.13.3",
        "aiortc>=1.14.0",
        "catkin-pkg>=1.1.0",
        "empy==3.3.4",
        "lark-parser>=0.12.0",
        "numpy==1.26.4",
        "opencv-python>=4.11.0.86",
        "requests>=2.32.5",
    ],
    zip_safe=True,
    maintainer="Nessaj",
    maintainer_email="ksnessaj@hotmail.com",
    description="Republisher for h264 stream via WebRTC to ROS2 topic sensor_msgs/msg/Image",
    license="MIT",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "repub=stream_repub.repub:main",
        ],
    },
)
