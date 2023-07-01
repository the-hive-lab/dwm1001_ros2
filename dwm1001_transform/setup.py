from setuptools import setup

package_name = "dwm1001_transform"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Adam Morrissett",
    maintainer_email="morrissettal2@vcu.edu",
    description="ROS 2 package for transforming points from the dwm1001 frame to the map frame",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dwm1001_transform = dwm1001_transform.dwm1001_transform_node:main"
        ],
    },
)
