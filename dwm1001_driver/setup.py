from setuptools import setup

package_name = "dwm1001_driver"

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
    description="ROS 2 driver package for Qorvo (previously Decawave) DWM1001",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "passive_tag = dwm1001_driver.passive_tag_node:main",
            "dummy_passive_tag = dwm1001_driver.dummy_passive_tag_node:main",
            "active_tag = dwm1001_driver.active_tag_node:main",
            "dummy_active_tag = dwm1001_driver.dummy_active_tag_node:main",
        ]
    },
)
