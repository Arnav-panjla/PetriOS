from setuptools import find_packages, setup

package_name = "arm_gazebo_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="robotics",
    maintainer_email="user@example.com",
    description="PetriOS web GUI to Gazebo joint trajectory bridge",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "gui_to_gazebo = arm_gazebo_bridge.bridge_node:main",
            "validate_kinematics = arm_gazebo_bridge.validate_node:main",
            "gazebo_direct_petri = arm_gazebo_bridge.direct_transfer:main",
        ],
    },
)
