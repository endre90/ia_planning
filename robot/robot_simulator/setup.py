from setuptools import setup

package_name = "robot_simulator"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="Endre Erős",
    author_email="endree@chalmers.se",
    maintainer="Endre Erős",
    maintainer_email="endree@chalmers.se",
    keywords=["ROS2", "Intelligent Automation"],
    classifiers=[
        "Intended Audience :: Developers",
        "Programming Language :: Python",
        "Topic :: Software Development",
    ],
    description="Dummies package Intelligent Automation course.",
    license="no_license",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            'robot_simulator = robot_simulator.robot_simulator:main',
            'robot_controller = robot_simulator.robot_controller:main'
        ],
    },
)