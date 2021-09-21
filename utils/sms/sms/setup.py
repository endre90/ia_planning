from setuptools import setup

package_name = "sms"

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
    author="Christian Larsen, Endre Erős",
    author_email="christian.larsen@fcc.chalmers.se, endree@chalmers.se",
    maintainer="Endre Erős",
    maintainer_email="endree@chalmers.se",
    keywords=["ROS2", "Volvo Rita"],
    classifiers=[
        "Intended Audience :: Developers",
        "Programming Language :: Python",
        "Topic :: Software Development",
    ],
    description="Scene manipulation service package.",
    license="",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sms = sms.sms:main",
        ],
    },
)
