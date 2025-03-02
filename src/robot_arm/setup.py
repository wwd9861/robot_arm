from setuptools import find_packages, setup

package_name = "robot_arm"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="wwd9861",
    maintainer_email="wwd9861@naver.com",
    description="robot_arm_control",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ik_solver = robot_arm.ik_solver:main",
        ],
    },
)
