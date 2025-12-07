from setuptools import setup

package_name = "line_follower_robot"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    package_dir={"": "src"},
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/urdf", ["urdf/line_follower_bot.urdf.xacro"]),
        ("share/" + package_name + "/launch", ["launch/spawn_robot.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ROS Line Follower",
    maintainer_email="dev@linefollower.local",
    description="Diff drive robot with camera for line following",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": []},
)
