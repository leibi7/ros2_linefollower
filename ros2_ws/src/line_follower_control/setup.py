from setuptools import setup

package_name = "line_follower_control"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    package_dir={"": "src"},
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/control.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ROS Line Follower",
    maintainer_email="dev@linefollower.local",
    description="Red line following controller node",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "line_follower_node = line_follower_control.line_follower_node:main",
            "goal_monitor_node = line_follower_control.goal_monitor_node:main",
            "celebration_node = line_follower_control.celebration_node:main",
        ],
    },
)
