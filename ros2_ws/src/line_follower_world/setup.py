from setuptools import setup
import os


def package_files(directory):
    paths = []
    for (path, _, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join(path, filename))
    return paths


model_files = package_files("models")

package_name = "line_follower_world"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    package_dir={"": "src"},
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/worlds", ["worlds/line_world.world"]),
        ("share/" + package_name + "/launch", ["launch/world.launch.py"]),
        ("share/" + package_name + "/models", model_files),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ROS Line Follower",
    maintainer_email="dev@linefollower.local",
    description="World definition for line follower demo",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
