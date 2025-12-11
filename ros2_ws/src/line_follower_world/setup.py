import os
from setuptools import setup

package_name = "line_follower_world"

def package_files(directory, destination_root):
    files_and_destinations = []
    for (path, _, filenames) in os.walk(directory):
        if not filenames:
            continue
        destination = os.path.join(destination_root, path)
        source_files = [os.path.join(path, filename) for filename in filenames]
        files_and_destinations.append((destination, source_files))
    return files_and_destinations


model_files = package_files("models", os.path.join("share", package_name))

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
    ]
    + model_files,
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
