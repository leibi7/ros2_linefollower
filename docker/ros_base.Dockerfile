FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

# Basic tools and ROS build deps
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip python3-colcon-common-extensions python3-rosdep \
    python3-vcstool git wget curl lsb-release \
    ros-humble-gazebo-ros-pkgs ros-humble-xacro ros-humble-cv-bridge \
    ros-humble-image-transport ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher ros-humble-nav-msgs \
    ros-humble-geometry-msgs ros-humble-launch-ros \
    ros-humble-launch python3-opencv python3-numpy xvfb \
    mesa-utils libgl1-mesa-dri libglu1-mesa && \
    rm -rf /var/lib/apt/lists/*

RUN rosdep init || true && rosdep update

WORKDIR /opt/ros2_ws
COPY ros2_ws/src /opt/ros2_ws/src

RUN . /opt/ros/humble/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install

ENV ROS_WS=/opt/ros2_ws
ENV ROS_DISTRO=humble
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Source workspace on entry
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /opt/ros2_ws/install/setup.bash" >> /root/.bashrc

CMD ["bash"]
