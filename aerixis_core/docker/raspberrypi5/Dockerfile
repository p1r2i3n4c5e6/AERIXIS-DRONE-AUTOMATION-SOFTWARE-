FROM ros:humble-ros-base-jammy

ENV ROS_DISTRO=humble
ENV AEROSTACK2_WORKSPACE=/root/aerostack2_ws
ENV AEROSTACK2_PATH=/root/aerostack2_ws/src/aerostack2

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        python3-colcon-common-extensions \
        python3-rosdep \
        ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/aerostack2_ws/src
COPY . ./aerostack2

WORKDIR /root/aerostack2_ws
RUN apt-get update \
    && if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi \
    && rosdep update --rosdistro humble \
    && rosdep install --from-paths src --ignore-src --rosdistro humble -r -y \
    && rm -rf /var/lib/apt/lists/*

RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    CMAKE_POLICY_VERSION_MINIMUM=3.5 MAKEFLAGS="-j1" \
        colcon build --executor sequential --parallel-workers 1 \
        --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
