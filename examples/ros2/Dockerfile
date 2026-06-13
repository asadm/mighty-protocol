ARG ROS_DISTRO=humble
ARG ARCH=amd64

FROM --platform=linux/${ARCH} ros:${ROS_DISTRO}-ros-base

SHELL ["/bin/bash", "-c"]

RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions \
      python3-pip \
      ros-${ROS_DISTRO}-geometry-msgs \
      ros-${ROS_DISTRO}-launch-ros \
      ros-${ROS_DISTRO}-nav-msgs \
      ros-${ROS_DISTRO}-sensor-msgs \
      ros-${ROS_DISTRO}-std-msgs \
      ros-${ROS_DISTRO}-tf2-ros \
 && rm -rf /var/lib/apt/lists/*

COPY . /opt/mighty-protocol

ENV PYTHONPATH=/opt/mighty-protocol/python

RUN mkdir -p /ros2_ws/src \
 && cp -a /opt/mighty-protocol/examples/ros2/mighty_ros2 /ros2_ws/src/mighty_ros2 \
 && source /opt/ros/${ROS_DISTRO}/setup.bash \
 && cd /ros2_ws \
 && colcon build --symlink-install

RUN printf '#!/bin/bash\nset -e\nsource /opt/ros/%s/setup.bash\nsource /ros2_ws/install/setup.bash\nexec "$@"\n' "${ROS_DISTRO}" > /entrypoint.sh \
 && chmod +x /entrypoint.sh

WORKDIR /ros2_ws
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "mighty_ros2", "mighty_ros2.launch.py"]
