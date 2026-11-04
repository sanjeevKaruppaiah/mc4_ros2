# ROS 2 Jazzy + the full MC 4.0 stack. Handy on Windows (WSL2 + WSLg shows RViz).
#   docker build -t mc4_ros2 -f docker/Dockerfile .
#   docker run -it --rm --net=host -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
#       -v /mnt/wslg:/mnt/wslg mc4_ros2 ros2 launch mc4_bringup sim.launch.py
FROM ros:jazzy

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-vcstool git \
        ros-jazzy-rviz2 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY src src
COPY deps.repos .
RUN vcs import src < deps.repos \
    && apt-get update && rosdep update \
    && rosdep install --from-paths src --ignore-src -y \
    && rm -rf /var/lib/apt/lists/*
RUN . /opt/ros/jazzy/setup.sh && colcon build --symlink-install

RUN echo 'source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash' >> /root/.bashrc
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && exec \"$@\"", "--"]
CMD ["bash"]
