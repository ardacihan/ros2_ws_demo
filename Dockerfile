FROM ros:humble-ros-core

# Install all dependencies in one layer
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-opencv \
    python3-numpy \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-launch \
    v4l-utils \
    && rm -rf /var/lib/apt/lists/*

# Create workspace directory
WORKDIR /ros2_ws/src

# Copy the ROS2 package
COPY image_processor ./image_processor

# Build the package
WORKDIR /ros2_ws
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

# Create a simple startup script
RUN echo '#!/bin/bash\n\
source /opt/ros/humble/setup.bash\n\
source /ros2_ws/install/setup.bash\n\
exec "$@"' > /startup.sh && \
    chmod +x /startup.sh

WORKDIR /ros2_ws

# Default to interactive bash with environment sourced
ENTRYPOINT ["/startup.sh"]
CMD ["bash"]