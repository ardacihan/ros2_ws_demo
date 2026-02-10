FROM ros:humble-ros-base

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-opencv \
    python3-numpy \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /ros2_ws

# Copy source code
COPY src/ ./src/

# Build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

# Set up the entrypoint to source both ROS and our workspace automatically
ENTRYPOINT ["bash", "-c", "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && \"$@\"", "--"]

# Default command to launch the nodes
CMD ["ros2", "launch", "image_processor", "start_all.launch.py"]