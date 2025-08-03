# Start from the ROS Noetic base image
FROM ros:noetic

# Set noninteractive frontend for apt
ENV DEBIAN_FRONTEND=noninteractive

# Update package lists and install necessary packages
RUN apt update && \
    apt install -y tmux git vim \
    ros-noetic-rosserial ros-noetic-rosserial-arduino python3-catkin-tools && \
    rm -rf /var/lib/apt/lists/*

# Source ROS setup
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

# Default command
CMD ["bash"]
