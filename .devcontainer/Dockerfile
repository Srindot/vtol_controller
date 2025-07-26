# Pull the rosgzpx4 image
FROM ghcr.io/srindot/rosgzpx4

# Set non-interactive mode for apt to prevent it from prompting for user input
ENV DEBIAN_FRONTEND=noninteractive

# 1. Fix the expired ROS GPG key FIRST.
#    We assume curl is already in the base image, which is standard for ROS images.
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# 2. NOW that the key is fixed, update the package lists and install all dependencies.
RUN apt-get update && apt-get install -y \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-plotjuggler-ros \
    git \
    --no-install-recommends && \
    rm -rf /var/lib/apt/lists/*

# Set the working directory
WORKDIR /home/rosusr/vtol/src/

# Clone the required repositories
RUN git clone https://github.com/PX4/px4_msgs.git && \
    git clone https://github.com/PX4/px4_ros_com.git

# Give ownership of the user's home directory to the non-root user
RUN chown -R rosusr:rosusr /home/rosusr

# Set the default command to open a bash shell
CMD ["bash"]