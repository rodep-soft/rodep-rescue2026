
# Dockerfile
FROM osrf/ros:jazzy-desktop

# --- Install ROS 2 Packages and System Dependencies ---
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y \
    gh \
    git \
    vim \
    less \
    tree \
    fzf \
    tmux \
    fish \
    lsof \
    curl \
    lsb-release \
    gnupg \
    ros-jazzy-xacro \
    ccache && \
    # ros-jazzy-foxglove-bridge \
    rm -rf /var/lib/apt/lists/* # Clean up apt cache 


ENV CCACHE_DIR=/root/.ccache
ENV PATH="/usr/lib/ccache:$PATH"
ENV CCACHE_MAXSIZE=30G

RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    apt-get update && \
    apt-get install gz-harmonic

# --- Configure Shell Defaults and ROS 2 setup ---
# This ensures colcon --symlink-install is default and ROS setup.bash is sourced
RUN mkdir -p /root/.config/colcon && \
    echo 'build:' > /root/.config/colcon/defaults.yaml && \
    echo '  args: ['\''--symlink-install'\'']' >> /root/.config/colcon/defaults.yaml && \
    echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc && \
    echo 'source /opt/ros/jazzy/setup.bash' >> /root/.profile


# Set the working directory for subsequent commands and when the container starts
# This should match the working_dir in docker-compose.yml
WORKDIR /root/ros_ws

COPY ./ros_ws/src ./src/

CMD ["bash"]


# launch foxglove
# ros2 launch foxglove_bridge foxglove_bridge_launch.xml &
