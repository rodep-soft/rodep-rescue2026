FROM osrf/ros:jazzy-desktop

# --- Install base tools and ROS packages ---
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y \
    gh git vim less tree fzf tmux fish lsof curl wget usbutils v4l-utils libx264-dev ffmpeg \
    lsb-release gnupg libopencv-dev\
    ros-jazzy-xacro \
    ros-jazzy-moveit \
    ros-jazzy-usb-cam \
    ros-jazzy-dynamixel-sdk \
    ros-jazzy-rosbridge-server \
    ros-jazzy-osrf-testing-tools-cpp \
    ros-jazzy-ffmpeg-image-transport \
    python3-rosdep \
    ccache && \
    rm -rf /var/lib/apt/lists/*

# Enable cache
ENV CCACHE_DIR=/root/.ccache
ENV PATH="/usr/lib/ccache:$PATH"
ENV CCACHE_MAXSIZE=30G

# --- Install Gazebo Harmonic (Separated RUNs for stability) ---
# Layer 1: Add OSRF Key and Repository List
RUN set -eux; \
    wget -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg https://packages.osrfoundation.org/gazebo.gpg; \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# --- Copy the ROS workspace to the container ---
COPY ./ /root/

# Update rosdep
RUN rosdep update

# Ensure universe repository is enabled so common dev packages (flex, clang-tidy, ...) are available
RUN apt-get update && apt-get install -y software-properties-common && \
    add-apt-repository universe || true && \
    apt-get update && \
    # Install dependencies (skip micro-ROS and other keys that are not available in the base image)
    rosdep install --from-paths /root/ros_ws/src -y --ignore-src --skip-keys="opencv4 microcdr microxrcedds_client clang-tidy osrf_testing_tools_cpp"

# This MUST be run in its own step because the repository was just added
RUN apt-get update

# Install Gazebo Harmonic
RUN apt-get install -y gz-harmonic

# --- Configure Shell Defaults and Workdir ---
RUN mkdir -p /root/.config/colcon && \
    echo 'build:' > /root/.config/colcon/defaults.yaml && \
    echo '  args: ['\''--symlink-install'\'']' >> /root/.config/colcon/defaults.yaml && \
    echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc && \
    echo 'source /opt/ros/jazzy/setup.bash' >> /root/.profile

WORKDIR /root/ros_ws

CMD ["bash"]
