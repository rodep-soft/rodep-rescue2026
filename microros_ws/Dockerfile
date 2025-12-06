FROM docker.io/osrf/ros:jazzy-desktop-full

# Install dependencies for both Agent and Firmware build
RUN apt-get update && apt-get install -y \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    usbutils \
    # Firmware build tools
    gcc-arm-none-eabi \
    libnewlib-arm-none-eabi \
    build-essential \
    cmake \
    openocd \
    gdb-multiarch \
    # Additional utilities
    vim \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /root/microros_ws

# Copy micro-ROS setup from host (submodule)
# This allows us to track changes to micro_ros_setup_forked in git
COPY microros_ws/src/micro_ros_setup ./src/micro_ros_setup

# Update dependencies using rosdep (as per tutorial)
RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y

# Build micro-ROS tools (as per tutorial)
SHELL ["/bin/bash", "-c"]
RUN . /opt/ros/jazzy/setup.bash && \
    colcon build && \
    . install/local_setup.bash

# Download micro-ROS-Agent packages (create step)
RUN . /opt/ros/jazzy/setup.bash && \
    . install/local_setup.bash && \
    ros2 run micro_ros_setup create_agent_ws.sh

# Build micro-ROS agent (build step)
RUN . /opt/ros/jazzy/setup.bash && \
    . install/local_setup.bash && \
    ros2 run micro_ros_setup build_agent.sh

# Create firmware workspace for STM32 Nucleo F446RE
RUN . /opt/ros/jazzy/setup.bash && \
    . install/local_setup.bash && \
    ros2 run micro_ros_setup create_firmware_ws.sh freertos nucleo_f446re

# Configure ping_pong app with serial transport
RUN . /opt/ros/jazzy/setup.bash && \
    . install/local_setup.bash && \
    ros2 run micro_ros_setup configure_firmware.sh ping_pong -t serial

# Build firmware (this may take some time)
RUN . /opt/ros/jazzy/setup.bash && \
    . install/local_setup.bash && \
    ros2 run micro_ros_setup build_firmware.sh

# Setup entrypoint to source ROS 2 and micro-ROS
RUN printf '#!/bin/bash\n\
set -e\n\
\n\
# Source ROS 2 Jazzy\n\
source /opt/ros/jazzy/setup.bash\n\
\n\
# Source micro-ROS workspace\n\
if [ -f /root/microros_ws/install/local_setup.bash ]; then\n\
    source /root/microros_ws/install/local_setup.bash\n\
fi\n\
\n\
echo "=== micro-ROS Environment ==="\n\
echo "Firmware: /root/microros_ws/firmware/"\n\
echo "Agent ready: ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0"\n\
echo "Flash: ros2 run micro_ros_setup flash_firmware.sh"\n\
echo "============================="\n\
\n\
exec "$@"\n' > /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
