# Use official ROS 2 Humble desktop image
# Note: ARM64 support varies by ROS 2 version and image variant
FROM osrf/ros:humble-desktop

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV WORKSPACE=/ros2_ws

# Install build essentials and development tools
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    python3-pip \
    wget \
    curl \
    vim \
    nano \
    htop \
    net-tools \
    iputils-ping \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true
RUN rosdep update

# Create workspace
WORKDIR ${WORKSPACE}
RUN mkdir -p src

# Copy package files first for dependency installation
COPY src/ src/

# Install package dependencies
RUN cd ${WORKSPACE} && \
    rosdep install --from-paths src --ignore-src -r -y

# Source ROS setup in bashrc for interactive sessions
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> ~/.bashrc

# Set up colcon mixins for faster builds
RUN colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml || true
RUN colcon mixin update default || true

# Create entrypoint script
RUN echo '#!/bin/bash\n\
set -e\n\
\n\
# Source ROS 2 setup\n\
source /opt/ros/${ROS_DISTRO}/setup.bash\n\
\n\
# Source workspace setup if it exists\n\
if [ -f "${WORKSPACE}/install/setup.bash" ]; then\n\
    source "${WORKSPACE}/install/setup.bash"\n\
fi\n\
\n\
# Execute the command\n\
exec "$@"' > /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh

# Set the entrypoint
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

# Expose common ROS ports
EXPOSE 11311 11345

# Set working directory
WORKDIR ${WORKSPACE} 