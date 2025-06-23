# Use OSRF's full desktop ROS 2 Jazzy image
FROM osrf/ros:jazzy-desktop-full

# Use bash as default shell
SHELL ["/bin/bash", "-c"]

# Build arguments
ARG USERNAME=minh
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Install dependencies and create user
RUN apt update && apt install -y \
    sudo \
    curl \
    gnupg2 \
    lsb-release \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Delete default user if UID is taken (e.g. Ubuntu noble default user)
RUN if id -u ${USER_UID} > /dev/null 2>&1; then \
        userdel -rf $(getent passwd ${USER_UID} | cut -d: -f1); \
    fi

# Create the new non-root user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Set ROS environment for all users
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc

# Set up colcon default behavior to avoid building tests
RUN mkdir -p /etc/colcon && \
    echo -e "build:\n  symlink-install: true\n  cmake-args:\n    -DCMAKE_BUILD_TYPE=Release" > /etc/colcon/defaults.yaml

# Set working directory
WORKDIR /home/${USERNAME}

# Set default user
USER $USERNAME
ENV SHELL=/bin/bash
CMD [ "bash" ]
