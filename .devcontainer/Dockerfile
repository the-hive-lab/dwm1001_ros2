FROM mcr.microsoft.com/devcontainers/cpp:ubuntu-22.04

ARG USERNAME=vscode

# Set up ROS 2 repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Set up environment variables
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO humble

# Install ROS 2 and colcon
RUN apt update \
    && DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends \
        ros-humble-ros-core=0.10.0-1* \
        ros-humble-ament-cmake-clang-tidy \
        ros-humble-ament-cmake-clang-format \
        python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Set up ROS 2 overlay sourcing for terminal
RUN echo 'source /opt/ros/humble/setup.bash' >> /home/${USERNAME}/.bashrc

# Install common dependencies
RUN apt update \
    && DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends \
        bash-completion \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /workspace/src && chown -R ${USERNAME}:${USERNAME} /workspace
