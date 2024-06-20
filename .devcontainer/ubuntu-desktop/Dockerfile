FROM osrf/ros:humble-desktop-full

ARG CACHEBUST=1

RUN apt-get update \
    && apt-get install -y \
    git \
    python3-pyqt5 \
    python3-pip \
    # libboost-python-dev \
    # libgl1-mesa-glx \
    && rm -rf /var/lib/apt/lists/*

# Add vscode user with same UID and GID as your host system
# (copied from https://code.visualstudio.com/remote/advancedcontainers/add-nonroot-user#_creating-a-nonroot-user)
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Switch from root to user
USER $USERNAME

# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USERNAME

RUN pip install --upgrade pip

RUN pip install \
    matplotlib \
    opencv-python-headless \
    pyqtgraph \
    scipy \
    scikit-image  \
    rosbags \
    utm 

RUN pip install numpy==1.26.4

# Update all packages
# RUN sudo apt update && sudo apt upgrade -y

# Rosdep update
# RUN rosdep update

# Create a directory for external repositories
RUN mkdir -p ~/my_python_packages

# Install map_data_tools
RUN git clone https://github.com/Jostan86/map_data_tools.git ~/my_python_packages/map_data_tools
RUN pip install ~/my_python_packages/map_data_tools

# Make a ros2 workspace
RUN mkdir -p ~/ros2_ws/src
RUN cd ~/ros2_ws/src && git clone https://github.com/Jostan86/pf_orchard_interfaces.git

# Build the workspace
RUN cd ~/ros2_ws && . /opt/ros/humble/setup.sh && colcon build

RUN mkdir -p ~/tmp/runtime-${USER}
RUN chmod -R 700 ~/tmp/runtime-${USER}

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
RUN echo  "export XDG_RUNTIME_DIR=/tmp/runtime-${USER}" >> ~/.bashrc
