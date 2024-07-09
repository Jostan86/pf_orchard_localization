FROM this_container:l4t-r36.3.0

USER root

# Add vscode user with same UID and GID as your host system
# (copied from https://code.visualstudio.com/remote/advancedcontainers/add-nonroot-user#_creating-a-nonroot-user)
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a non-root user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && mkdir /home/$USERNAME/.config && chown ${USER_UID}:${USER_GID} /home/$USERNAME/.config

# Set up sudo
RUN apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

# Switch from root to user
USER $USERNAME

# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USERNAME

RUN sudo apt-get update && \
    sudo apt-get install -y --no-install-recommends \
    git \
    python3-pyqt5 \
    python3-pip \
    libgl1-mesa-glx && \
    sudo apt-get clean && \
    sudo rm -rf /var/lib/apt/lists/*

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

# Set up environment variables
ARG ROS2_WS=/home/$USERNAME/ros2_ws
ARG ROS_DISTRO=humble

# Make a ros2 workspace
RUN mkdir -p ${ROS2_WS}/src

# Clone the interface pacakge into the workspace
RUN cd ${ROS2_WS}/src && git clone https://github.com/Jostan86/pf_orchard_interfaces.git

# Build the workspace
RUN cd ~/ros2_ws && . /opt/ros/${ROS_DISTRO}/install/setup.sh && colcon build

RUN mkdir -p ~/tmp/runtime-${USER}
RUN chmod -R 700 ~/tmp/runtime-${USER}

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/install/setup.bash" >> ~/.bashrc
RUN echo "source ${ROS2_WS}/install/setup.bash" >> ~/.bashrc
# RUN echo "export DISPLAY=:0" >> ~/.bashrc
# RUN echo "export REMOTE_CONTAINERS_DISPLAY_SOCK=/tmp/.X11-unix/X0" >> ~/.bashrc
RUN echo  "export XDG_RUNTIME_DIR=/tmp/runtime-${USER}" >> ~/.bashrc

USER root







