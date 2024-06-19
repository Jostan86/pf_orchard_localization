FROM dustynv/ros:noetic-desktop-l4t-r35.4.1

USER root

# Install stuff, if you have any
RUN apt-get update \
    && apt-get install -y \
    git \
    python3-rostopic \
    python3-rosmsg \
    python3-pyqt5 \
    libboost-python-dev \
    ros-noetic-nav-msgs \
    ros-noetic-sensor-msgs \
    libgl1-mesa-glx \
    && rm -rf /var/lib/apt/lists/*

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

# Update all packages
# RUN sudo apt update && sudo apt upgrade -y

# Rosdep update
RUN rosdep update

# Set up environment variables
ENV CATKIN_WS=/home/$USERNAME/catkin_ws
ENV ROS_DISTRO=noetic

# Clone the vision_opencv repository into the workspace
RUN git clone https://github.com/ros-perception/vision_opencv.git -b noetic ${CATKIN_WS}/src/vision_opencv

RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && cd ${CATKIN_WS} && catkin_make"

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source ${CATKIN_WS}/devel/setup.bash" >> ~/.bashrc
RUN echo "export DISPLAY=:0" >> ~/.bashrc
RUN echo "export REMOTE_CONTAINERS_DISPLAY_SOCK=/tmp/.X11-unix/X0" >> ~/.bashrc

USER root