# Orchard Localization Using a Particle Filter
This package provides the source code for the particle filter localization system discussed in our paper, "Tree Detection and In-Row Localization for Autonomous Precision Orchard Management." The paper is currently under review, but a pre-print is available [here](https://1drv.ms/b/s!AhPJ6XcTEu5umd0A4t0iqxyoTPey4A?e=BG1mKW). Additionally, this short [demo video](https://www.youtube.com/watch?v=VH3EroQQkak) describes a module we built using an Nvidia Jetson that uses this code to localize in an orchard.

## Table of Contents
1. [Introduction](#introduction)
2. [Overview](#overview)
3. [Installation](#installation)
4. [Usage](#usage)
<!-- 5. [Configuration](#configuration) -->
<!-- 6. [Examples](#examples) -->
<!-- 7. [License](#license) -->
<!-- 8. [Contact](#contact) -->

## Introduction
Precision agriculture (PA) technologies are essential for optimizing resource inputs and maximizing production outputs in the tree fruit industry. While PA has been widely implemented in row crops, its adoption in fruit orchards faces challenges due to the variability in tree types, orchard designs, and the labor-intensive nature of the industry. Autonomous ground robots present a potential solution for implementing PA practices in orchards, enabling selective management decisions for individual trees.

Our framework addresses the challenge of localizing ground robots within high-density fruit tree orchards, where obtaining accurate GNSS measurements is difficult. Our approach uses deep learning to segment tree trunks in RGB-D images and estimate their width. These segmentations and widths are then used to calculate particle weights in a particle filter-based localization system. Our experiments demonstrate that integrating trunk width into the particle update step significantly improves localization accuracy and efficiency, reducing the distance traveled before convergence by 45% and the convergence time by 31%. We validated our system in realistic field experiments in a commercial apple orchard, achieving reliable, tree-level localization.

## Overview 

The [trunk_width_estimation](https://github.com/Jostan86/trunk_width_estimation) package is used by this package to estimate the trunk widths.

There are several versions of the desktop application for different use cases. 

1. **Bag Data:** Loads a ROS2 bag file with RGB-D data from a Realsense D435 and wheel odometry from a ground robot. In the absence of wheel odometry, visual odometry can be used instead, meaning only RGB-D data is required. Additionally, there is an option to either send the RGB-D images to a ROS2 service running in a separate node to be processed, or the processing can be done internally in the app. The former allows the image processing to be done in parallel with the rest
of the processing but it requires the ROS2 trunk width estimation service to be activated. The latter allows for simpler usage but requires the trunk_width_estimation package to be installed in the same environment as the app.
2. **Cached Data:** Loads a set of data that has been saved during a previous usage with a bag file. The data includes the results of the trunk width estimation, the odometry data, and the images of the segmentations (which are included 
for visualization). This was implemented so that fine tuning tests could be run on the particle without having to re-segment the trunk images.
3. **Live Data:** Runs on data streaming from ROS2 topics on a live system. This version is used for real time operation. It requires a ROS2 node setup in the trunk_width_estimation package to be streaming the trunk data, and it's currently
setup to get visual odometry data from another ROS2 node.

## Installation 

### Option 1: VSCode Devcontainer
If familiar, the easiest way to get started is likely using a VSCode devcontainer and associated Docker compose file. A .devcontainer folder is available with options for a container that runs on the Jetson and one on an Ubuntu Desktop. For the desktop, two docker compose files are available, one has the trunk_width_estimation package installed and therefore can run self-contained, while the other requires a ROS2 service for the trunk width estimation.

A GPU and Nvidia Runtime is needed for better processing, install Nvidia runtime from [here](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html). Alternatively, the code can be run on CPU, the ```runtime: nvidia``` argument in the docker compose file will have to be removed however.

Alternatively, the dockerfiles in the docker folder could also be used to make a docker container and development can be done in there. 
### Option 2: Ubuntu 22.04 with ROS2 Humble
To run locally, ROS2 Humble will need to be installed, then a python virtual environment can be setup with the needed dependencies. 

```bash
cd /where/you/want/to/install/venv
python3.10 venv -m trunk_width_venv
source /path/to/venv/bin/activate
```

The following python packages will need to be installed:

```bash
pip install ultralytics scikit-image pydantic matplotlib opencv-python-headless pyqt5 pyqtgraph scipy scikit-image rosbags utm 
```

The numpy version may also have to be set back:
```bash
pip install numpy==1.26.4
```

Install the pf_orchard_localization package with:
```bash
cd /where/you/want/to/install
git clone https://github.com/Jostan86/pf_orchard_localization.git
cd pf_orchard_localization
pip install -e .
```

The [trunk_width_estimation](https://github.com/Jostan86/trunk_width_estimation) and [map_data](https://github.com/Jostan86/map_data_tools) packages will also have to be installed in like manner.
```bash
cd /where/you/want/to/install
git clone https://github.com/Jostan86/trunk_width_estimation.git
cd trunk_width_estimation
pip install -e 
```
```bash
cd /where/you/want/to/install
git clone https://github.com/Jostan86/map_data_tools.git
cd map_data_tools
pip install -e 
```
For sending messages over ROS, the [pf_orchard_interfaces](https://github.com/Jostan86/pf_orchard_interfaces) package will also have to be installed to a ros2 workspace. 
```bash
cd /your/ros2_ws/src/
git clone https://github.com/Jostan86/pf_orchard_interfaces.git
cd ..
colcon build
```
And don't forget to source the workspace:
```bash
source /your/ros2_ws/install.setup.bash
```

## Usage 
Some sample data is available [here](https://1drv.ms/f/s!AhPJ6XcTEu5umc8Ljo8Ds4Z7sdz0BA?e=eBsrgu). There is bag data and cached data available, so those versions of the app can be tested. The rosbag can also be played back for testing the live version of the app. 

### Bag data

#### Paths Running in Devcontainer
If using the dev container, change this line in the docker compose file to point to the directory with the downloaded files.

```/media/jostan/portabits/pf_app_data:/home/vscode/app_data:rw```

Change to:

```/path/to/pf_app_data:/home/vscode/app_data:rw```

#### Paths Running Locally
If running locally, change the ```map_data_path``` and ```data_file_dir``` parameters in the ```config/parameters_pf_app_bags.yaml``` file to match where those were downloaded to. 

The ```pf_config_file_path``` will also need to be changed to point to it's location at ```config/parameters_pf.yaml``` from root.

#### Run the app
 To run the app, use the ```scripts/run_pf_app.py``` script. In this script, make sure the correct config file is uncommented, and uncomment the desired app version. Then the app will open when the script is run.

 #### App usage
 In the app, tooltips are available for most elements to explain their usage. To start the particle filter, press the 'Start' button. Shift click on the plot to change to particle start locations.

<!-- ## Configuration -->

<!-- ## Examples -->

<!-- ## License -->
<!-- MIT License -->
