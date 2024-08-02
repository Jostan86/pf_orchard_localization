sudo apt-get update
pip install -e /pf_orchard_localization/
pip install -e /home/vscode/my_python_packages/map_data_tools/
cd ~/ros2_ws/src/pf_orchard_interfaces && git pull
cd ~/ros2_ws && colcon build && source install/setup.bash
cd /trunk_width_estimation

#git config --global user.email
#git config --global user.name
# cd /pf_orchard_localization/ && git remote set-url origin https://github.com/Jostan86/particle-filter-app.git