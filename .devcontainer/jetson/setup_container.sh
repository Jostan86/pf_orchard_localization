sudo apt-get update
pip install -e /pf_orchard_localization/particle-filter-app/
pip install -e /pf_orchard_localization/map_data_tools/
#git config --global user.email
#git config --global user.name
cd /pf_orchard_localization/particle-filter-app/ && git remote set-url origin https://github.com/Jostan86/particle-filter-app.git
