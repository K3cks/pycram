#!/bin/bash
sudo systemctl start mongod
source ${PYCRAM_WS}/devel/setup.bash &
roscore &
roslaunch pycram ik_and_description.launch robot:='pr2'&
roslaunch suturo_knowledge suturo_knowledge.launch &
roslaunch knowrob_designator knowrob_designator_service.launch &
roslaunch --wait rvizweb rvizweb.launch config_file:=${PYCRAM_WS}/src/pycram/binder/rviz_configs/pr2_config.json &
cp ${PYCRAM_WS}/src/pycram/binder/webapps.json ${PYCRAM_WS}/src/rvizweb/webapps/app.json 


xvfb-run exec "$@"
