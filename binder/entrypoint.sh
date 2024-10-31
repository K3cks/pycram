#!/bin/bash

source ${PYCRAM_WS}/devel/setup.bash
sudo systemctl start mongod &
roscore &
roslaunch --wait rvizweb rvizweb.launch config_file:=${PYCRAM_WS}/src/pycram/binder/rviz_configs/pr2_config.json &
roslaunch --wait pycram ik_and_description.launch robot:='pr2'&
roslaunch --wait suturo_knowledge suturo_knowledge.launch &
roslaunch --wait knowrob_designator knowrob_designator_service.launch &

cp ${PYCRAM_WS}/src/pycram/binder/webapps.json ${PYCRAM_WS}/src/rvizweb/webapps/app.json

xvfb-run exec "$@"
