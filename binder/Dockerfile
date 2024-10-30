FROM intel4coro/base-notebook:20.04-noetic-full-xpra

# Set environment
ENV PATH=$PATH:/home/user/.local/bin
ENV PYCRAM_WS=/home/${NB_USER}/workspace/ros
WORKDIR ${PYCRAM_WS}/src/
COPY --chown=${NB_USER}:users . pycram/
RUN vcs import --input pycram/binder/pycram-http.rosinstall --recursive

RUN pip install --requirement ${PYCRAM_WS}/src/pycram/binder/requirements.txt --user 

COPY --chown=${NB_USER}:users binder/me ${PYCRAM_WS}/src/me

USER root
RUN apt-get update
RUN apt install -y ros-noetic-pr2-arm-kinematics


# Build pycram workspace
WORKDIR  ${PYCRAM_WS}
USER root
RUN rosdep update \
  && rosdep install -y --ignore-src --from-paths ./ -r \
  && rosdep fix-permissions
USER ${NB_USER}
RUN catkin build

# Knowrob: install MongoDB
USER root
RUN sudo apt-get install gnupg curl -y
RUN curl -fsSL https://www.mongodb.org/static/pgp/server-8.0.asc | \
   sudo gpg -o /usr/share/keyrings/mongodb-server-8.0.gpg \
   --dearmor
RUN echo "deb [ arch=amd64,arm64 signed-by=/usr/share/keyrings/mongodb-server-8.0.gpg ] https://repo.mongodb.org/apt/ubuntu focal/mongodb-org/8.0 multiverse" | sudo tee /etc/apt/sources.list.d/mongodb-org-8.0.list
RUN sudo apt-get update
RUN sudo apt-get install -y mongodb-org=8.0.0 mongodb-org-database=8.0.0 mongodb-org-server=8.0.0 mongodb-mongosh mongodb-org-mongos=8.0.0 mongodb-org-tools=8.0.0 -y

# Knowrob: install SWI prolog
USER root
RUN sudo apt-get install software-properties-common -y
RUN sudo apt-add-repository ppa:swi-prolog/stable -y
RUN sudo apt-get update
RUN sudo apt-get install swi-prolog -y

WORKDIR ${PYCRAM_WS}/src/
RUN git clone --branch fallschool https://github.com/SUTURO/suturo_knowledge.git
RUN git clone --branch main https://github.com/SUTURO/knowrob.git
RUN git clone --branch tede-simulateable-semantic-map https://github.com/SUTURO/suturo_resources.git
RUN git clone --branch soma-cram https://github.com/hawkina/soma.git
RUN pip install rdflib


RUN pip install service_identity
RUN pip install netifaces
RUN catkin build
# Jupyter Config
COPY --chown=${NB_USER}:users binder/jupyter-config.json /opt/conda/share/jupyter/lab/settings/overrides.json

WORKDIR ${PYCRAM_WS}/src/pycram
RUN git config --global --add safe.directory ${PWD}
COPY --chown=${NB_USER}:users binder/entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["jupyter", "lab", "--allow-root", "--NotebookApp.token=''", "--no-browser", "--ip=0.0.0.0"]
RUN pip install https://raw.githubusercontent.com/yxzhan/jupyterlab-rviz/master/dist/jupyterlab_rviz-0.3.1.tar.gz
