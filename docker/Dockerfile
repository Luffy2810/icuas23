FROM nvidia/cuda:11.6.0-cudnn8-devel-ubuntu20.04

# Use ARG - persists only during docker build
# https://github.com/moby/moby/issues/4032#issuecomment-192327844
ARG CATKIN_WORKSPACE=uav_ws
ARG DEBIAN_FRONTEND=noninteractive
ARG HOME=/root
ARG USER=root

# Install all the things to stop docker build from breaking
RUN ln -fs /usr/share/zoneinfo/Europe/Zagreb /etc/localtime && \
    apt-get update && apt-get install -q -y \
    git \
    sudo \
    lsb-release \
    gnupg2 \
    apt-utils \
    dialog \
    tzdata \
    curl \
    keyboard-configuration && \
    dpkg-reconfigure --frontend noninteractive tzdata 

# Install uav_ros_simulation and setup catkin workspace
WORKDIR $HOME
RUN git clone https://github.com/larics/uav_ros_simulation.git
RUN ./uav_ros_simulation/installation/install_and_setup_workspace.sh $CATKIN_WORKSPACE

# Build catkin workspace
WORKDIR $HOME/$CATKIN_WORKSPACE/src
RUN catkin build --limit-status-rate 0.2

# Build ardupilot binaries
RUN sudo ln -sf /usr/bin/python3 /usr/bin/python
WORKDIR $HOME/uav_ros_simulation/firmware/ardupilot
RUN modules/waf/waf-light configure --board sitl
RUN modules/waf/waf-light build --target bin/arducopter

# Prepare for startup
RUN echo "export PATH=\$HOME/.local/bin:\$PATH" >> $HOME/.bashrc
WORKDIR $HOME/uav_ros_simulation/startup/kopterworx_one_flying

ARG HOME=/root
ARG CATKIN_WORKSPACE=uav_ws
ARG USER=root

# Step 1: Install all the needed software packages here
RUN apt-get update && apt-get install -q -y \
    vim

# Step 2: Create the icuas23_competition package and copy its contents
WORKDIR $HOME/$CATKIN_WORKSPACE/src/
RUN git clone https://github.com/larics/icuas23_competition.git

# Step 3: Build the Catkin workspace
RUN catkin build --limit-status-rate 0.2

WORKDIR $HOME
RUN git clone https://github.com/WongKinYiu/yolov7.git
RUN apt-get update && apt-get install ffmpeg libsm6 libxext6  -y	

WORKDIR $HOME
WORKDIR $HOME/yolov7
RUN pip install -r requirements.txt
RUN pip install gdown    
RUN gdown 1rF0Hq7JhosXx8EM1yJu1DXQ6nPctAKtq #best.pt
RUN gdown 1k7jzh2hdD4TSVaW5j6WcsRq1OX35xj4P #classify.pt
RUN gdown 1_ly-r_k6CEObjMZ7kbGSvq22ib7n1Ulx #detect_class.py


WORKDIR $HOME
RUN gdown 1Gsb_VdLHsMRXKma0w3tOJ6h9T2fxJFFn
RUN apt-get install  -q -y unzip
RUN unzip ego-planner.zip -d .

WORKDIR $HOME/$CATKIN_WORKSPACE/src/icuas23_competition/startup/challenge
RUN rm session.yml
RUN gdown 1H4z8KaFO1YenEM66zagDZziMhGIhDGl_
RUN gdown 1sPwxtPzet_6Y1frHSYn-XypPl-nvZ8QL
#RUN chmod +x startup.sh
RUN chmod +x traj_node.py 

RUN  rm -rf /usr/bin/python
RUN sudo ln -sf /usr/bin/python3.8 /usr/bin/python

WORKDIR $HOME
RUN gdown 1G6M1MUaGsyMc_h55pBC4xb4j0O8b2hb- #run.sh
RUN chmod +x run.sh


