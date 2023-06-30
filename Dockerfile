FROM ros:indigo-ros-base

# use bash to be able to source the ros files
SHELL ["/bin/bash", "-c"]

# upgrade ros packages

RUN apt-get update && \
    apt-get install software-properties-common -y && \ 
    add-apt-repository ppa:openjdk-r/ppa -y && \
    apt-get update
   
RUN apt-get install python-wstool -y && \
    apt-get install ros-indigo-rosjava ros-indigo-actionlib ros-indigo-actionlib-tutorials ros-indigo-genjava -y && \
    apt-get install openjdk-6-jdk openjdk-8-jdk -y && update-java-alternatives --set /usr/lib/jvm/java-1.8.0-openjdk-amd64 && \
    apt-get install build-essential -y

# setup catkin_ws for compiling the lib  
RUN mkdir -p /catkin_ws/src/rosjava_actionlib
WORKDIR /catkin_ws

RUN source /opt/ros/indigo/setup.bash && \
    catkin_init_workspace src && \
    catkin_make

ADD . /catkin_ws/src/rosjava_actionlib

