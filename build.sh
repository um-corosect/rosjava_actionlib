docker build --tag rosjava_actionlib:latest .
docker run --name rosjava_actionlib -it rosjava_actionlib:latest /bin/bash -c "source devel/setup.bash && catkin_make"

docker cp rosjava_actionlib:/catkin_ws/src/rosjava_actionlib/build/libs/rosjava_actionlib-0.4.6.jar .

docker rm rosjava_actionlib
