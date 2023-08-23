#!/bin/bash

docker run -it --rm --net=host -e DISPLAY=$DISPLAY -v $1:/root/catkin_ws/src/A-LOAM/ a-loam:latest bash -i -c \
    "source ~/.bashrc; \
    cd /root/catkin_ws/; \
    catkin config \
        --cmake-args \
            -DCMAKE_BUILD_TYPE=Release; \
        catkin build; \
        source devel/setup.bash; \
        roslaunch aloam_velodyne aloam_velodyne_VLP_16.launch rviz:=false"
