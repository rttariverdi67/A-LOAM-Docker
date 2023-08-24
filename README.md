# A-LOAM-Docker
This repo contains Docker image for [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM) and useful information for working with it.

## Building and launching Docker image
At first you need to build Docker image.
```
cd <path-to-this-repo>
docker build -t a-loam:latest .
```
Then you can run Docker image with our script:
```
xhost +
./run.sh <path-to-A-LOAM-repo>
```
You can get A-LOAM [here](https://github.com/HKUST-Aerial-Robotics/A-LOAM).

## Using Docker image
### Dataset playing
For playing Velodyne VLP-16 dataset you need to connect to the existing Docker container from the second terminal instance:
```
docker exec -it <container-id> bash
```
Then you can play Rosbag:
```
rosbag play <path-to-dataset>
```
### Rosbag recording
You can record Rosbag for saving dataset playing results from the third terminal instance:
```
rosbag record /aft_mapped_to_init /laser_cloud_map /velodyne_cloud_registered
```
*NOTE:* You can record any other topics, but we will use only these ones.
### Getting point clouds
**Use the provided [notebook](bag2pcd.ipynb)**

### Getting trajectory
Per frame trjaectory is available in `/aft_mapped_to_init` topic.

# LOAM preprocessing 

Using ROS-bridge for transfering ros2 bags into ros1 bags: 
for thisyou need to have ROS1 and ROS2 installed on your local or a docker with ros bridg installed. 
To install ros bridg: 
```shell
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
# if you have diffrent version of ros2 repace "foxy" with it
sudo apt install ros-foxy-ros1-bridge 
```
#### Transfer .db3 to .bag
**Steps:** 
1. Open 4 shells (A, B, C, and D). 
2. In shell **A**:
```shell
# Source ROS 1 first:
source /opt/ros/noetic/setup.bash 
# Source ROS 2 next:
source /opt/ros/foxy/setup.bash
# export ros master 
export ROS_MASTER_URI=http://localhost:11311
# Run ros-bridg
ros2 run ros1_bridge dynamic_bridge
``` 
3. In shell **B**: 
```shell
# Source ROS 1:
source /opt/ros/noetic/setup.bash
# Run ros core
roscore
```
4. In shell **C**: 
```shell
# Source ROS 1:
source /opt/ros/noetic/setup.bash
# navigat to the directory you want the bag to be stored in
cd <path>
# record all topics with 
rosbag record -a 
# or record a certain topic or topics with 
rosbag record <topic1_name> <topic2_name> <topic3_name>
```
5. In shell **D**: 
```shell
# Source ROS 2:
source /opt/ros/foxy/setup.bash
# Run the ros2 bag .db3
ros2 bag play <name_of_the_ros2_bag.db3>
```
6. when the bag is finished playing stop the recording the the ros1 bag .bag should be in the dirctory you record in. 



