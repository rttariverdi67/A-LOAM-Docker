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
Point clouds will be contained in the `/aft_mapped_to_init` and `/laser_cloud_map` topics after recording process. Please, use the `/aft_mapped_to_init` topic for getting per frame point clouds and the last message from `/laser_cloud_map` to get the whole map of the scene.

You can use this command to save point clouds:
```
rosrun pcl_ros bag_to_pcd <input_file.bag> <topic> <output_directory>
```
### Getting trajectory
Per frame trjaectory is available in `/aft_mapped_to_init` topic.

