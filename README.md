# FIESTA-test

# 1. Creating and running the docker container

Clone the repository inside `fiesta_shared` directory:
```Shell
git clone https://github.com/mmcza/FIESTA-test.git ~/fiesta_shared
```

> [!NOTE]  
> If you clone the repository/download the rosbag to another directory, make sure to change the line 29 inside `start_container.sh`

To build the image, go inside the `fiesta_shared` directory and run:
```Shell
docker build -t fiesta_test .
```

To start the container simply use:
```Shell
bash start_container.sh
```

To download the proposed dataset use the following command inside `fiesta_shared` (size of the file is 4.5GB):
```Shell
wget http://robotics.ethz.ch/~asl-datasets/iros_2017_voxblox/data.bag
```

# 2. Running the algorythm

# 2.1. Cow and lady dataset (from the original FIESTA repository)

In first console run the demo:
```Shell
roslaunch fiesta cow_and_lady.launch
```
RVIZ should launch. In second terminal run connect to the container:
```Shell
docker exec -ti fiesta bash
```
Next, play the data from the rosbag:
```Shell
rosbag play data.bag
```

# 2.2. TUM Datasets

Download one of the [TUM datasets](https://cvg.cit.tum.de/data/datasets/rgbd-dataset/download) (**it must have a pointcloud**) inside the `fiesta_shared` directory. For example:
```Shell
wget https://cvg.cit.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_teddy-2hz-with-pointclouds.bag
```

With docker running simply run:
```Shell
roslaunch transform_publisher tum_datasets.launch
```
```Shell
rosbag play <name_of_the_bag>
```
> [!NOTE]  
> Check if the pointcloud is published on the `/camera/rgb/points` topic, if not, just change the line 76 in the [launch file](/transform_publisher/launch/tum_datasets.launch) and also change the `frame_id` of the pointcloud and if it's not `openni_rgb_optical_frame` than you can use the argument `child_frame_id` of the launch file. Additionally you can change the rate of checking for the transformation on the `/tf` topic.
 
> [!NOTE]  
> If you need the tool to select one transformation sent over the `/tf` topic and publish it into another topic as TransformStamped, you can do it like this: `rosrun transform_publisher transform_publisher_node <frame_id> <child_frame_id> <topic_to_publish> <time_between_checks>`, e.g.: `rosrun transform_publisher transform_publisher_node world openni_rgb_optical_frame testowy 0.1`.