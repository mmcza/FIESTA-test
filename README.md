# FIESTA-test

# 1. Creating and running the docker container

Clone the repository inside `fiesta_shared` directory:
```Shell
git clone https://github.com/mmcza/FIESTA-test.git /home/$user/fiesta_shared
```

To build the image, go inside the `fiesta_shared` directory and run:
```Shell
docker build -t fiesta_test .
```

To start the container simply use:
```Shell
bash start_container.sh
```

To test the proposed dataset use the following command inside `fiesta_shared` (size of the file is 4.5GB):
```Shell
wget http://robotics.ethz.ch/~asl-datasets/iros_2017_voxblox/data.bag
```

# 2. Running the algorythm

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