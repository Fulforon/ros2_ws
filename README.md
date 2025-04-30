# How to run the docker:

docker run -it --rm --privileged  --env=DISPLAY  --env=QT_X11_NO_MITSHM=1  -v /tmp/.X11-unix:/tmp/.X11-unix -v /home/autolab/ros2_ws:/ros2_ws -v /home/autolab/autoware_map:/autoware_map --workdir /ros2_ws mohsen_aw:full bash

# Build the docker

colcon build

# Run outside of the docker inside the ros2 workerspace to unlock the files incase they get locked during colcon build

sudo chown -R $(id -u):$(id -g) .

# Run the setup file

. setup.sh

# Run the launch file

ros2 launch my_robot_controller car_nav.launch.py 

