# The Scenario Editor is an online tool to create and modify OpenScenario files for Autoware simulation.
https://scenario.ci.tier4.jp/

# Run the following command before opening docker - This is so that docker can open external windows

xhost +local:docker

# Run the docker with the command below

docker run -it --rm --privileged --env=DISPLAY  --env=QT_X11_NO_MITSHM=1  -v /tmp/.X11-unix:/tmp/.X11-unix -v /home/autolab/ros2_ws:/ros2_ws -v /home/autolab/autoware_map:/autoware_map --workdir /ros2_ws mohsen_aw:full bash

# Before running any code you have to source the setup.sh in the ros2ws

. setup.sh

# This happend and I have no idea if it causes issues but the autoware map folder inside of the row2_ws got locked, to unlock it run the following command outside the docker in the same folder

sudo chown -R $(id -u):$(id -g) .

# Running the custom the scenario, the t4v2 is the name of the custom scenario gotten from the webpage scenario creater

ros2 launch scenario_test_runner scenario_test_runner.launch.py record:=false scenario:='/autoware_map/t4v2.yaml' sensor_model:=sample_sensor_kit vehicle_model:=sample_vehicle output_directory:='/autoware_map' launch_rviz:=true

# Every time you restart the docker and run the scenario you have to go to the display section and enable some stuff so you can actually see the cars - then go file -> save config, to make it remember the display settings it opens next time. Bear in mind that it forgets every time the docker is reset.
