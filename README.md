-- How to run the docker:  
docker run -it --rm --privileged --net=host  --env=DISPLAY  --env=QT_X11_NO_MITSHM=1  -v /tmp/.X11-unix:/tmp/.X11-unix  -v /home/autolab/ros2_ws:/home/student/ros2_ws ros2-turtlebot3:v6.0
============================================  
--building and sourcing the docker  
$ cd ~/ros2_ws  
$ colcon build --symlink-install  
$ source ~/ros2_ws/install/setup.bash 
============================================  
--run the navigation launch file
$ ros2 launch my_robot_controller run_navigation.launch.py
