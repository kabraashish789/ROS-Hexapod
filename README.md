# ROS Hexapod
 Simulating and controlling hexapod in gazebo using ROS

Commands to run for testing the simulation in gazebo
1) Open a terminal
2) enter: cd hexapod_ws   #### or simply ensure that you are currently in "hexapod_ws", with respect to where you have downloaded the zip file
3)Run : colcon build
4) Run : source install/setup.bash
5)Run :ros2 launch hexapod_description position1.launch.py
#### You have launched the gazebo simulation and spawned the robot in it.

Now you need to open a second terminal to run the action server
repeat the previous steps 1 to 4 ## actually it you just add a new terminal by clicking on the "+" at top left corner you won't need to run 2) again
6)Run : ros2 run patrol_action_server patrol_action_server_exe

Now you need to open a third terminal to run the action client
repeat the previous steps 1 to 4
7) Run : ros2 run patrol_action_client patrol_action_client_exe

You are ready to start the simulation in gazebo.
