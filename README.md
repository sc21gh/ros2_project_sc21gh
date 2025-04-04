# ros2-project
A lab project combines what has been taught in this module, to complete a task 

To run the project:
- Open 4 terminals and enter singularity prompt for all of them
- cd to "~/ros2_ws" and "colcon build" in one terminal
- Enter "source ~/.bashrc" in all terminals
- Terminal 1 enter "ros2 launch turtlebot3_gazebo turtlebot3_task_world_2025.launch.py"
- Terminal 2 enter "ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/ros2_ws/src/ros2_project_sc21gh/map/map.yaml"
- Set the 2D Post estimate in RViz for the map.
- Terminal 3 enter "ros2 run ros2_project_sc21gh vision"
- Terminal 4 enter "ros2 run ros2_project_sc21gh motion_planning"

What the robot does on loop:
- Generate random x and y coordinates in the map and travel to them
- Spin 360 degrees and look for any blue objects
- If a blue object is found, vision will broadcast a message to tell motion planning to stop. Then vision will orientate and move the robot to the blue object. Then the program ends when the contour Area is big enoguh (very close to the object.)
