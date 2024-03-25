## ENPM663 RWA3 for ARIAC
* Using ROS2 complete the assignments based on the competition called ARIAC.

## This branch focused on assignment 3 of the course.
## Instructions to run task4
* Run in the chronology
- First terminal
* If want to run direct node: 
```bash
ros2 run rwa3_2 ariac_interface.py
```
* Other option to launch using launch file:
```bash
ros2 launch rwa3_2 ariac_interface.launch.py
```
- Second terminal
```bash
ros2 launch ariac_gazebo ariac.launch.py trial_name:=rwa3_spring2024
```

