## ENPM663 RWA3
* ARIAC task flow for Kitting task, using ROS2 (Galactic) and Python.

## This branch focused on assignment 3 of the course.

## Instructions to run task4
* Clone the package
* Build the package using
  ```bash
  colcon build
  ``
* Run in the chronology
- First terminal
```bash
ros2 run rwa3_2 ariac_interface.py
```
- Second terminal
```bash
ros2 launch ariac_gazebo ariac.launch.py trial_name:=rwa3_spring2024
```

