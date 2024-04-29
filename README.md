## ENPM663 ARIAC Group2
* ARIAC task flow for Kitting task, using ROS2 (Galactic) and Python.

## Team Members:
1. Yashveer Jain
2. Mayank Sharma
3. Abraruddin Syed
4. Dhruv Sharma

## Instructions to run rwa5 
Make sure that the trial file `rwa5_spring2024.yaml` is put in the ariac_gazebo package in the config folder. Then do the following steps to run:
* Clone the package in the src of the ARIAC workspace
* cd in to the workspace ariac_ws/
```bash
cd ariac_ws/
``` 
* Build the package using
  ```bash
  colcon build --packages-select rwa5_2

  ```
* Run in the chronology
  - First terminal
  ```
  source install/setup.bash
  ```
  ```bash
  ros2 launch ariac_gazebo ariac.launch.py trial_name:=rwa5_spring2024
  ```
  

  - Second terminal
  Start the competition
  ```
  source install/setup.bash
  ```
```
ros2 service call /ariac/start_competition std_srvs/srv/Trigger
```
- Launch Moveit

  ```
   ros2 launch rwa5_2 mov_launch.py 

  ```
- Trigger the move service
```
ros2 service call /move_floor_robot std_srvs/srv/Trigger
```


