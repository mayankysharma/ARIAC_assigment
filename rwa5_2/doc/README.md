## ENPM663 RWA5 Group2
* ARIAC task flow for Kitting task, using ROS2 (Galactic) and Python.

## Team Members:
1. Yashveer Jain
2. Mayank Sharma
3. Abraruddin Syed
4. Dhruv Sharma

## Instructions to run 
Make sure that the trial file `rwa5_spring2024.yaml` is put in the ariac_gazebo package in the config folder. Then do the following steps to run:
* Unzip the package in the src of the ARIAC workspace
* cd in to the workspace ariac_ws/
```bash
cd ariac_ws/
``` 
* Build the package using
  ```bash
  colcon build
  source install/setup.bash

  ```
* Run in the chronology
  - First terminal
  ```bash
  source install/setup.bash
  ros2 launch ariac_gazebo ariac.launch.py trial_name:=rwa5_spring2024
  ```

  - Second terminal
  ```bash
  source install/setup.bash
  ros2 launch rwa5_2 mov_launch.py
  ```
  
  - Third terminal
  ```bash
  source install/setup.bash
  ros2 launch rwa5_2 ariac_interface.launch.py
  ```


