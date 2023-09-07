# bf_patrol

Based on a example of the book **A Concise Introduction to Robot Programming with ROS2**.

[Link to the original source](https://github.com/fmrico/book_ros2/tree/main/br2_bt_patrolling)


# How to run
```
ros2 launch multi_robot multi_tb3_simulation.launch.py autostart:=True use_composition:=False
ros2 run bf_patrol bf_patrol
ros2 launch bf_patrol bf_3_patrol.launch.py
```

# Problems 
* Goal, as soon as it is extracted from the bb, should be marked as IN PROCESS so others do not take it.
* This implies there should be a process checking if a task has been in process longer than a certain threshold, so it is set back to PENDING. 

