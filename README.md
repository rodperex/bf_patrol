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
* Only one robot participates in the poll
