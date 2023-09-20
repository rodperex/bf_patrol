# bf_patrol

[Link to the original source](https://github.com/fmrico/book_ros2/tree/main/br2_bt_patrolling)

This package exemplifies the capabilitites of the [behaviorfleets](https://github.com/rodperex/behaviorfleets), a package to execute behavior trees in a distributed manner. It is called *bf_patrol* since all examples are dummy and robots patrol instead of doing actual work.

Everything here is based on a example of the book **A Concise Introduction to Robot Programming with ROS2**. Do not miss the opportunity to learn from this super book.

# How to run
To make it work it is **essential** that following commands are executed **in order**. If the main program is not run before the robots are launched, the shared blackboard will not be configured thus group patrolling will fail.

To run the basic patrolling example:

```
ros2 launch multi_robot multi_tb3_simulation.launch.py autostart:=True use_composition:=False
ros2 run bf_patrol bf_patrol
ros2 launch bf_patrol bf_3_patrol.launch.py
```

To run the factory:

```
ros2 launch multi_robot multi_tb3_simulation.launch.py autostart:=True use_composition:=False
ros2 run bf_patrol factory
ros2 launch bf_patrol factory.launch.py
```
