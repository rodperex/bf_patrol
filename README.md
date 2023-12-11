# bf_patrol

[Link to the original source](https://github.com/fmrico/book_ros2/tree/main/br2_bt_patrolling)

This package exemplifies the capabilitites of the [behaviorfleets](https://github.com/rodperex/behaviorfleets), a package to execute behavior trees in a distributed manner. It is called *bf_patrol* since all examples are dummy and robots patrol instead of doing actual work.

Everything here is based on a example of the book **A Concise Introduction to Robot Programming with ROS2**. Do not miss the opportunity to learn from this super book.

# Examples

* Visiting a set of waypoints
* Manufacturing


# How to run the simulations
To make it work it is **essential** that following commands are executed **in order**. If the main program is not run before the robots are launched, the shared blackboard will not be configured thus collaboration will fail.

To run the basic patrolling example:

```
ros2 launch multi_robot multi_tb3_simulation.launch.py autostart:=True use_composition:=False
ros2 launch bf_patrol orch_patrol.launch.py
ros2 launch bf_patrol work_patrol.launch.py
```

To run the factory:

```
ros2 launch multi_robot multi_tb3_simulation.launch.py autostart:=True use_composition:=False
ros2 run bf_patrol factory
ros2 launch bf_patrol worker_1.launch.py
ros2 launch bf_patrol worker_2.launch.py
ros2 launch bf_patrol assembler_3.launch.py
```

Please make sure parameter `use_sim_time` is set to `True` for the simulations.

# Clarifications
All examples here have a non-robot orchestrator that publishes the tasks to be done, which does not imply a robot can delegate a task that it cannot perform, such as opening a door, or doing some highly specialized operation.

