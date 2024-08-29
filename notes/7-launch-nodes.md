# Launch nodes

## Background

In most of the introductory tutorials, you have been opening new terminals for every new node you run

But it becomes tedious

Launch files allow you to start up and configure a number of executables containing ROS 2 nodes simultaneously

Running a single launch file with the `ros2 launch` command will start up your entire system - all nodes and their configurations - at once

### Running a launch file

To know where file multisim is in Linux

```
find . -name multisim.launch.py
```

Launch the file

```
ros2 launch turtlesim multisim.launch.py
```
