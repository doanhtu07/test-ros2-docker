# Nodes

https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html

## Background

### ROS 2 graph

- A network of ROS 2 elements processing data together at the same time
- Includes all executables and connections between them if we map them all out and visualize them

### ROS 2 nodes

<img src="./media/1-1-Nodes-TopicandService.gif" alt="drawing" width="500px" style="display: block; margin-left: auto; margin-right: auto; margin-top: 20px; margin-bottom: 20px;" />

- Each node is responsible for a single, modular purpose

  - E.g: controlling the wheel motors
  - Or: publishing the sensor data from a laser range-finder

- Each node can send and receive data from other nodes via:

  - Topics
  - Services
  - Actions
  - Or: parameters

## Run example

### Launch an executable

```
ros2 run <package_name> <executable_name>
```

--> `ros2 run turtlesim turtlesim_node`

### List nodes

```
ros2 node list
```

### Remapping

**Link**: https://design.ros2.org/articles/ros_command_line_arguments.html#name-remapping-rules

Remapping allows you to reassign default node properties to custom values:

- Node name
- Topic names
- Service names
- Etc.

### Node info

```
ros2 node info <node_name>
```

--> `ros2 node info /my_turtle`
