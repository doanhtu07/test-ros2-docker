# Parameters

## Background

- A parameter is a configuration value of a node

- You can think of parameters as node settings

- A node can store parameters as integers, floats, booleans, strings, and lists

- In ROS 2, each node maintains its own parameters

### Param list

```
ros2 param list
```

### Param get

- Display type and value of a parameter

```
ros2 param get <node_name> <parameter_name>
```

- `ros2 param get /turtlesim background_g`

### Param set

- To change a parameter’s value at runtime

```
ros2 param set <node_name> <parameter_name> <value>
```

- `ros2 param set /turtlesim background_r 150`

### Param dump

- Display all the parameters' values and types of a node

```
ros2 param dump <node_name>
```

- The command prints to the standard output (stdout) by default but you can also redirect the parameter values into a file to save them for later

```
ros2 param dump <node_name> > <filename>.yaml
```

- `ros2 param dump /turtlesim > turtlesim.yaml`

### Param load

- Load parameters from a file to a currently running node

```
ros2 param load <node_name> <parameter_file>
```

- `ros2 param load /turtlesim turtlesim.yaml`

**NOTE**:

- Read-only parameters can only be modified at startup and not afterwards, that is why there are some warnings for the “qos_overrides” parameters

### Param load on node startup

```
ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>
```

- `ros2 run turtlesim turtlesim_node --ros-args --params-file turtlesim.yaml`
