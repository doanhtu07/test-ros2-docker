# Topics

https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html

## Background

Topics are a vital element of the ROS graph that act as a bus for nodes to exchange messages

<img src="./media/2-1-Topic-MultiplePublisherandMultipleSubscriber.gif" alt="drawing" width="500px" style="display: block; margin-left: auto; margin-right: auto; margin-top: 20px; margin-bottom: 20px;" />

### rqt_graph

- A tool for visualizing the ROS graph
- A plugin of the `rqt` package

### List topics

- List out all topics and their types

```

ros2 topic list

ros2 topic list -t

```

### Topic echo

- Listen to data published to a topic

```
ros2 topic echo <topic_name>
```

--> `ros2 topic echo <topic_name>`

### Topic info

- Get information about a topic

```
ros2 topic info <topic_name>
```

### Show interface

- Show the interface of the type of a topic

```
ros2 interface show <interface_name>
```

--> `ros2 interface show geometry_msgs/msg/Twist`

### Publish to a topic directly

- Publish to a topic directly through the command line

```
ros2 topic pub <topic_name> <message_type> '<args>'
```

- `args` is the actual data to be published in the structure of the interface
- `args` needs to be in YAML structure

Once:

--> `ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"`

Rate 1 Hz steady stream:

--> `ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"`

#### Publish messages with timestamps

When publishing messages with timestamps, pub has two methods to automatically fill them out with the current time.

---

For messages with a `std_msgs/msg/Header`, the header field can be set to auto to fill out the `stamp` field.

```
ros2 topic pub /pose geometry_msgs/msg/PoseStamped '{header: "auto", pose: {position: {x: 1.0, y: 2.0, z: 3.0}}}'
```

---

If the message does not use a full header, but just has a field with type `builtin_interfaces/msg/Time`, that can be set to the value now.

```
ros2 topic pub /reference sensor_msgs/msg/TimeReference '{header: "auto", time_ref: "now", source: "dumy"}'
```

### ROS 2 topic hz

- View the rate at which data is published

```
ros2 topic hz /turtle1/pose
```
