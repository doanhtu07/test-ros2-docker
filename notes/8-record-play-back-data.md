# Record and play back data

## Background

`ros2 bag` is a command line tool for recording data published on topics and services in your ROS 2 system

It accumulates the data passed on any number of topics and services, then saves it in a database

You can then replay the data to reproduce the results of your tests and experiments

Recording topics and services is also a great way to share your work and allow others to recreate it

## Manage topic data

### Setup

```
mkdir bag_files
cd bag_files
```

### Choose a topic

`ros2 bag` can record data from messages published to topics

```
ros2 topic list
ros2 topic echo /turtle1/cmd_vel
```

### Record topics

Before running this command on your chosen topic, open a new terminal and move into the bag_files directory you created earlier, because the rosbag file will save in the directory where you run it

```
ros2 bag record <topic_name>
```

- `ros2 bag record /turtle1/cmd_vel`

The data will be accumulated in a new bag directory with a name in the pattern of `rosbag2_year_month_day-hour_minute_second`

This directory will contain a `metadata.yaml` along with the bag file in the recorded format

### Record multiple topics

You can also record multiple topics, as well as change the name of the file `ros2 bag` saves to

```
ros2 bag record -o subset /turtle1/cmd_vel /turtle1/pose
```

- The `-o` option allows you to choose a unique name for your bag file
- The following string, in this case subset, is the file name

### Inspect recorded data

You can see details about your recording

```
ros2 bag info <bag_file_name>
```

### Play topic data

Before replaying the bag file, enter Ctrl+C in the terminal where the teleop is running

Then make sure your turtlesim window is visible so you can see the bag file in action

```
ros2 bag play <bag_file_name>
```

Your turtle will follow the same path you entered while recording (though not 100% exactly; turtlesim is sensitive to small changes in the system’s timing).

Because the subset file recorded the /turtle1/pose topic, the ros2 bag play command won’t quit for as long as you had turtlesim running, even if you weren’t moving

- This is because as long as the /turtlesim node is active, it publishes data on the /turtle1/pose topic at regular intervals

## Manage service data

You’ll be recording service data between `introspection_client` and `introspection_service`, then display and replay that same data later on

To record service data between service client and server, `Service Introspection` must be enabled on the node

```
ros2 run demo_nodes_cpp introspection_service --ros-args -p service_configure_introspection:=contents
```

```
ros2 run demo_nodes_cpp introspection_client --ros-args -p client_configure_introspection:=contents
```

**See more**: https://docs.ros.org/en/jazzy/Tutorials/Demos/Service-Introspection.html

### Check service list

```
ros2 service list
```

### Check Service Introspection mode

To check if `Service Introspection` is enabled on the client and service

```
ros2 service echo --flow-style /add_two_ints
```

### Record services

Service data can be recorded with topics at the same time

```
ros2 bag record --service <service_names>
```

Record all services

```
ros2 bag record --all-services
```

### Inspect service data

```
ros2 bag info <bag_file_name>
```

### Play service data

```
ros2 bag play --publish-service-requests <bag_file_name>
```

Before replaying the bag file, enter `Ctrl+C` in the terminal where `introspection_client` is running

When `introspection_client` stops running, `introspection_service` also stops printing the result because there are no incoming requests

Replaying the service data from the bag file will start sending the requests to `introspection_service`

This is because `ros2 bag play` sends the service request data from the bag file to the `/add_two_ints` service
