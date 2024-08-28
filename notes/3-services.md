# Services

https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html

## Background

Services are based on a call-and-response model versus the publisher-subscriber model of topics

While topics allow nodes to subscribe to data streams and get continual updates, services only provide data when they are specifically called by a client

<img src="./media/3-1-Service-SingleServiceClient.gif" alt="drawing" width="500px" style="display: block; margin-left: auto; margin-right: auto; margin-top: 20px; margin-bottom: 20px;" />

<img src="./media/3-2-Service-MultipleServiceClient.gif" alt="drawing" width="500px" style="display: block; margin-left: auto; margin-right: auto; margin-top: 20px; margin-bottom: 20px;" />

### Service list

- List all the services currently active in the system

```
ros2 service list
```

- You will see that both nodes have the same six services with parameters in their names

- Nearly every node in ROS 2 has these infrastructure services

### Service type

```
ros2 service type <service_name>
```

- `ros2 service type /clear`

  - Output: `std_srvs/srv/Empty`

- Empty means the service call sends no data when making a request and receives no data when receiving a response

### Service list with type

```
ros2 service list -t
```

### Service info

```
ros2 service info <service_name>
```

### Service find using type

```
ros2 service find <type_name>
```

- `ros2 service find std_srvs/srv/Empty`

### Service interface show

```
ros2 interface show <type_name>
```

- `ros2 interface show std_srvs/srv/Empty`

  - Output: `---`

- The `---` separates the `request structure (above)` from the `response structure (below)`

- But `Empty` don't have this so it will output `---` only

- `ros2 interface show turtlesim/srv/Spawn`

  Output:

  ```
  float32 x
  float32 y
  float32 theta
  string name # Optional.  A unique name will be created and returned if this is empty
  ---
  string name
  ```

### Service call

```
ros2 service call <service_name> <service_type> <arguments>
```

- The `<arguments>` part is optional for services with optional/zero arguments

- `<arguments>` needs to be in YAML format

- `ros2 service call /clear std_srvs/srv/Empty`

- `ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: ''}"`

### Service echo

- To see the data communication between a service client and a service server you can `echo` the service

- `ros2 service echo <service_name | service_type> <arguments>`

- `ros2 service echo` depends on service introspection of a service client and server, that is disabled by default

  - To enable it, users must call `configure_introspection` after creating a server client or server

  ```
  ros2 launch demo_nodes_cpp introspect_services_launch.py

  ros2 param set /introspection_service service_configure_introspection contents

  ros2 param set /introspection_client client_configure_introspection contents

  ros2 service echo --flow-style /add_two_ints
  ```
