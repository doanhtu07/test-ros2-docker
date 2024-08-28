# Test ROS 2 with Docker

Learn from:

- https://github.com/adeeb10abbas/ros2-docker-dev
- https://github.com/2b-t/docker-for-robotics

Test:

- https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html

# Run

- Create + run docker containers in the background

```
docker compose -f docker/docker-compose.yml up -d
```

- Run docker service

```
docker exec -it test-ros2-docker /bin/bash

```

- Inside the container

```
source ros_entrypoint.sh
```

- Test ROS 2

```
ros2
```

- Test Rviz 2. Use localhost:8080 on host to view through noVNC

```
rviz2
```