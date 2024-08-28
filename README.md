# Test ROS 2 with Docker

Learn from:

- Main docker setup: https://github.com/adeeb10abbas/ros2-docker-dev
- Extra learning: https://github.com/2b-t/docker-for-robotics

Knowledge:

- Basic concepts: https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html
- Robot simulation: https://docs.ros.org/en/jazzy/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html#

Extra:

- ROS 2 workshop: https://ros2-industrial-workshop.readthedocs.io/en/latest/index.html

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

