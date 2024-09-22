# Actions

## Background

- Actions are one of the communication types in ROS 2 and are intended for long running tasks

- Consist of 3 parts:

  - goal,
  - feedback
  - result

- Actions are built on topics and services

- Their functionality is similar to services, except actions can be **canceled**

  - They also provide steady feedback, as opposed to services which return a single response

- Actions use a client-server model, similar to the publisher-subscriber model

  - An “action client” node sends a goal to an “action server” node that acknowledges the goal and returns a stream of feedback and a result

<img src="./media/5-1-Action-SingleActionClient.gif" alt="drawing" width="500px" style="display: block; margin-left: auto; margin-right: auto; margin-top: 20px; margin-bottom: 20px;" />

### Use actions

- When you launch the /teleop_turtle node, you will see the following message in your terminal

```
Use arrow keys to move the turtle.
Use G|B|V|C|D|E|R|T keys to rotate to absolute orientations. 'F' to cancel a rotation.
```

Let’s focus on the second line, which corresponds to an action. (The first instruction corresponds to the “cmd_vel” topic, discussed previously in `topics tutorial`)

- Each time you press one of these keys, you are sending a goal to an action server that is part of the `/turtlesim` node

  - The goal is to rotate the turtle to face a particular direction
  - A message relaying the result of the goal should display once the turtle completes its rotation
  - The `F` key will cancel a goal mid-execution

- Not only can the client-side (your input in the teleop) stop a goal, but the server-side (the /turtlesim node) can as well

  - When the server-side chooses to stop processing a goal, it is said to “abort” the goal
  - Try hitting the `D` key, then the `G` key before the first rotation can complete
    - In the terminal where the /turtlesim node is running, you will see the message
    ```
    [WARN] [turtlesim]: Rotation goal received before a previous goal finished. Aborting previous goal
    ```

---

**NOTE**

This action server chose to abort the first goal because it got a new one. It could have chosen something else, like reject the new goal or execute the second goal after the first one finished. Don’t assume every action server will choose to abort the current goal when it gets a new one.

---

### Node info

To see the list of actions a node provides

```
ros2 node info <node_name>
```

- `ros2 node info /turtlesim`

### Actions list

To identify all the actions in the ROS graph

```
ros2 action list
```

or do it with types

```
ros2 action list -t
```

### Action type

If you want to check the action type for the action

```
ros2 action type <action_name>
```

- `ros2 action type /turtle1/rotate_absolute`

### Action info

You can further introspect an action

```
ros2 action info <action_name>
```

- `ros2 action info /turtle1/rotate_absolute`

### Action interface show

```
ros2 interface show <type_name>
```

- `ros2 interface show turtlesim/action/RotateAbsolute`

  ```
  # The desired heading in radians
  float32 theta
  ---
  # The angular displacement in radians to the starting position
  float32 delta
  ---
  # The remaining rotation in radians
  float32 remaining
  ```

- The section of this message above the first `---` is the structure (data type and name) of the `goal request`

- The next section is the structure of the `result`

  - In this case, it's the total angular distance from starting position to the goal

- The last section is the structure of the `feedback`

### Send goal

```
ros2 action send_goal <action_name> <action_type> <values>
```

- `<values>` needs to be in YAML format

- `ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.57}"`

To see feedback of the action

```
ros2 action send_goal <action_name> <action_type> <values> --feedback
```

- `ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: -1.57}" --feedback`
