# rqt_console

## Background

`rqt_console` is a GUI tool used to introspect log messages in ROS 2

With `rqt_console`, you can:

- collect those messages over time
- view them closely and in a more organized manner
- filter them
- save them
- even reload the saved files to introspect at a different time

### Setup

```
ros2 run rqt_console rqt_console
```

- The first section of the console is where log messages from your system will display
- In the middle you have the option to filter messages by excluding severity levels. You can also add more exclusion filters using the plus-sign button to the right
- The bottom section is for highlighting messages that include a string you input. You can add more filters to this section as well

### Logger levels

ROS 2’s logger levels are ordered by severity

- Fatal: system is going to terminate to try to protect itself from detriment
- Error: significant issues that won’t necessarily damage the system, but are preventing it from functioning properly
- Warn: unexpected activity or non-ideal results that might represent a deeper issue, but don’t harm functionality outright
- Info (**Default**): event and status updates that serve as a visual verification that the system is running as expected
- Debug: messages detail the entire step-by-step process of the system execution

**NOTE**: You will only see messages of the default severity level and more-severe levels

### Set default logger level

You can set the default logger level when you first run the /turtlesim node using remapping

```
ros2 run turtlesim turtlesim_node --ros-args --log-level WARN
```
