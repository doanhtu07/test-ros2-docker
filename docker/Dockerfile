# Use the official ROS image as the base image
FROM ros:humble-ros-core

# Set shell for running commands
SHELL ["/bin/bash", "-c"]

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
  build-essential \
  git \
  vim \
  net-tools \
  wget \
  python3-colcon-common-extensions \
  python3-colcon-mixin \
  python3-rosdep \
  python3-vcstool \
  && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# setup colcon mixin and metadata
RUN colcon mixin add default \
  https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
  colcon mixin update && \
  colcon metadata add default \
  https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
  colcon metadata update

# install desktop variants, which extends on ros-base, which extends on our base image ros-core
# https://snapcraft.io/ros-humble-desktop
RUN apt-get update && apt-get install --no-install-recommends -y \
  ros-humble-desktop \
  && rm -rf /var/lib/apt/lists/*

COPY ./build-scripts/*.sh /build-scripts/

# prepare the environment
RUN chmod +x /build-scripts/prepare.sh
RUN /build-scripts/prepare.sh 

# Set the entrypoint to source ROS setup.bash and run a bash shell
CMD ["/bin/bash"]
