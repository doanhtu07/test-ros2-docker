if [ ! -e "~/.bashrc" ]; then
  touch ~/.bashrc
fi

cat > ~/.bashrc <<EOF
source /ros_entrypoint.sh
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
EOF

if [ ! -e "~/.bash_profile" ]; then
  touch ~/.bash_profile
fi

cat ~/.bash_profile <<EOF
if [ -f ~/.bashrc ]; then
  . ~/.bashrc
fi
EOF

