#!/bin/bash

source ./prepare.sh

# Discovery server manual: https://turtlebot.github.io/turtlebot4-user-manual/setup/discovery_server.html

# Base IP (first three octets)
base_ip="192.168.0."

# Starting and ending values for the last octet
base=220
start=1
end=20
finalInput="\n" # ROS_DOMAIN_ID set to 0 by default

# Loop to increment the last octet
for ((i=start; i<=end; i++)); do
    # Construct the full IP address
    sum=$((i+base))
    full_ip="${base_ip}${sum}"

    finalInput="${finalInput}${i}\n" # Server ID
    finalInput="${finalInput}${full_ip}\n" # Server IP
    finalInput="${finalInput}\n" # Server port set to 11311 by default

    if [ $i -eq $end ]; then
        finalInput="${finalInput}d\n" # done
    else
        finalInput="${finalInput}a\n" # add more
    fi
done

# Clean up previous configurations
sudo rm -rf /etc/turtlebot4_discovery/

printf "$finalInput" | bash <(wget -qO - https://raw.githubusercontent.com/turtlebot/turtlebot4_setup/humble/turtlebot4_discovery/configure_discovery.sh)

source /home/vagrant/.bashrc

ros2 daemon stop; ros2 daemon start
