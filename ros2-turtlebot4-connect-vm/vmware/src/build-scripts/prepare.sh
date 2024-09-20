#!/bin/bash

search_strings=(
  "source /opt/ros/humble/setup.bash"
  "export DEBIAN_FRONTEND=noninteractive"
)

file_paths=(
  "/home/vagrant/.bashrc"
  "/home/vagrant/.bashrc"
)

# Loop through both arrays
for ((i=0; i<${#file_paths[@]}; i++)); do
    file="${file_paths[$i]}"
    search_string="${search_strings[$i]}"
    
    # Check if the string exists in the file
    if grep -Fxq "$search_string" "$file"; then
        echo "The line '$search_string' exists in '$file'."
    else
        # Append the string to the file
        echo "$search_string" >> "$file"
    fi
done
