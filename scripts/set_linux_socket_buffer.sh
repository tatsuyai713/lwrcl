#!/bin/bash

# Set the target variable and file
target_var="export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA"
bashrc_file="$HOME/.bashrc"

# Check if the variable exists in the .bashrc file
if ! grep -q "^${target_var}$" "$bashrc_file"; then
  # If the variable does not exist, append it to the file
  echo "$target_var" >> "$bashrc_file"
  echo "The variable was added to your .bashrc"
else
  echo "The variable already exists in your .bashrc"
fi

# Increase the maximum socket send buffer size
sudo sysctl -w net.core.wmem_max=12582912
# Increase the maximum socket receive buffer size
sudo sysctl -w net.core.rmem_max=12582912

# Retrieve a list of NICs and store it in an array
mapfile -t nic_list < <(ip link show | grep -oP '(?<=: ).*?(?=:)')

# Display the list of available NICs
echo "List of Network Interfaces:"
for i in "${!nic_list[@]}"; do
    echo "  [$((i+1))] ${nic_list[i]}"
done

# Prompt the user to select a NIC
read -p "Enter the number corresponding to the NIC you wish to select: " selection

# Validate the input is a number and within the range of available NICs
if [[ ! $selection =~ ^[0-9]+$ ]] || (( selection < 1 || selection > ${#nic_list[@]} )); then
    echo "Error: Please enter a valid number between 1 and ${#nic_list[@]}."
    exit 1
fi

# Display the selected NIC
selected_nic=${nic_list[$((selection-1))]}
echo "You have selected: $selected_nic"

# Change the transmission queue length for the selected NIC
sudo ip link set txqueuelen 20000 dev ${selected_nic}
echo "Successfully updated the queue length for $selected_nic to 20000."
