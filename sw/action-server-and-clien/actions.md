
# Prerequisites

## Install ROS 2 Foxy on both PCs.
    - Follow the official ROS 2 Foxy installation guide.

## File Structure

Ensure the following directory structure for the workspaces:

## Drone PC

 ~/uav_interface_ws: Contains the action definitions.
 ~/uav_ws: Contains the drone server and MAVSDK integration.

##Husky PC
~/uav_interface_ws: Contains the shared action definitions.
 ~/ugv_ws: Contains the ground vehicle client for sending commands.

## Cloning and Building Workspaces

On Drone PC (UAV):

-git clone <https://github.com/hamodiss/actrion-server-and-clien.git>

## Step 1: Clone and Build UAV

-cd ~

-cd uav_interface_ws

-colcon build --cmake-clean-cache

-source install/setup.bash

**Step 2: Run the UAV Action Server**

-ros2 run uav_control flight_action_server

On Husky PC (UGV):

-git clone <https://github.com/hamodiss/actrion-server-and-clien.git>

**Step 1: Clone and Build UGV**

-cd ~

-cd ugv_ws

-colcon build --cmake-clean-cache

-source install/setup.bash

**Step 2: Run the UGV Action Client**

-ros2 run ugv_control flight_action_client

## Running the System

Steps to Test the System:

Start the UAV Server:

On the Drone PC, run:

-ros2 run uav_control flight_action_server

Start the UGV Client:

 On the Husky PC, run:

-ros2 run ugv_control flight_action_client

Send Commands from UGV to UAV:
Use the terminal on the Husky PC to send commands:
  - 1: Start flying to the target altitude.
  - 0: Land the UAV.
