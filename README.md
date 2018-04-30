# Bebop Vicon Control
Package combining vicon_bridge, bebop_autonomy, and pid to control the Parrot Bebop quadcopter using Vicon motion capture system.
The quadcopter is controlled through keyboard node or by calling service functions through API server.
# Setup
Need to download and install vicon_bridge and bebop_autonomy from source:
https://github.com/AutonomyLab/bebop_autonomy
https://github.com/ethz-asl/vicon_bridge
And install pid package:
apt-get install ros-\<distributionName\>-pid

#Launch
1. Connect to Bebop Wifi AP
2. `roslaunch bebop_vicon_control bebop_control.py`
3. `roslaunch bebop_vicon_control bebop_pid_keyboard.py`

#Usage
More notes...
