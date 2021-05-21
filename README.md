# CAN Inverter Communication

Package provides ROS <- CAN -> Inverter communication (both ways)
And can be used to mesure "speed set time"


## Requirements
- Ubuntu 20.04
- ROS Noetic
- DV Workspace (from DV recruitment task)


## Nodes description
- APPS_publisher - publishes mes to CAN_sender and calculates speed set time
- CAN_sender - node responsible for communication ROS -> Inverter
- INVERTER - simulates Inverter
- TIMER - receives mes from INVERTER and sends timestamps to APPS_publisher

![](CanApps.png)

## Installation



```bash
cd ~/dv_ws/src
git clone https://github.com/GrzegorzCzput/CAN_Inverter_Communication.git CAN_apps/
```

## Usage
### Import can library and create a virtual CAN

```bash
sudo modprobe can_dev
sudo modprobe can_raw
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

### Give permissions to execute
for example
``` bash 
chmod +x CAN_reciver.py
```

### Build workspace
```bash
cd ~/dv_ws
catkin_make
source devel/setup.bash
roscore
```

### Run node (another terminal)
```bash
cd ~/dv_ws
source devel/setup.bash
rosrun CAN_apps APPS_publisher.py 
```
