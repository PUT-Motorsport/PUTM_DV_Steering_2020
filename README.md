# CAN Inverter Communication


Package provides ROS <- CAN -> Inverter communication (both ways)
And can be used to measure "speed set time"



## Requirements

- Ubuntu 20.04 / 18.04
- ROS Noetic / ROS Melodic
- DV Workspace (from DV recruitment task)


## Nodes description

- APPS_publisher - publishes mes to CAN_sender and calculates speed set time
- CAN_sender - node responsible for communication ROS -> Inverter
- INVERTER - simulates Inverter
- TIMER - receives mes from INVERTER and sends timestamps to APPS_publisher


- apps_publisher - node responsible for simulating APPS data
- can_sender - node responsible for communication ROS -> Inverter (changing ROS msg to CAN msg)
- inverter - node used for simulating inverter and testing data receiving via CAN bus 

## Installation

```bash
cd ~/dv_ws/src

git clone https://github.com/PUT-Motorsport/PUTM_DV_Steering_2020.git -b feature/inverter-steering-class-approach

```

## Usage

### Install [python-can](https://pypi.org/project/python-can/) package

```bash
pip install python-can
```

### Create a virtual CAN bus

```bash
sudo modprobe can_dev
sudo modprobe can_raw
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

### Create a CAN bus

**TODO**

### Give permissions to execute

```bash
chmod +x src/*.py
```

### ROS use case

```bash
cd ~/dv_ws

# build workspace
catkin_make

# source workspace
source devel/setup.bash
rosun CAN_apps APPS_publisher.py 
```
