#!/usr/bin/env python
import rospy
import can
from std_msgs.msg import Int32MultiArray


bustype = 'socketcan'
channel = 'vcan0'


bus = can.interface.Bus(channel=channel, bustype=bustype)


def InverterDataHendler(message):
    pub = rospy.Publisher('InverterData', Int32MultiArray, queue_size=10)
    rospy.init_node('InverterDataReciver', anonymous=False)
    dlc = message.dlc
    InvData = []

    for i in range(0, dlc):
        InvData.append(int(message.data[i]))

    print(InvData)
    InvDataPublish = Int32MultiArray(data=InvData)

    pub.publish(InvDataPublish)


if __name__ == '__main__':
    print("hi")
    while True:
        message = bus.recv()
        print(message)
        if message.arbitration_id == 0x0A:
            InverterDataHendler(message)
