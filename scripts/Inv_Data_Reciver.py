#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32MultiArray


bustype = 'socketcan'
channel = 'vcan0'


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard: %s', data.data)
    print(23)
    print(data)


def listener():
    rospy.init_node('InverterDataListiner', anonymous=False)
    rospy.Subscriber('InverterData', Int32MultiArray, callback)

    rospy.spin()


if __name__ == '__main__':
    print("hi")
    listener()
