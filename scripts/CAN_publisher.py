#!/usr/bin/env python
import rospy
import can
from std_msgs.msg import String


class CANSender:

    def __init__(self):
        rospy.init_node('CAN_publisher', anonymous=False)
        self.bustype = 'socketcan'
        self.channel = 'vcan0'
        self.bus = can.interface.Bus(channel=self.channel,
                                     bustype=self.bustype)
        self.apps_msg = can.Message(arbitration_id=0x210, extended_id=False,
                                    is_remote_frame=False,
                                    is_error_frame=False,
                                    data=[0x31, 0x0, 0x0])
        rospy.Subscriber('apps_topic', String, self.callback)

    def set_msg(self, apps):
        self.apps_msg = can.Message(arbitration_id=0x210, extended_id=False,
                                    is_remote_frame=False,
                                    is_error_frame=False,
                                    data=apps)
        print(self.apps_msg)

    def can_send(self):
        while True:
            self.bus.send(self.apps_msg)

    def callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + 'I heard: %s', data.data)
        apps_str = ''.join(data.data)  # converting list into string

        apps_int = int(apps_str)

        low_byte = apps_int & 0xff
        high_byte = apps_int >> 8
        print('low_byte:', low_byte)
        print('high_byte:', high_byte)
        self.set_msg([0x31, high_byte, low_byte])


if __name__ == '__main__':
    CS = CANSender()
    CS.can_send()
    rospy.spin()
