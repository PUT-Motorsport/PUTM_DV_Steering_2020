#!/usr/bin/env python

import can
import rospy as rp

from std_msgs.msg import String


class CANSender:
    def __init__(self):
        self.bustype = 'socketcan'
        self.channel = 'vcan0'
        self.arbitration_id = 0x0A
        self.regid = 0x31

        self.bus = can.interface.Bus(channel=self.channel,
                                     bustype=self.bustype)

        self.apps_msg = can.Message(arbitration_id=self.arbitration_id,
                                    extended_id=False,
                                    is_remote_frame=False,
                                    is_error_frame=False,
                                    data=[0x0, 0x0])  # To communicate with inverter change data format to [0x31, 0x0, 0x0]

        rp.Subscriber('apps', String, self.apps_callback)

    def set_msg(self, apps):
        self.apps_msg = can.Message(arbitration_id=self.arbitration_id,
                                    extended_id=False,
                                    is_remote_frame=False,
                                    is_error_frame=False,
                                    data=apps)

        rp.loginfo('CAN sender set msg: {}'.format(self.apps_msg))

    def can_send(self):
        while True:
            self.bus.send(self.apps_msg)
            rp.sleep(0.005)

    def apps_callback(self, data):
        rp.loginfo('CAN sender receive APPS msg: {}'.format(data.data))
        apps_str = ''.join(data.data)
        apps_int = int(apps_str)

        low_byte = apps_int & 0xff
        high_byte = apps_int >> 8

        rp.loginfo('CAN sender low byte: {}, high byte: {}'.format(low_byte, high_byte))

        self.set_msg([low_byte, high_byte])  # To communicate with inverter change msg format to [self.regid, low_byte, high_byte])


if __name__ == '__main__':
    rp.init_node('can_sender', log_level=rp.DEBUG)
    rp.loginfo('CAN sender starting...')

    CS = CANSender()
    CS.can_send()
    rp.spin()
