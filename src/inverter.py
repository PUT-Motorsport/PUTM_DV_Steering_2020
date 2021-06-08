#!/usr/bin/env python

import can
import rospy as rp


class Inverter:
    def __init__(self):
        self.bustype = rp.get_param('/can/bustype')
        self.channel = rp.get_param('/can/channel')
        self.arbitration_id = rp.get_param('/can/arbitration_id')

        self.bus = can.interface.Bus(channel=self.channel,
                                     bustype=self.bustype)


    def handle_inverter_can_msg(self):
        while True:
            message = self.bus.recv()

            if message.arbitration_id == self.arbitration_id:
                dlc = message.dlc
                inv_data = [int(message.data[i]) for i in range(dlc)]

                # rp.loginfo('Inverter receive: {}'.format(inv_data))


if __name__ == '__main__':
    rp.init_node('inverter', log_level=rp.DEBUG)

    Inv = Inverter()

    Inv.handle_inverter_can_msg()

    rp.spin()
