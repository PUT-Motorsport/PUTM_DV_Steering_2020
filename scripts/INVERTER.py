#!/usr/bin/env python

import can
import rospy as rp


class Inverter:
    def __init__(self):
        self.bustype = 'socketcan'
        self.channel = 'vcan0'
        self.arbitration_id = 0x0A
        self.current_speed_id = 0x0B

        self.bus = can.interface.Bus(channel=self.channel,
                                     bustype=self.bustype)

        self.apps_msg = can.Message(arbitration_id=self.current_speed_id,
                                    extended_id=False,
                                    is_remote_frame=False,
                                    is_error_frame=False,
                                    data=[0xA, 0xA, 0xA, 0xA, 0xA, 0xA, 0x0, 0x0])

        self.speed = 0
        self.set_speed = 25.0

    def handle_inverter_can_msg(self):
        while True:
            message = self.bus.recv()
            # print(str(self.speed) + " " + str(self.set_speed))

            if message.arbitration_id == self.arbitration_id:
                dlc = message.dlc
                inv_data = [int(message.data[i]) for i in range(dlc)]
                rp.loginfo('Self_speed, {}  {}'.format(self.speed, self.set_speed))

                self.set_speed = 256*inv_data[1] + inv_data[0]

            if self.set_speed - self.speed >= 2:
                self.speed = self.speed + 1
            elif self.set_speed - self.speed <= -2:
                self.speed = self.speed - 1
            else:
                self.speed = self.speed
            low_byte = self.speed & 0xff
            high_byte = self.speed >> 8

            self.set_msg([low_byte, high_byte, 0xA, 0xA, 0xA, 0xA, 0xA, 0xA])

            self.bus.send(self.apps_msg)

    def set_msg(self, msg):
        self.apps_msg = can.Message(arbitration_id=self.current_speed_id,
                                    extended_id=False,
                                    is_remote_frame=False,
                                    is_error_frame=False,
                                    data=msg)


if __name__ == '__main__':
    rp.init_node('inverter', log_level=rp.DEBUG)
    rp.loginfo('Inverter starting...')
    rp.sleep(20)

    Inv = Inverter()

    Inv.handle_inverter_can_msg()

    rp.spin()
