#!/usr/bin/env python

import rospy as rp
import can
from std_msgs.msg import Float64MultiArray


class timer:
    def __init__(self):
        self.time_publisher = rp.Publisher('time_to_set', Float64MultiArray, queue_size=10)

        self.current_speed = 0

        self.last_speed_time = 0
        self.set_speed_time = 0

        self.bustype = 'socketcan'
        self.channel = 'vcan0'
        self.set_speed_arbitration_id = 0x0A
        self.current_speed_arbitration_id = 0x0B

        self.bus = can.interface.Bus(channel=self.channel,
                                     bustype=self.bustype)

    def publish_time_msg(self, time_msg):
        my_array_for_publishing = Float64MultiArray()
        my_array_for_publishing.data = time_msg

        # rp.loginfo('Time publish value: {}'.format(time_msg))
        self.time_publisher.publish(my_array_for_publishing)

    def handle_inverter_can_msg(self):
        last_data = [1, 2]
        while True:
            message = self.bus.recv()

            if message.arbitration_id == self.current_speed_arbitration_id:
                speed = 256*int(message.data[1]) + int(message.data[0])
                self.current_speed = speed
                rp.loginfo('currentspeed data {}'.format(speed))
                timestamp = message.timestamp
                self.last_speed_time = timestamp

            if message.arbitration_id == self.set_speed_arbitration_id:
                if last_data != message.data:
                    timestamp = message.timestamp
                    time = self.set_speed_time - timestamp
                    self.set_speed_time = timestamp
                    rp.loginfo('set speed data {}'.format(time))
                last_data = message.data

            self.publish_time_msg([self.current_speed, self.last_speed_time, self.set_speed_time])


if __name__ == '__main__':
    rp.init_node('timer', log_level=rp.DEBUG)
    rp.loginfo('Time publisher starting...')
    rp.sleep(20)
    A = timer()

    A.handle_inverter_can_msg()

    rp.spin()
