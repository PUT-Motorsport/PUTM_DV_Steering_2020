#!/usr/bin/env python

import rospy as rp

from std_msgs.msg import String


class APPS:
    def __init__(self):
        self.apps_publisher = rp.Publisher('apps', String, queue_size=10)

    
    def publish_apps_msg(self, apps_msg):
        rp.loginfo('APPS publish value: {}'.format(apps_msg))

        self.apps_publisher.publish(str(apps_msg))

        rp.sleep(10) # sleep 10 seconds


if __name__ == '__main__':
    rp.init_node('apps_publisher', log_level=rp.DEBUG)

    rp.sleep(15)
    rp.loginfo('APPS publisher starting...')

    apps_msgs = [0, 1100]
    msg_id = 0

    A = APPS()

    while True:
        apps_msg = apps_msgs[msg_id % len(apps_msgs)]

        A.publish_apps_msg(apps_msg)

        msg_id += 1
