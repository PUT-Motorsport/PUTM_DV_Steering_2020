#!/usr/bin/env python

import rospy as rp
import numpy as np
import matplotlib.pyplot as plt

from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray


class APPS:
    def __init__(self):
        self.apps_publisher = rp.Publisher('apps', String, queue_size=10)
        rp.Subscriber('time_to_set', Float64MultiArray, self.apps_callback)

        self.current_speed = 0
        self.last_mesure_time = 0
        self.last_set_time = 0

        self.eps = 2

    def publish_apps_msg(self, apps_msg):
        rp.loginfo('APPS publish value: {}'.format(apps_msg))
        self.apps_publisher.publish(str(apps_msg))

    def apps_callback(self, data):
        # rp.loginfo('current_s, last_speed_time, set_set_time: {} {} {}'.format(data.data[0], data.data[1], data.data[2]))
        self.current_speed = data.data[0]
        self.last_mesure_time = data.data[1]
        self.last_set_time = data.data[2]
        # rp.loginfo('time to set: '.format(data.data[1] - data.data[2]))

    def convert_speed(self, speed):
        return(int(speed))              # TO DO: CHCECHK HOW TO CONVERT SPEED TO APPS MESSAGE

    def test(self, start, stop):
        stop = self.convert_speed(stop)
        start = self.convert_speed(start)

        self.publish_apps_msg(start)
        delta = abs(self.current_speed - start)
        while delta > self.eps:
            delta = abs(self.current_speed - start)

        rp.loginfo('speed: {}'.format(self.current_speed))

        self.publish_apps_msg(stop)
        while True:
            delta = self.current_speed - stop
            time = self.last_mesure_time - self.last_set_time
            if abs(delta) < self.eps:
                return time


if __name__ == '__main__':
    rp.init_node('apps_publisher', log_level=rp.DEBUG)
    rp.loginfo('APPS publisher starting...')

    A = APPS()
    rp.sleep(20)
    start = 0
    stop = 50
    resolution = 3

    step = (stop-start)/(resolution-1)

    X = np.zeros(resolution**2)
    Y = np.zeros(resolution**2)
    tab = np.zeros(resolution**2)
    k = 0

    time_tab = np.zeros([resolution, resolution])

    for i in range(resolution):
        for j in range(resolution):
            if i != j and time_tab[i][j] == 0:
                X[i*resolution+j] = start+i*step
                Y[i*resolution+j] = start+j*step
                time_tab[i][j] = A.test(start+i*step, start+j*step)
                tab[k] = A.test(start+i*step, start+j*step)
                k += 1

                X[j*resolution+i] = start+j*step
                Y[j*resolution+i] = start+i*step
                time_tab[j][i] = A.test(start+j*step, start+i*step)
                tab[k] = A.test(start+j*step, start+i*step)
                k += 1
                print(time_tab)
            elif i == j:
                X[i*resolution+j] = start+j*step
                Y[i*resolution+j] = start+i*step
                time_tab[i][j] = 0
                k += 1

    print(time_tab.reshape(resolution**2))
    print(X)
    print(Y)
    plt.scatter(X, Y, 300*time_tab.reshape(resolution**2))
    plt.show()
