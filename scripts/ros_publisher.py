#!/usr/bin/env python
import rospy
from std_msgs.msg import String


def publish_apps(data):
    pub = rospy.Publisher('apps_topic', String, queue_size=10)
    rospy.init_node('apps_publisher', anonymous=False)
    rate = rospy.Rate(10)  # 10hz
    str_apps = (data)
    rospy.loginfo(str_apps)
    pub.publish(str_apps)
    print(str_apps)
    rate.sleep()


if __name__ == '__main__':

    while True:
        mes = input("Enter value to inverter or press q to quit:")
        if mes == 'q':
            break
        else:
            try:
                publish_apps(str(mes))
            except rospy.ROSInterruptException:
                pass
