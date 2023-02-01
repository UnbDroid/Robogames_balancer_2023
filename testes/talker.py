#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
def talker():
    pub = rospy.Publisher('chatter', Float64)
    rospy.init_node('talker')
    count = 0.0
    while not rospy.is_shutdown():
        rospy.loginfo(count)
        pub.publish(Float64(count))
        rospy.sleep(1.0)
        count = count + 1
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass