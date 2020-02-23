#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def talker():
    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('chatter', String, queue_size=10)

    #Srate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        print("Valid Workspace -1:x:1 -1:y:1 -1:z:1")
        x = input("end effector position x:")
        y = input("end effector position y:")
        z = input("end effector position z:")
        position = str(x) + " " + str(y) + " " + str(z)
        rospy.loginfo(str(position))
        pub.publish(str(position))
        #rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
