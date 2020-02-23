#!/usr/bin/env python3
# license removed for brevity
import rospy
import math
import numpy as np
from mrm_publisher.msg import instructionsMessage, responseMessage


class RobotModelClient:

    def __init__(self):
        rospy.init_node('robotModelClient', anonymous=True)
        self.pub = rospy.Publisher('chatter', instructionsMessage, queue_size=1)
        # rospy.Subscriber("chatter2", responseMessage, self.callback)
        self.msg = instructionsMessage()

    def start(self):
        while not rospy.is_shutdown():
            self.clearMessage()
            print("Drive robot with joint angles or End effector position")
            print("Joint angles type 1 for End effector position type 2")
            driveStyle = self.returnInputIfValid("Which drive Style 1 or 2:")
            print("\n \n")
            self.msg.driveStyle = driveStyle
            if self.msg.driveStyle == 1:
                print("Valid angles for joint1 and joint2 in radians")
                print("joint1 full pi and joint2 full pi")
                t1 = self.returnInputIfValid("radians for joint1:")
                t2 = self.returnInputIfValid("radians for joint2:")
                self.msg.joint1 = t1
                self.msg.joint2 = t2
                print("Angles are Valid")
            elif self.msg.driveStyle == 2:
                print("Valid Workspace -1:x:1 -1:y:1 -0.4:z:1.6")
                xEnd = self.returnInputIfValid("end effector position x:")
                yEnd = self.returnInputIfValid("end effector position y:")
                zEnd = self.returnInputIfValid("end effector position z:")
                self.msg.x = xEnd
                self.msg.y = yEnd
                self.msg.z = zEnd
                #print("End effector position is Valid")
            else:
                print("Drive Style is invalid")
                print("Try Again!")
            print("\n")
            #rospy.loginfo(self.msg)
            self.pub.publish(self.msg)
            response = rospy.wait_for_message("chatter", instructionsMessage)
            rospy.loginfo(response)
            # rospy.wait_for_service('instructions')
            # try:
            #     response = rospy.ServiceProxy('instructions', instructions)
            #     print(response())
            # except Exception as e:
            #     print(e)
            print("\n \n")
            # rospy.spin()
            # rate.sleep()

    def callback(self, data):
        # response = rospy.wait_for_message("chatter2", responseMessage)
        rospy.loginfo(data)
        # rospy.loginfo(data)

    def clearMessage(self):
        self.msg.driveStyle = 0
        self.msg.x = 0
        self.msg.y = 0
        self.msg.z = 0
        self.msg.joint1 = 0
        self.msg.joint2 = 0
        self.msg.moveSuccessful = 'N/A'
        self.msg.finalPosition = 'N/A'
        self.msg.side = 'Client'

    def returnInputIfValid(self, question):
    	while True:
    		num = raw_input(question)
    		try:
    			val = int(num)
    			break;
    		except ValueError:
    			try:
    				val = float(num)
    				break;
    			except ValueError:
    				print("This is not a number. Please enter a valid number")
    	return val

if __name__ == '__main__':
    try:
        robotModelClient = RobotModelClient()
        robotModelClient.start()
    except rospy.ROSInterruptException:
        pass
