#!/usr/bin/env python3

import rospy
from threading import Thread
from sensor_msgs.msg import JointState
from mrm_publisher.msg import instructionsMessage, responseMessage
import math
import numpy as np


class RobotState:
    def __init__(self):
        self.initJointState()
        self.initPublishersAndSubscribers()
        self.msg = instructionsMessage()
        self.clearMessage()
        # self.service = rospy.Service('instructions', instructions, self.sendStatus)

    def initJointState(self):
        self.joint_state = JointState()
        self.joint_state.name = ['joint1', 'joint2']
        self.joint_state.position = [0.0, 0.0]

    def initPublishersAndSubscribers(self):
        rospy.Subscriber("chatter", instructionsMessage, self.callback)
        self.pub = rospy.Publisher('chatter', instructionsMessage, queue_size=1)
        self.joint_pub = rospy.Publisher('joint_states', JointState, queue_size=1)

    def responseMessage(self, data):
        response = self.getJointStatePosition(data)
        if response[2] == 'Valid':
            joint1,joint2 = response[0],response[1]
            x,y,z = self.getForwardKinematics(joint1,joint2)
            self.joint_state.position = [joint1, joint2]
        else:
            x,y,z = 0.0,0.0,0.0
            joint1,joint2 = self.joint_state.position[0],self.joint_state.position[1]
        self.msg.x = x
        self.msg.y = y
        self.msg.z = z
        self.msg.joint1 = joint1
        self.msg.joint2 = joint2
        self.msg.moveSuccessful = response[2]
        self.msg.finalPosition = "Joint1 position: " + str(
            self.joint_state.position[0]) + ", Joint2 position: " + str(self.joint_state.position[1])

    def start(self):
        while not rospy.is_shutdown():
            self.updateJoints()
            self.publishMessage(self.msg)

    def publishMessage(self, message):
        self.pub.publish(message)

    def callback(self, data):
        if data.side == 'Client':
            self.responseMessage(data)

    def updateJoints(self):
        self.joint_state.header.stamp = rospy.Time.now()
        self.joint_pub.publish(self.joint_state)

    def getJointStatePosition(self, data):
        if data.driveStyle == 1:
            return data.joint1, data.joint2, 'Valid'
        t1, t2 = self.getInverseKinematics(data.x, data.y, data.z)
        if self.isValidPosition(t1, t2, data):
            return t1, t2, 'Valid'
        return self.joint_state.position[0], self.joint_state.position[1], 'End Effector Position Not Valid. Try Again!!'

    def isValidPosition(self, t1, t2, data):
        x, y, z = self.getForwardKinematics(t1, t2)
        return np.isclose(x, data.x) and np.isclose(y, data.y) and np.isclose(z, data.z)

    def getInverseKinematics(self, x, y, z):
        link1 = 1.0
        robotHeight = 0.6
        t1 = 0.0
        if x != 0.0 or y != 0.0:
            t1 = math.atan2(x, y)
        t2 = 0.0
        trueHeight = z - robotHeight
        if trueHeight <= 1 and trueHeight >= -0.4 and trueHeight != 0.0:
            t2 = math.asin(trueHeight / link1)
        return t1, t2

    def getForwardKinematics(self, t1, t2):
        link1 = 1.0
        robotHeight = 0.6
        z = link1 * math.sin(t2) + robotHeight
        x = 0.0
        y = 0.0
        if not z == link1 + robotHeight:
            x = link1 * math.sin(t1)
            y = link1 * math.cos(t1)
        return x, y, z

    def clearMessage(self):
        self.msg.driveStyle = 0
        self.msg.x = 0
        self.msg.y = 0
        self.msg.z = 0
        self.msg.joint1 = 0
        self.msg.joint2 = 0
        self.msg.moveSuccessful = 'N/A'
        self.msg.finalPosition = 'N/A'
        self.msg.side = 'Server'

if __name__ == '__main__':
    try:
        rospy.init_node('RobotModelServer')
        jsp = RobotState()
        jsp.start()
        # rospy.spin()
    except rospy.ROSInterruptException:
        pass
