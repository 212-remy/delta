#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point

import robot212_odrive as bot
import kinematicsSolver as kin
import time
import numpy as np

pi = np.pi #3.1415927
bot.trajMoveRad((0,0,0))
deltaKin = kin.deltaSolver()


def callback(data):
    x, y, z = data
    thtDes = deltaKin.IK((x, y, z))
    rospy.loginfo("x:%3.5f    y:%3.5f    z:%3.5f", %(x, y, z))

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('inverse_kinematics', anonymous=False)

    rospy.Subscriber('position', Point, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()


