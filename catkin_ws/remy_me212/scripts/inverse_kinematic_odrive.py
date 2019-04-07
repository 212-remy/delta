#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point

#####################################
realBot = True
#####################################

if realBot:
    import robot212_odrive as bot
else:
    import robot212_virtual as bot

import kinematicsSolver as kin
import time
import numpy as np

pi = np.pi #3.1415927
#bot.trajMoveRad((0,0,0))
deltaKin = kin.deltaSolver()


def callback(data):
    x = data.x
    y = data.y
    z = data.z
    thtDes = deltaKin.IK((x, y, z))
    print ("x:%3.5f    y:%3.5f    z:%3.5f" %(x, y, z))
    print (u"\u03b8\u2081:%3.5f    \u03b8\u2082:%3.5f    \u03b8\u2083:%3.5f" %(thtDes[0], thtDes[1], thtDes[2]))
    bot.trajMoveRad(thtDes, 2*pi/8, 2*pi/8) # (Desired Angles [rad], Max Velocity [rad/s], Acceleration/Deceleration [rad/s^2])
    deltaKin.updatePlot((x, y, z))
#    rospy.loginfo("x:%3.5f    y:%3.5f    z:%3.5f", %(x, y, z))

def listener():

    rospy.init_node('inverse_kinematics', anonymous=False)

    rospy.Subscriber('position', Point, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

