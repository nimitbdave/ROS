#!/usr/bin/env python

import rospy

from basics.msg import Complex

from random import random


rospy.init_node('message_publisher')

pub = rospy.Publisher('complex', Complex)                     # publisher, which calls Complex

rate = rospy.Rate(2)

while not rospy.is_shutdown():
  msg = Complex()                                             # calls message, Complex()
  msg.real = random()                                         # when real, random
  msg.imaginary = random()                                    # when imaginary, random

  pub.publish(msg)                                            # publish
  rate.sleep()
