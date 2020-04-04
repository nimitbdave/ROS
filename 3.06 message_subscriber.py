#!/usr/bin/env python

import rospy
from basics.msg import Complex

def callback(msg):
  print 'Real:', msg.real
  print 'Imaginary:', msg.imaginary
  print


rospy.init_node('message_subscriber')

sub = rospy.Subscriber('complex', Complex, callback)

rospy.spin()


"""
Importing your new message type works just like including a standard ROS message
type and allows you to create a message instance just like any other Python class.

Once you’ve created the instance, you can fill in the values for the individual fields.
(Any fields that are not explicitly assigned a value should be considered to have an
undefined value.)

Subscribing to and using your new message is similarly easy, as shown in 3.06, here.
"""
