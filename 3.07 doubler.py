#!/usr/bin/env python

import rospy

from std_msgs.msg import Int32


rospy.init_node('doubler')


def callback(msg):
  doubled = Int32()
  doubled.data = msg.data * 2

  pub.publish(doubled)


sub = rospy.Subscriber('number', Int32, callback)
pub = rospy.Publisher('doubled', Int32)

rospy.spin()

"""
The subscriber and publisher are set up as before, but now we’re going to publish data
in the callback, rather than periodically. The idea behind this is that we only want to
publish when we have new data coming in, since the purpose of this node is to transform
data (in this case, by doubling the number that comes in on the subscribed
topic).
"""
