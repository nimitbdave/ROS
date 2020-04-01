#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

def callback(msg):                                      # callback that handles the messages as they come in.  
  print msg.data                                        # This function simply prints out the data contained in the message.
  
                                                       
rospy.init_node('topic_subscriber')                     # Once a node has subscribed to a topic, 
                                                        # every time a message arrives on it 
                                                        # the associated callback function is called, 
                                                        # with the message as its parameter.


sub = rospy.Subscriber('counter', Int32, callback)      # After initializing the node, as before, we subscribe to the counter topic. 
                                                        # We give the name of the topic, the message type of the topic, 
                                                        # and the name of the callback function.
                                                        
                                                        # Behind the scenes, 
                                                        # the subscriber passes this information on to roscore 
                                                        # and tries to make a direct connection with the publishers of this topic. 
                                                        # If the topic does not exist, or if the type is wrong, 
                                                        # there are no error messages: 
                                                        # the node will simply wait until messages start being published on the topic.

rospy.spin()                                            # Once the subscription is made, 
                                                        # we give control over to ROS by calling rospy.spin().
                                                        
                                                        # This function will only 'return' when the node is ready to shut down. 
                                                        # This is just a useful shortcut to avoid having to define a while loop 
                                                        # like we did in 3.01; 
                                                        # ROS does not necessarily need to “take over” the main thread of execution.
