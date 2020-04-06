#!/usr/bin/env python

import rospy

from basics.srv import WordCount

import sys


rospy.init_node('service_client')

rospy.wait_for_service('word_count')                                    # First, we wait for the service to be advertised by the server:
                                                                        # If we try to use the service before it’s advertised, 
                                                                        # the call will fail with an exception.
    
                                                                        # This is a major difference between topics and services. 
                                                                        # We can subscribe to topics that are not yet advertised, 
                                                                        # but we can only use advertised services. 
                                                                        
word_counter = rospy.ServiceProxy('word_count', WordCount)              # Once the service is advertised, 
                                                                        # we can set up a local proxy for it:

words = ' '.join(sys.argv[1:])                                          # We need to specify the name of the service (word_count)
                                                                        # and the type (WordCount). 
                                                                       

word_count = word_counter(words)                                        # This will allow us to use word_counter 
                                                                        # like a local function that, when called, 
                                                                        # will actually make the service call for us:

print words, '->', word_count.count
