#! /usr/bin/env python
import rospy

import actionlib
from basics.msg import TimerAction, TimerGoal, TimerResult

rospy.init_node('timer_action_client')                                    # Following the usual imports and initialization
                                                                          # of our ROS node, 
                                                                          
client = actionlib.SimpleActionClient('timer', TimerAction)               # we create a SimpleActionClient.
client.wait_for_server()
                                                                          # The first constructor argument 
                                                                          # is the name of the action server, 
                                                                          # which the client will use to determine the
                                                                          # topics that it will use when communicating with the server.
                                                                          # This name must match the one that we used 
                                                                          # in creating the server, which is 
                                                                          # 'timer'. 
                                                                          
                                                                          # The second argument is the type of the action, 
                                                                          # which must also match the server: 
                                                                          # TimerAction.
                                                                          
                                                                          # Having created the client, 
                                                                          # we tell it to wait for the action server to come up, 
                                                                          # which it does by checking for the five advertised topics 
                                                                          # that we saw earlier when testing the server. 
                                                                          # Similar to rospy.wait_for_service(), 
                                                                          # which we used to wait for a service to be ready, 
                                                                          # SimpleActionClient.wait_for_server() will block until
                                                                          # the server is ready:
                                                                          
                                                                          
goal = TimerGoal()                                                        # Now we create a goal of type TimerGoal 
                                                                          # and fill in the amount of time we want the
goal.time_to_wait = rospy.Duration.from_sec(5.0)                          # timer to wait, which is five seconds. 
client.send_goal(goal)                                                    # Then we send the goal, 
                                                                          # which causes the transmission
                                                                          # of the goal message to the server:

client.wait_for_result()                                                  # Next, we wait for a result from the server. 
                                                                          # If things are working properly, we expect to
                                                                          # block here for about five seconds. 
print('Time elapsed: %f'%(client.get_result().time_elapsed.to_sec()))     # After the result comes in, 
                                                                          # we use get_result() to retrieve it 
                                                                          # from within the client object and 
                                                                          # print out the time_elapsed field that was
                                                                          # reported by the server:

 
"""
Checking That Everything Works as Expected

Now that we have implemented the action client, we can get to work. Make sure that
your roscore and action server are still running, then run the action client:

  user@hostname$ rosrun basics simple_action_client.py
  Time elapsed: 5.001044

Between the invocation of the client and the printing of the result data, you should
see a delay of approximately five seconds, as requested. The time elapsed should be
slightly more than five seconds, because a call to time.sleep() will usually take a little
longer than requested.
"""
