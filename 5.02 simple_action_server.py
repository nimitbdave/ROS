#! /usr/bin/env python
import rospy

import time                                                                         # First we import the standard Python time package, 
                                                                                    # which we’ll use for the timer functionality 
                                                                                    # of our server.
    
import actionlib                                                                    # Also import the ROS actionlib package 
                                                                                    # that provides the SimpleActionServer class that 
                                                                                    # we’ll be using.
    
from basics.msg import TimerAction, TimerGoal, TimerResult                          # Finally, we import some of the message classes 
                                                                                    # that were autogenerated from our Timer.action file:

def do_timer(goal):                                                                 # Next, we define do_timer(), 
                                                                                    # the function that will be invoked 
                                                                                    # when we receive a new goal.
                                                                                      # In this function, we handle the new goal in-place
                                                                                      # and set a result before returning.
                                                                                    # The type of the "goal" argument that is passed 
                                                                                    # to do_timer() is TimerGoal, 
                                                                                    # which corresponds to the goal part of Timer.action.
                
  start_time = time.time()                                                          # We save the current time, 
                                                                                    # using the standard Python time.time() function
    
  time.sleep(goal.time_to_wait.to_sec())                                            # then sleep for the time requested in the goal,
                                                                                    # converting the time_to_wait field 
                                                                                    # from a ROS duration to seconds:
      
  result = TimerResult()                                                            # The next step is to build up the result message,
                                                                                    # which will be of type TimerResult;
                                                                                    # corresponds to the result part of Timer.action.
      
  result.time_elapsed = rospy.Duration.from_sec(time.time() - start_time)           # We fill in the time_elapsed field
                                                                                    # by subtracting our saved "start time" from 
                                                                                    # "current time," and simultaneously 
                                                                                    # convert the result to a ROS duration.
        
  result.updates_sent = 0                                                           # We set updates_sent to zero, 
                                                                                    # because we didn’t send any updates
                                                                                    # along the way (we’ll add that part shortly):
      
  server.set_succeeded(result)                                                      # Our final step in the callback 
                                                                                    # is to tell the SimpleActionServer 
                                                                                    # that we successfully achieved the goal 
                                                                                    # by calling set_succeeded() 
                                                                                    # and passing it the result. 
                                                                                    # For this simple server, we always succeed; 
                                                                                    # we’ll address failure cases later:

rospy.init_node('timer_action_server')                                              # Back in the global scope, 
                                                                                    # we initialize and name our node as usual
                                                                                    
server = actionlib.SimpleActionServer('timer', TimerAction, do_timer, False)        # then create a SimpleActionServer
                                                                                    # The first constructor argument 
                                                                                    # for SimpleActionServer is the server’s name,
                                                                                    # which will determine the namespace 
                                                                                    # into which its constituent topics 
                                                                                    # will be advertised; we’ll use "timer".
                                                                                    
                                                                                    # The second argument is the type of the action 
                                                                                    # that the server will be handling, 
                                                                                    # which in our case is TimerAction.
                                                                          
                                                                                    # The third argument is the goal callback, 
                                                                                    # which is the function do_timer() 
                                                                                    # that we defined earlier.
                                                                                    
                                                                                    # Finally, we pass False 
                                                                                    # to disable autostarting the server.
                                
server.start()                                                                      # Having created the action server, 
                                                                                    # we explicitly start() it, 
                                                                                    
rospy.spin()                                                                        # then go into the usual ROS spin() loop 
                                                                                    # to wait for goals to arrive:
