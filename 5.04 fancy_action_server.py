#! /usr/bin/env python
import rospy

import time
import actionlib
from basics.msg import TimerAction, TimerGoal, TimerResult, TimerFeedback         # Let’s step through the changes 
                                                                                  # with respect to 5.02. 
  
                                                                                  # Because we will be providing feedback, 
                                                                                  # we add TimerFeedback to the list of message types
                                                                                  # that we import: 

def do_timer(goal):
  start_time = time.time()
  update_count = 0                                                                # Stepping inside our do_timer() callback, 
                                                                                  # we add a variable (update_count) 
                                                                                  # that will keep track of
                                                                                  # how many times we publish feedback:

  if goal.time_to_wait.to_sec() > 60.0:                                           # 
    result = TimerResult()
    result.time_elapsed = rospy.Duration.from_sec(time.time() - start_time)
    result.updates_sent = update_count
    server.set_aborted(result, "Timer aborted due to too-long wait")
    return                                                                        

  """
Next, we add some error checking. 
We don’t want this timer to be used for long waits,
so we check whether the requested time_to_wait is greater than 60 seconds, and if
so, we explicitly abort the goal by calling set_aborted().

This call sends a message to
the client notifying it that the goal has been aborted. Like with set_succeeded(), we
include a result; doing this is optional, but a good idea if possible.

We also include a status string 
to help the client understand what happened; 
in this case, we aborted 
because the requested wait was too long. 

Finally, we return from the callback because
we’re done with this goal:
  """
  
  
  
  while (time.time() - start_time) < goal.time_to_wait.to_sec():

    if server.is_preempt_requested():
      result = TimerResult()
      result.time_elapsed = \
        rospy.Duration.from_sec(time.time() - start_time)
      result.updates_sent = update_count
      server.set_preempted(result, "Timer preempted")
      return

    feedback = TimerFeedback()
    feedback.time_elapsed = rospy.Duration.from_sec(time.time() - start_time)
    feedback.time_remaining = goal.time_to_wait - feedback.time_elapsed
    server.publish_feedback(feedback)
    update_count += 1

    time.sleep(1.0)

  result = TimerResult()
  result.time_elapsed = rospy.Duration.from_sec(time.time() - start_time)
  result.updates_sent = update_count
  server.set_succeeded(result, "Timer completed successfully")

rospy.init_node('timer_action_server')
server = actionlib.SimpleActionServer('timer', TimerAction, do_timer, False)
server.start()
rospy.spin()
