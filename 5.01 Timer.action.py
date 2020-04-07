                                                      # This is an action definition file, which has three parts: 
                                                      # the goal, the result, and the feedback.
                                                      
duration time_to_wait                                 # Part 1: the goal, to be sent by the client
                                                      
                                                      # The amount of time we want to wait

---
                                                      # Part 2: the result, to be sent by the server upon completion
                                                      
                                                      # How much time we waited
duration time_elapsed
                                                      # How many updates we provided along the way
uint32 updates_sent
---
                                                      # Part 3: the feedback, to be sent periodically by the server during
                                                      # execution.
                                                      #
                                                      # The amount of time that has elapsed from the start
duration time_elapsed
                                                      # The amount of time remaining until we're done
duration time_remaining

"""
Just like with service-definition files, we use three dashes (---) as the separator
between the parts of the definition. While service definitions have two parts (request
and response), action definitions have three parts (goal, result, and feedback).
"""
