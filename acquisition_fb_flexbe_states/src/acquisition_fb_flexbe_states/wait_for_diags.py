#!/usr/bin/env python3
import rospy
import tf
from flexbe_core import EventState, Logger
from diagnostic_msgs.msg import DiagnosticStatus,DiagnosticArray
import traceback
from collections import deque

class WaitForDiags(EventState):
    '''
    Wait for all the diags in the list to be okay

    -- diags_list 	float 	Time which needs to have passed since the behavior started.

    <= continue 			Given time has passed.
    <= failed 				Example for a failure outcome.

    '''

    def __init__(self, diags_list, timeout=600, response_list_size=800):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(WaitForDiags, self).__init__(outcomes = ['continue', 'failed'])

        # Store state parameter for later use.

        if type(diags_list) == type(""):
            diags_list = [diags_list]
        
        self._diags_list = diags_list
        self._initial_diags_len = len(self._diags_list)
        self._initial_time = None
        self._timeout_time = rospy.Duration(timeout)
        self._response_deque = deque([], maxlen=response_list_size)


    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # Main purpose is to check state conditions and trigger a corresponding outcome.
        # If no outcome is returned, the state will stay active.
        if len(self._diags_list) == 0:
            return 'continue'

        #return 'continue'
        try:
            i=0
            
            while len(self._diags_list) > 0:
                #Logger.loghint("looking for stuff")
                if rospy.Time.now() > self._initial_time + self._timeout_time:
                    Logger.logerr("Timeout exceeded")
                    return 'failed'
                try:
                    some_response = rospy.wait_for_message("/diagnostics", DiagnosticArray, timeout=self._timeout_time.to_sec()/self._initial_diags_len)

                    self._response_deque.append(some_response)
                except rospy.ROSException as eee:
                    if i%100 == 1:
                        Logger.loghint(f"looking for diags from {a_diag} exceeded timeout, i think..")
                    continue
                for a_diag in self._diags_list:
                 #   if i%100 == 1:
                 #       i = 0
                    Logger.loghint(f"Looking for diags from {a_diag}")

                    #print(a_response.status[0])
                    #print(len(a_response.status))
                    #Logger.loginfo("{}".format(str(a_response)))
                    #Logger.loginfo("{}".format(str(a_response.status)))
                    #Logger.loginfo("{}".format(str(a_response.status[0])))
                    #a_response = DiagnosticArray()
                    for a_response in self._response_deque:
                        for status in a_response.status:
                            print(status)
                            if a_diag in status.name and a_diag in self._diags_list: 
                                self._diags_list.remove(a_diag)
                                if False:
                                    if status.level == status.OK:
                                        self._diags_list.remove(a_diag)
                                        break
                                    else:
                                        return 'failed'

        except Exception as e:
            st = traceback.format_stack()
            traceback.print_stack()
            Logger.logerr("I failed while waiting for diags: {}\n{}".format(str(e),str(st)))
            return 'failed'


    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
        # It is primarily used to start actions which are associated with this state.
        self._initial_time = rospy.Time.now()

    def on_exit(self, userdata):
        # This method is called when an outcome is returned and another state gets active.
        # It can be used to stop possibly running processes started by on_enter.

        pass # Nothing to do in this example.


    def on_start(self):
        # This method is called when the behavior is started.
        # If possible, it is generally better to initialize used resources in the constructor
        # because if anything failed, the behavior would not even be started.

        pass


    def on_stop(self):
        # This method is called whenever the behavior stops execution, also if it is cancelled.
        # Use this event to clean up things like claimed resources.

        pass # Nothing to do in this example.

