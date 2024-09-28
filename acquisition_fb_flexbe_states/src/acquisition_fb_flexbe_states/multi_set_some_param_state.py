#!/usr/bin/env python
import rospy
import os

from flexbe_core import EventState, Logger
from acquisition_of_raw_data import multiservice_plex
from std_srvs.srv import EmptyRequest

class MultiSetSomeParamState(EventState):
    '''
    This will set a single parameter on multiple nodes from a list

    -- multi_node_list 	    list 	list of nodes
    -- param_to_set         str     string that will be a param_to_set of all services in the list
    -- value_of_param       str     value that this parameter will be set to

    <= done 			    it worked..
    <= failed 				some error occurred.

    '''

    def __init__(self, multi_node_list, param_to_set, value_of_param, check_if_nodes_exist=True):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(MultiSetSomeParamState, self).__init__(outcomes = ['done', 'failed'])
        
        #Logger.logerr("Started param set state")

        # Store state parameter for later use.
        self._param_to_set = param_to_set
        self._value_of_param = value_of_param
        self._check_if_nodes_exist_prior_to_setting_them = check_if_nodes_exist
        if type(multi_node_list) == type(""):
            multi_node_list = [multi_node_list]
        self._multi_node_list = multi_node_list

        self._param_list = []

        # constructs the list of params to be changed

        for a_node in self._multi_node_list:
            self._param_list.append(os.path.join(a_node,self._param_to_set))
            #Logger.logerr("added a node")

        self._error = []
        self._state = None

    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # Main purpose is to check state conditions and trigger a corresponding outcome.
        # If no outcome is returned, the state will stay active.

#		if rospy.Time.now() - self._start_time > self._multi_node_list:
        if len(self._error) == 0 and self._state == "Entered":
            return 'done' # One of the outcomes declared above.
        else:
            Logger.logerr(str(self._error))
            return 'failed'


    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
        # It is primarily used to start actions which are associated with this state.

        # actually sets the params

        for a_param in self._param_list:
            try:
                if rospy.has_param(a_param) or not self._check_if_nodes_exist_prior_to_setting_them:
                    Logger.loghint(f"setting param {a_param} to {self._value_of_param}")
                    rospy.set_param(a_param, self._value_of_param)
                else:
                    self._error.append(f"Param {a_param} did not already exist. This either means that this was called before the node was online or that either the node name or the parameter name is incorrect.")
            except rospy.ROSException as e:
                self._error.append(e)
        self._state = "Entered"

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

