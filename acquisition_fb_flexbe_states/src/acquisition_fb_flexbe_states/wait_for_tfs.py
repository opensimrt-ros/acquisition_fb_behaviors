#!/usr/bin/env python3
import rospy
import tf
from flexbe_core import EventState, Logger


class WaitForTfsState(EventState):
    '''
    Wait for all the tfs in the list to be available

    -- tf_list 	float 	Time which needs to have passed since the behavior started.

    <= continue 			Given time has passed.
    <= failed 				Example for a failure outcome.

    '''

    def __init__(self, tf_list, reference_frame = "map"):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(WaitForTfsState, self).__init__(outcomes = ['continue', 'failed'])

        # Store state parameter for later use.
        if type(tf_list) == type(""):
            tf_list = [tf_list]
        self._tf_list = tf_list

        self._tfl = tf.TransformListener()

        self._reference_frame = reference_frame

    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # Main purpose is to check state conditions and trigger a corresponding outcome.
        # If no outcome is returned, the state will stay active.
        if len(self._tf_list) == 0:
            return 'continue'

        try:
            for a_frame in self._tf_list:
                Logger.loghint(f"looking for transform from {self._reference_frame} to {a_frame}")
                if self._tfl.canTransform(self._reference_frame, a_frame,rospy.Time.now()):
                    self._tf_list.remove(a_frame)
                    break

        except tf.Exception as e:
            Logger.logerror(e)
            return 'failed'


    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
        # It is primarily used to start actions which are associated with this state.
        pass


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

