#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger
from acquisition_of_raw_data import multiservice_plex
from opensimrt_msgs.srv import SetFileNameSrv, SetFileNameSrvRequest, SetFileNameSrvResponse

class VariableMultiSetNameAndPathState(EventState):
    '''

    sets path

    -- multi_service_list str[]     list of srvs
    -- prefix           str         prefix of srvs
    -- suffix           stf         suffix of srvs

    ># activity...      object[]    Input(s) to the calculation function as a list of userdata.
    #> activity...		object		Latest message on the given topic of the respective type.
    

    <= done 			Given time has passed.
    <= failed 				Example for a failure outcome.

    '''

    def __init__(self, prefix, suffix, activity_name, save_dir, subject_num):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(VariableMultiSetNameAndPathState, self).__init__(outcomes = ['done', 'failed'],
                                                        input_keys = ['activity_counter', 'activity_save_dir','activity_save_name','multi_service_list'],
                                                        output_keys = ['activity_counter', 'activity_save_dir','activity_save_name'])

        # Store state parameter for later use.

        self._activity_name = activity_name
        self._save_dir = save_dir
        self._subject_num = subject_num
        self._multi_service_list = []
        self._prefix = prefix
        self._predicate = suffix

        # The constructor is called when building the state machine, not when actually starting the behavior.
        # Thus, we cannot save the starting time now and will do so later.
        self._start_time = None
        self._responses = None

        self._multi_service_plex = None

    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # Main purpose is to check state conditions and trigger a corresponding outcome.
        # If no outcome is returned, the state will stay active.

#		if rospy.Time.now() - self._start_time > self._multi_service_list:
        if not self._multi_service_plex:
            Logger.logerr("MULTISERVICEPLEX IS NONE???")
            return 'failed'
        if self._multi_service_plex.error_list == [] and not self._responses == None:
            #Logger.loginfo("responses ok:%s"%self._responses)
            return 'done' # One of the outcomes declared above.
        else:
            Logger.logwarn(str(self._responses))
            Logger.logwarn(str(self._multi_service_plex.error_list))
            return 'failed'


    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
        # It is primarily used to start actions which are associated with this state.

        # The following code is just for illustrating how the behavior logger works.
        # Text logged by the behavior logger is sent to the operator and displayed in the GUI.

        #time_to_wait = (self._multi_service_list - (rospy.Time.now() - self._start_time)).to_sec()

        #if time_to_wait > 0:
        #	Logger.loginfo('Need to wait for %.1f seconds.' % time_to_wait)

        ## does call

        if type(userdata.multi_service_list) == type(""):
            userdata.multi_service_list = [userdata.multi_service_list]

        for an_srv_name in userdata.multi_service_list:
            self._multi_service_list.append(self._prefix+an_srv_name+self._predicate)
        Logger.loginfo("received list of services to be called: %s" % self._multi_service_list)

        self._multi_service_plex = multiservice_plex.MultiServiceCaller(self._multi_service_list, SetFileNameSrv(), SetFileNameSrvResponse())


        # we maybe dont want this, so we can rerecord another?
        ##userdata.activity_counter +=1
        if userdata.activity_save_name == "":
            Logger.loginfo("activity filename not set, setting now")
        
        userdata.activity_save_name = self._activity_name + str(userdata.activity_counter)
        
        if userdata.activity_save_dir == "":
            Logger.loginfo("activity filename not set, setting now")
        
        userdata.activity_save_dir = self._save_dir + str(self._subject_num)
        
        

        req = SetFileNameSrvRequest()
        req.name = userdata.activity_save_name
        req.path = userdata.activity_save_dir
        Logger.log("my req msg: "+str(req),Logger.REPORT_HINT )
        _, self._responses = self._multi_service_plex(req)


    def on_exit(self, userdata):
        # This method is called when an outcome is returned and another state gets active.
        # It can be used to stop possibly running processes started by on_enter.

        pass # Nothing to do in this example.


    def on_start(self):
        # This method is called when the behavior is started.
        # If possible, it is generally better to initialize used resources in the constructor
        # because if anything failed, the behavior would not even be started.

        # In this example, we use this event to set the correct start time.
        self._start_time = rospy.Time.now()


    def on_stop(self):
        # This method is called whenever the behavior stops execution, also if it is cancelled.
        # Use this event to clean up things like claimed resources.

        pass # Nothing to do in this example.

