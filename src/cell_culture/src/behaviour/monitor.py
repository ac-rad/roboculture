import py_trees
import rospy
import actionlib
from cell_culture.msg import ImagingAction, ImagingGoal
from dynamic_reconfigure.client import Client

"""
- This behaviour acts as an 'incrementor' which steps through the list of wells in monitor_pattern. 
- We increment self.curr_idx each time the behaviour is ticked, and we add to the blackboard self.monitor_pattern[self.curr_idx] as the 
current target (TODO: might make more sense to set dynamic reconfigure variable directly instead of going thru BB)
- monitor_pattern stores a list of wells which perceive will use
- Return success each time we increment self.curr_idx
- If self.curr_idx is None, we set it to 0 (first time running)
- If self.curr_idx is equal to the length of self.monitor_pattern, we return failure (we have reached the end of the list)
"""

class Monitor(py_trees.behaviour.Behaviour):
    def __init__(self, name, pattern, imaging=False):
        super(Monitor, self).__init__(name=name)
        self.monitor_pattern = pattern
        self.curr_idx = None
        self.imaging = imaging
        self.monitoring_idx = 0

        self.perception_reconfigure_client = Client('cc_perception_node', timeout=30, config_callback=self.perception_reconfig_callback)
        
        # Imaging action client used to communicate with perception node for imaging wells
        self.imaging_client = actionlib.SimpleActionClient('imaging', ImagingAction)
        self.imaging_client.wait_for_server()

        self.blackboard = py_trees.blackboard.Blackboard()
        
    def setup(self, timeout):
        return True
    
    def perception_reconfig_callback(self, config):
        pass

    def update(self):
        rospy.loginfo('[{}] Behaviour update'.format(self.__class__.__name__))

        if self.curr_idx is None:
            self.curr_idx = 0
        else:
            self.curr_idx += 1

        rospy.loginfo('[{}] Current idx: {}'.format(self.__class__.__name__, self.curr_idx))

        # Image wells
        if self.curr_idx > 0:
            if self.imaging:
                rospy.loginfo('[{}] Sending imaging goal, current idx: {}'.format(self.__class__.__name__, self.curr_idx))
                img_goal = ImagingGoal()
                img_goal.image = self.curr_idx - 1
                self.imaging_client.send_goal(img_goal)
                rospy.sleep(1)

        if self.curr_idx >= len(self.monitor_pattern):
            self.curr_idx = None
            self.blackboard.monitor_complete = True
            return py_trees.common.Status.FAILURE
                
        rospy.loginfo('[{}] Setting desired well to {}'.format(self.__class__.__name__, self.monitor_pattern[self.curr_idx]))
        new_config = {'desired_well': self.monitor_pattern[self.curr_idx]}
        self.perception_reconfigure_client.update_configuration(new_config)

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):

        self.feedback_message = "cleared"