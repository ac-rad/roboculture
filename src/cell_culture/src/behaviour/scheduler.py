import py_trees
import py_trees_ros
import rospy
import time
from dynamic_reconfigure.client import Client

class Scheduler(py_trees.behaviour.Behaviour):

    """
    Used to create a delay in the behavior tree, such as to wait a certain duration before proceeding to the next step.

    This behaviour will return SUCCESS after a specified duration in seconds has passed since the last success.

    Args:
        - name (:obj:`str`): name of the behaviour
        - duration (:obj:`int`): the duration in seconds to wait before returning SUCCESS again
    """

    def __init__(self, name="Scheduler", duration=30):
        super(Scheduler, self).__init__(name)
        self.last_success_time = time.time()
        self.duration = duration
        self.has_succeeded = False
        self.is_first_time = True
        self.global_reconfigure_client = Client('cell_culture_global_reconfigure', timeout=30, config_callback=self.dyn_reconfig_callback)
        self.monitoring_idx = 0

    def dyn_reconfig_callback(self, config):
        self.monitoring_idx = config['monitoring_idx']

    def setup(self, timeout):
        rospy.loginfo(f'[{self.__class__.__name__}] Setting up Scheduler behaviour')
        return True

    def initialise(self):
        self.last_success_time = time.time()

    def update(self):
        # Check if desired duration has passed since the last success
        current_time = time.time()

        if self.is_first_time:
            self.last_success_time = current_time
            self.is_first_time = False
            return py_trees.common.Status.SUCCESS

        if self.has_succeeded and self.monitoring_idx == 7: # hardcoding; if we are at the end of the monitoring pattern
            self.has_succeeded = False
            self.global_reconfigure_client.update_configuration({"monitoring_idx": 0})
            return py_trees.common.Status.SUCCESS
        
        if current_time - self.last_success_time >= self.duration:
            self.last_success_time = current_time
            self.has_succeeded = True

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.feedback_message = "cleared"