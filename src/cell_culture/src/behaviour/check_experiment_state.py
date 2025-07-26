import py_trees
import rospy
from dynamic_reconfigure.client import Client

'''
The CheckExperimentState behavior evaluates the value of a runtime parameter
exposed through the dynamic_reconfigure server to control conditional branching in the behavior tree. The behavior
evaluates the current value of a specified parameter against a desired target value using a comparison operator (e.g., equals,
greater than, less than). If the condition holds, the behavior returns SUCCESS; otherwise, it returns FAILURE.
This node is often used to guard access to downstream actions, such as checking whether a pipette is currently held
before attempting to grasp one.
'''

class CheckExperimentState(py_trees.behaviour.Behaviour):
    def __init__(self, name, variable_to_check, desired_value, operator="=", invert=False):
        super(CheckExperimentState, self).__init__(name=name)
        self.variable_to_check = variable_to_check
        self.desired_value = desired_value
        self.operator = operator
        self.invert = invert

        # Dynamic reconfigure parameters
        self.tip_rack_count = None
        self.holding_pipette = None
        self.pipette_status = None
        self.pipette_actuator_pos = None
        self.needs_split = ""
        self.needs_media = ""
        self.needs_yeast = ""

    def dyn_reconfig_callback(self, config):
        rospy.logdebug("[{}] dynamic_reconfigure_callback triggered".format(self.__class__.__name__))
        self.tip_rack_count = config["tip_rack_count"]
        self.holding_pipette = config["holding_pipette"]
        self.pipette_status = config["pipette_status"]
        self.pipette_actuator_pos = config["pipette_actuator_pos"]
        self.needs_split = config["needs_split"]
        self.needs_media = config["needs_media"]
        self.needs_yeast = config["needs_yeast"]


    def setup(self, timeout):
        rospy.logdebug("[{}] Setting up behavior".format(self.__class__.__name__))
        client = Client("cell_culture_global_reconfigure", timeout=30, config_callback=self.dyn_reconfig_callback)
        return True

    def update(self):
        rospy.loginfo_once("[{}] Behaviour updating...".format(self.__class__.__name__))
        
        if self.operator == "=":  # default: check equality
            if getattr(self, self.variable_to_check) == self.desired_value:
                result = py_trees.common.Status.SUCCESS if not self.invert else py_trees.common.Status.FAILURE
                rospy.logdebug("[{}] {} is {}; Returning {}".format(
                    self.__class__.__name__,
                    self.variable_to_check,
                    self.desired_value,
                    result.name
                ))
                return result

        elif self.operator == ">":  # check greater than
            if getattr(self, self.variable_to_check) > self.desired_value:
                result = py_trees.common.Status.SUCCESS if not self.invert else py_trees.common.Status.FAILURE
                rospy.logdebug("[{}] {} is greater than {}; Returning {}".format(
                    self.__class__.__name__,
                    self.variable_to_check,
                    self.desired_value,
                    result.name
                ))
                return result

        elif self.operator == "<":  # check less than
            if getattr(self, self.variable_to_check) < self.desired_value:
                result = py_trees.common.Status.SUCCESS if not self.invert else py_trees.common.Status.FAILURE
                rospy.logdebug("[{}] {} is less than {}; Returning {}".format(
                    self.__class__.__name__,
                    self.variable_to_check,
                    self.desired_value,
                    result.name
                ))
                return result
        
        result = py_trees.common.Status.FAILURE if not self.invert else py_trees.common.Status.SUCCESS
        rospy.logdebug("[{}] {} is not {}; (it is {}); Returning {}".format(
            self.__class__.__name__,
            self.variable_to_check,
            self.desired_value,
            getattr(self, self.variable_to_check),
            result.name
        ))
        return result

    def terminate(self, new_status):
        self.feedback_message = "cleared"
