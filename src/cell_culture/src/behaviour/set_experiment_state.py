import py_trees
import py_trees_ros
import py_trees.console as console
import rospy
import numpy as np
import actionlib
from dynamic_reconfigure.client import Client

class SetExperimentState(py_trees.behaviour.Behaviour):

    """
    The dynamic_reconfigure ROS package is used to manage parameters which are useful for the execution of the experiment. 
    The package allows for the reconfiguration of parameters during execution, and also provides a graphical user interface. 
    The SetExperimentState behavior is used to programatically set, increment, or decrement one of the parameters stored in
    the dynamic reconfigure server. It always returns SUCCESS.
    """


    def __init__(self, name, variable_to_set, value, delay=0, relative=False):
        super(SetExperimentState, self).__init__(name=name)
        self.variable_to_set = variable_to_set
        self.value = value
        self.relative = relative
        self.delay = delay

        # Store numeric values for incrementing/decrementing
        self.tip_rack_count = None
        self.desired_well = None
        self.pipette_actuator_pos = None
        self.needs_media = ""
        self.needs_yeast = ""
        self.needs_split = ""

        self.dilution_factor = 3 ## TODO: make param

    def global_reconfig_callback(self, config):
        self.tip_rack_count = config["tip_rack_count"]
        self.pipette_actuator_pos = config["pipette_actuator_pos"]
        self.needs_media = config["needs_media"]
        self.needs_yeast = config["needs_yeast"]
        self.needs_split = config["needs_split"]
        self.pipette_status = config["pipette_status"]

    def perception_reconfig_callback(self, config):
        self.desired_well = config["desired_well"]

    def rgb_camera_callback(self, config):
        pass

    def setup(self, timeout):
        rospy.loginfo(f'[{self.__class__.__name__}] Setting up SetExperimentState behaviour')
        self.global_client = Client("cell_culture_global_reconfigure", timeout=30, config_callback=self.global_reconfig_callback)
        self.perception_client = Client("cc_perception_node", timeout=30, config_callback=self.perception_reconfig_callback)
        self.rgb_camera_reconfigure_client = Client("/camera/rgb_camera", timeout=30, config_callback=self.rgb_camera_callback)

        return True

    def update(self):
        rospy.sleep(self.delay)
        rospy.loginfo(f'[{self.__class__.__name__}] Behaviour update')

        # Set camera auto exposure
        if self.variable_to_set == "auto_exposure":
            if self.value == False:
                self.rgb_camera_reconfigure_client.update_configuration({"enable_auto_exposure": False, "enable_auto_white_balance": False, "auto_exposure_priority": False, "exposure": 550})
                rospy.sleep(5)
                return py_trees.common.Status.SUCCESS
            else:
                self.rgb_camera_reconfigure_client.update_configuration({"enable_auto_exposure": True, "enable_auto_white_balance": True, "auto_exposure_priority": True})
                rospy.sleep(5)
                return py_trees.common.Status.SUCCESS
        
        # Increment or decrement numeric values
        if self.value == "increment":
            rospy.loginfo(f'[{self.__class__.__name__}] Incrementing {self.variable_to_set}')
            val = getattr(self, self.variable_to_set) + 1
        elif self.value == "decrement":
            rospy.loginfo(f'[{self.__class__.__name__}] Decrementing {self.variable_to_set}')
            val = getattr(self, self.variable_to_set) - 1
        else:
            if self.relative:
                val = getattr(self, self.variable_to_set) + self.value
            else:
                val = self.value

        # Special case: setting pipette actuator position
        if self.variable_to_set == "pipette_actuator_pos" and val == "inhale_media":
            num_wells_needs_media = len(self.needs_media.split(","))
            val = 1850 - (num_wells_needs_media * 9) - 18 - 9
            self.pipette_actuator_pos = val

        elif self.variable_to_set == "pipette_actuator_pos" and val == "resuspend":
            # Special case: resuspend yeast
            for _ in range(3):
                self.global_client.update_configuration({"pipette_actuator_pos": 1822})
                rospy.sleep(5)
                self.global_client.update_configuration({"pipette_actuator_pos": 1849})
                rospy.sleep(5)
            return py_trees.common.Status.SUCCESS

        rospy.loginfo(f'[{self.__class__.__name__}] Setting {self.variable_to_set} to {val}')


        new_config = {self.variable_to_set: val}

        # Set pipette_status to empty if pipette_actuator_pos is 1850 (hardcoded)
        if self.variable_to_set == "pipette_actuator_pos" and val == 1850:
            if self.pipette_status == 3: # yeast -> contaminated
                new_config["pipette_status"] = 4
            elif self.pipette_status == 2: # media -> empty
                new_config["pipette_status"] = 1

        # Special cases: manipulating needs_split, needs_media, needs_yeast
        if self.variable_to_set == "needs_media":
            rospy.loginfo(f'[{self.__class__.__name__}] Removing desired_well from needs_media')
            needs_media = self.needs_media.split(",")
            if str(self.desired_well) in needs_media:
                needs_media.remove(str(self.desired_well))
                val = ",".join(needs_media)
                new_config["needs_media"] = val
        elif self.variable_to_set == "needs_split":
            rospy.loginfo(f'[{self.__class__.__name__}] Removing desired_well from needs_split')
            needs_split = self.needs_split.split(",")
            if str(self.desired_well) in needs_split:
                needs_split.remove(str(self.desired_well))
                val = ",".join(needs_split)
                new_config["needs_split"] = val

                # Populate needs_yeast: add to needs_yeast the wells in the subsequent rows based on dilution factor
                # Eg. if the desired_well is 1, then add 13,25,37 to needs_yeast because there are 12 wells in a row
                new_needs_yeast = []
                for i in range(1, self.dilution_factor+1):
                    new_needs_yeast.append(str(self.desired_well + (12 * i)))
                new_config["needs_yeast"] = ",".join(new_needs_yeast)
        elif self.variable_to_set == "needs_yeast":
            rospy.loginfo(f'[{self.__class__.__name__}] Removing desired_well from needs_yeast')
            needs_yeast = self.needs_yeast.split(",")
            if str(self.desired_well) in needs_yeast:
                needs_yeast.remove(str(self.desired_well))
                val = ",".join(needs_yeast)
                new_config["needs_yeast"] = val

        
        if self.variable_to_set in ["tip_rack_count", "holding_pipette", "pipette_status", "pipette_actuator_pos", "needs_split", "needs_media", "needs_yeast", "shaker_active"]:
            self.global_client.update_configuration(new_config)
        elif self.variable_to_set in ["desired_well", "target"]:
            rospy.loginfo(f'[{self.__class__.__name__}] Updating config for cc_perception_node')
            self.perception_client.update_configuration(new_config)

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.feedback_message = "cleared"
