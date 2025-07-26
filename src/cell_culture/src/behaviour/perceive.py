import py_trees
import rospy
import actionlib
from cell_culture.msg import PerceptionEnableAction, PerceptionEnableResult, PerceptionEnableFeedback, PerceptionEnableGoal
from dynamic_reconfigure.client import Client

class Perceive(py_trees.behaviour.Behaviour):
    """
    This behavoiur sends a command via a ROS action server to the Perception Node 
    to begin searching for a specified target. 
    
    Returns `py_trees.common.Status.RUNNING` while the perception_node is actively processing the goal (i.e. SAM is processing)
    Returns `py_trees.common.Status.SUCCESS` if the perception_node begins publishing error vectors
    Returns `py_trees.common.Status.FAILURE` if the perception fails (e.g., the target is not detected)

    Args:
        - name (:obj:`str`): name of the behaviour
        - target (:obj:`str`): the target to perceive (e.g., 'april_tag', 'plate_tip_well', 'plate_cam_well', 'cancel')
        - num (:obj:`str`): the number associated with the target (e.g., AprilTag ID, well number)
        - offset (:obj:`list` of :obj:`float`): the offset to apply to the target position in pixels
        - use_tip_offsets (:obj:`bool`): whether to use predefined offsets for the given target, useful for tip racks
    """
    def __init__(self, name, target, num="-", offset=[0.0, 0.0, 0.0], use_tip_offsets=False):
        self.name = f'Perceive({target}, {num})'
        super(Perceive, self).__init__(name=self.name)
        self.feedback = PerceptionEnableFeedback()

        self.target = target
        self.num = num
        self.offset = offset
        self.use_tip_offsets = use_tip_offsets

        # Dynamic Reconfigure variables
        self.tip_rack_count = 12
        self.needs_media = ""
        self.needs_split = ""
        self.needs_yeast = ""
        self.monitoring_idx = 0

        self.idx_to_offset = {
            # TIP OFFSETS (top view of tip rack)
            # 1  2
            # 3  4
            # 5  6
            # 7  8
            # 9  10
            # 11 12
            12: [88, 270],
            11: [-24, 270],
            10: [84, 160],
            9: [-24, 160],
            8: [80, 50],
            7: [-32, 50],
            6: [88, -52],
            5: [-24, -52],
            4: [88, -160],
            3: [-24, -160],
            2: [88, 206], # Using tag 7
            1: [-24, 206], # Using tag 7
        }


    def feedback_cb(self, feedback):
        rospy.loginfo(f'[{self.__class__.__name__}] Feedback received: {feedback.status}')
        self.feedback = feedback

    def setup(self, timeout):
        rospy.loginfo(f'[{self.__class__.__name__}] Setting up behaviour: {self.name}')
        self.blackboard = py_trees.blackboard.Blackboard()

        # Init action client
        self.perception_client = actionlib.SimpleActionClient('cc_perception_node', PerceptionEnableAction)
        self.perception_client.wait_for_server()

        # Reconfigure client
        self.reconfigure_client = Client('cell_culture_global_reconfigure', timeout=30, config_callback=self.dyn_reconfig_callback)
        
        self.goal_sent = False
        self.feedback_message = "setup"

        rospy.loginfo(f'[{self.__class__.__name__}] Behaviour setup complete')
        return True
    
    def dyn_reconfig_callback(self, config):
        self.tip_rack_count = config["tip_rack_count"]
        self.needs_media = config["needs_media"]
        self.needs_split = config["needs_split"]
        self.needs_yeast = config["needs_yeast"]
        self.monitoring_idx = config["monitoring_idx"]
        rospy.loginfo(f"[{self.__class__.__name__}] DR Config set to: {self.tip_rack_count=}, {self.needs_media=}, {self.needs_split=}, {self.needs_yeast=}, {self.monitoring_idx=}")

    def update(self):
        """
        Perceive behaviour sends target name (AprilTag, desired well, tip) and number (id, well #) to Perception Node via ROS action server
        """
        rospy.loginfo_once(f'[{self.__class__.__name__}] Behaviour updating for {self.name}')
        rospy.loginfo_once(f'[{self.__class__.__name__}] Target: {self.target}')
        rospy.loginfo_once(f'[{self.__class__.__name__}] Num: {self.num}')
        rospy.loginfo_once(f'[{self.__class__.__name__}] Offset: {self.offset}')

        goal_num = self.num

        if self.target == 'cancel':
            if self.goal_sent:
                self.perception_client.cancel_all_goals()
                self.goal_sent = False
            rospy.loginfo(f'[{self.__class__.__name__}] Perception cancelled')

        if self.feedback.status == 'Failed':
            goal = PerceptionEnableGoal()
            goal.target = "cancel"
            goal.num = "-"
            rospy.loginfo(f'[{self.__class__.__name__}] Cancelling perception')
            return py_trees.common.Status.FAILURE
        

        # Set tip offset based on tip rack count - which tip to pick up based on number of tips remaining
        if self.target == "april_tag" and self.use_tip_offsets:
            tip_idx = self.tip_rack_count
            if tip_idx in [1,2]: # switch to other april tag - tips 1 and 2 are visible from the other tag
                goal_num = "7" ## TODO: make param elsewhere

            rospy.loginfo(f'[{self.__class__.__name__}] Tip idx: {tip_idx}')
            self.offset[0] = self.idx_to_offset[tip_idx][0]
            self.offset[1] = self.idx_to_offset[tip_idx][1]
            rospy.loginfo(f'[{self.__class__.__name__}] Tip offset: {self.offset}')


        if self.target == 'plate_tip_well' and not self.goal_sent:
            # goal_num is retreived from dynamic reconfigure; it gets assigned to the first well in the queue
            if goal_num == "needs_media":
                needs_media = self.needs_media.split(',')
                goal_num = needs_media[0]

            elif goal_num == "needs_yeast":
                needs_yeast = self.needs_yeast.split(',')
                goal_num = needs_yeast[0]

            elif goal_num == "needs_split":
                needs_split = self.needs_split.split(',')
                goal_num = needs_split[0]

        # Send goal to Perception Node if it hasn't been sent yet
        if not self.goal_sent and not self.blackboard.get("currently_servoing"):
            rospy.loginfo(f'[{self.__class__.__name__}] Sending goal to Perception Node')
            self.perception_client.cancel_all_goals()
            goal = PerceptionEnableGoal()
            goal.target = self.target
            goal.num = goal_num
            goal.x_offset = self.offset[0]
            goal.y_offset = self.offset[1]
            goal.z_offset = self.offset[2]
            rospy.loginfo(f'[{self.__class__.__name__}] goal.target: {goal.target}')
            rospy.loginfo(f'[{self.__class__.__name__}] goal.num: {goal_num}')
            rospy.loginfo(f'[{self.__class__.__name__}] goal.x_offset: {goal.x_offset}')
            rospy.loginfo(f'[{self.__class__.__name__}] goal.y_offset: {goal.y_offset}')

            self.perception_client.send_goal(goal, feedback_cb=self.feedback_cb)
            self.goal_sent = True

        action_state = self.perception_client.get_state()
        if action_state == actionlib.GoalStatus.SUCCEEDED:
            # self.goal_sent = False
            currently_servoing = self.blackboard.get("currently_servoing")
            if not currently_servoing:
                self.goal_sent = False
            rospy.loginfo(f'[{self.__class__.__name__}] Completed with status: SUCCEEDED')
            return py_trees.common.Status.SUCCESS
        elif action_state in [actionlib.GoalStatus.ABORTED, actionlib.GoalStatus.PREEMPTED]:
            if not currently_servoing:
                self.goal_sent = False
            rospy.loginfo(f'[{self.__class__.__name__}] Completed with status: ABORTED or PREEMPTED')
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.feedback_message = "cleared"