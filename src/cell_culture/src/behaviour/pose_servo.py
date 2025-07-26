import py_trees
import rospy
import actionlib
from cell_culture.msg import ControlEnableAction, ControlEnableGoal

class PoseServo(py_trees.behaviour.Behaviour):
    """
    This behavior is used to servo the robot's end effector to a desired cartesian pose specified by x, y, z coordinates in the robot's base frame.
    The behavior communicates with the control_node by sending a goal via a ROS action server, which handles the underlying motion execution.
    Moving the end-effector by a specified offset or an absolute position in the robot's base frame, is useful for open-loop motions like insertions, 
    retraction, or coarse positioning. Coordinates can be specified as relative to the current pose (use ~) or absolute (no ~).

    Returns `py_trees.common.Status.RUNNING` while the robot is in motion
    Returns `py_trees.common.Status.SUCCESS` when the target is reached
    Returns `py_trees.common.Status.FAILURE` if the motion is aborted

    Args:
        - name (:obj:`str`): name of the behaviour
        - x (:obj:`float`): x coordinate of the desired pose
        - y (:obj:`float`): y coordinate of the desired pose
        - z (:obj:`float`): z coordinate of the desired pose
        - kp (:obj:`float`): proportional gain for the servoing
        - sleep (:obj:`float`): time to wait before sending the goal to the action server
        - insertion (:obj:`bool`): whether this is an insertion motion (default: False) - will abort if the perceived end effector force rises above a threshold
    """
    def __init__(self, name, x, y, z, kp=0.009, sleep=0, insertion=False):
        name_str = f'PoseServo({x}, {y}, {z})'
        super(PoseServo, self).__init__(name=name_str)
        self.x = x
        self.y = y
        self.z = z
        self.kp = kp
        self.sleep = sleep
        self.insertion = insertion

    def feedback_cb(self, feedback):
        rospy.loginfo_once(f'[{self.__class__.__name__}] Feedback received: %s', feedback.status)
        self.feedback = feedback 

    def setup(self, timeout):
        rospy.loginfo(f'[{self.__class__.__name__}] Setting up behaviour')
        self.client = actionlib.SimpleActionClient('control_node', ControlEnableAction)
        self.client.wait_for_server()
        self.feedback_message = "setup"
        self.goal_sent = False
        rospy.loginfo(f'[{self.__class__.__name__}] Behaviour setup complete')
        return True

    def update(self):
        # Servo to desired pose using the Control Node action server

        rospy.loginfo_once(f'[{self.__class__.__name__}] Behaviour updating...')

        rospy.sleep(self.sleep)

        if not self.goal_sent:
            goal = ControlEnableGoal()
            goal.command = f'pose_servo {self.x} {self.y} {self.z} {self.kp} {self.insertion}'
            self.client.send_goal(goal, feedback_cb=self.feedback_cb)
            rospy.loginfo(f'[{self.__class__.__name__}] Sent goal: %s', goal.command)
            self.goal_sent = True

        action_state = self.client.get_state()
        if action_state == actionlib.GoalStatus.SUCCEEDED:
            self.goal_sent = False
            rospy.loginfo(f'[{self.__class__.__name__}] Completed with status: SUCCEEDED')
            return py_trees.common.Status.SUCCESS
        elif action_state in [actionlib.GoalStatus.ABORTED, actionlib.GoalStatus.PREEMPTED]:
            self.goal_sent = False
            rospy.loginfo(f'[{self.__class__.__name__}] Completed with status: ABORTED or PREEMPTED')
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING


    def terminate(self, new_status):
        self.feedback_message = "cleared"