import py_trees
import rospy
import actionlib
from cell_culture.msg import ControlEnableAction, ControlEnableFeedback, ControlEnableGoal
from cell_culture.msg import ImagingAction, ImagingGoal
from std_msgs.msg import Bool
from dynamic_reconfigure.client import Client

class Servo(py_trees.behaviour.Behaviour):
    """
    The Servo behavior is used to command the robot's end-effector toward a target pose, through
    perception-guided visual feedback. The behavior communicates with the control_node by sending a goal via a ROS action server, 
    which handles the underlying motion execution. When operating in visual servoing mode, the Servo behavior minimizes a 
    perception error vector provided by the perception_node to align a target with a detected object such as an AprilTag or well. 
    
    Returns `py_trees.common.Status.RUNNING` while the robot is in motion
    Returns `py_trees.common.Status.SUCCESS` when the target is reached
    Returns `py_trees.common.Status.FAILURE` if the target becomes undetectable or the motion is aborted

    Args:
        - name (:obj:`str`): name of the behaviour
        - z (:obj:`str`): target z coordinate to servo to (~ means relative to current z)
        - kp_z (:obj:`float`): proportional gain for the servoing in the z direction
        - kp_img (:obj:`float`): proportional gain for the servoing in the image plane
        - finish_thresh_img (:obj:`float`): threshold for the error vector in the image plane to consider the servoing finished
        - sleep (:obj:`float`): time to sleep before sending the goal
        - spiral (:obj:`bool`): invoke spiral search pattern
        - monitor (:obj:`bool`): whether to send an imaging goal after servoing is complete
        - rotational_stiff_z (:obj:`float`): rotational stiffness of the robot in the z direction, used to tune the robot's dynamical response when using impedance control
    """
    def __init__(self, name, z="~", kp_z=0.002, kp_img=0.00001, finish_thresh_img=1, sleep=0, spiral=False, monitor=False, rotational_stiff_z = 50):
        super(Servo, self).__init__(name=name)
        self.feedback = ControlEnableFeedback()
        self.z = z
        self.kp_z = kp_z
        self.kp_img = kp_img
        self.sleep = sleep
        self.finish_thresh_img = finish_thresh_img
        self.spiral = spiral
        self.monitor = monitor
        self.rotational_stiff_z = rotational_stiff_z

        self.perception_success_sub = rospy.Subscriber('/cell_culture/perception/success', Bool, self.perception_success_cb)
        self.perception_success = False

        self.global_reconfigure_client = Client('cell_culture_global_reconfigure', timeout=30, config_callback=self.global_reconfig_callback)
        self.monitoring_idx = 0

    def feedback_cb(self, feedback):
        rospy.loginfo_once(f'[{self.__class__.__name__}] Feedback received: %s', feedback.status)
        self.feedback = feedback 

    def perception_success_cb(self, msg):
        self.perception_success = msg.data

    def global_reconfig_callback(self, config):
        self.monitoring_idx = config['monitoring_idx']

    def setup(self, timeout):
        rospy.loginfo(f'[{self.__class__.__name__}] Setting up behaviour')
        self.blackboard = py_trees.blackboard.Blackboard()
        self.client = actionlib.SimpleActionClient('control_node', ControlEnableAction)
        self.client.wait_for_server()
        self.imaging_client = actionlib.SimpleActionClient('imaging', ImagingAction)
        self.imaging_client.wait_for_server()
        self.client.cancel_all_goals()
        self.goal_sent = False
        self.feedback_message = "setup"
        rospy.loginfo(f'[{self.__class__.__name__}] Behaviour setup complete')
        return True

    def update(self):
        rospy.loginfo_once(f'[{self.__class__.__name__}] Behaviour updating...')

        # If perception is not successful, we stop servoing
        if self.perception_success is False:
            if self.spiral: # if we are spiraling, we don't care if we don't see the target
                return py_trees.common.Status.RUNNING
            if self.goal_sent:
                self.client.cancel_all_goals()
                self.goal_sent = False
            rospy.loginfo(f'[{self.__class__.__name__}] No Target detected - stopping servoing.')
            return py_trees.common.Status.FAILURE
        
        rospy.sleep(self.sleep)

        if not self.goal_sent:
            goal = ControlEnableGoal()

            # Contruct command string - parsed by the control_node
            if self.spiral:
                goal.command = f'spiral'
            else:
                goal.command = f'servo {self.z} {self.kp_z} {self.kp_img} {self.finish_thresh_img} {self.rotational_stiff_z}'

            rospy.loginfo(f'[{self.__class__.__name__}] Sent goal: {goal.command}')
            self.client.send_goal(goal, feedback_cb=self.feedback_cb)
            self.goal_sent = True
            self.blackboard.currently_servoing = True

        action_state = self.client.get_state()
        if action_state == actionlib.GoalStatus.SUCCEEDED:
            self.goal_sent = False
            rospy.loginfo(f'[{self.__class__.__name__}] Completed with status: SUCCEEDED')
            self.blackboard.currently_servoing = False

            if self.monitor:
                # send image goal
                img_goal = ImagingGoal()
                img_goal.image = self.monitoring_idx
                self.imaging_client.send_goal(img_goal)
                rospy.sleep(0.5)

                if not self.monitoring_idx == 7:
                    self.global_reconfigure_client.update_configuration({"monitoring_idx": self.monitoring_idx + 1})
                    
            return py_trees.common.Status.SUCCESS
        
        elif action_state in [actionlib.GoalStatus.ABORTED, actionlib.GoalStatus.PREEMPTED]:
            self.goal_sent = False
            rospy.loginfo(f'[{self.__class__.__name__}] Completed with status: ABORTED or PREEMPTED')
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING


    def terminate(self, new_status):
        self.feedback_message = "cleared"