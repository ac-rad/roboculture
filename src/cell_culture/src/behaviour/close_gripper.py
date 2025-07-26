import py_trees
import rospy
from std_msgs.msg import String

class CloseGripper(py_trees.behaviour.Behaviour):
    """
    Close the gripper
    
    Returns :attr:`~py_trees.common.Status.SUCCESS` when the robot has closed the gripper
    """
    def __init__(self, name):
        super(CloseGripper, self).__init__(name=name)
        self.gripper_pub = rospy.Publisher("franka_gripper", String, queue_size=10)

    def setup(self, timeout):
        return True

    def update(self):
        """
        Close the gripper
        """
        
        rospy.loginfo("[Close Gripper] Closing gripper.")	
        self.gripper_pub.publish("close_gripper 240")	
        rospy.sleep(2)	
        return py_trees.common.Status.SUCCESS


    def terminate(self, new_status):
        """
        Args:
            new_status (:class:`~py_trees.common.Status`): the behaviour is transitioning to this new status
        """
        self.feedback_message = "cleared"