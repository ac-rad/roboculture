#!/usr/bin/env python

import functools
import operator
import py_trees
import py_trees_ros
import py_trees.console as console
import rospy
import sys
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
from behaviour.servo import Servo
from behaviour.perceive import Perceive


class ToBlackboard(py_trees_ros.subscribers.ToBlackboard):
    """
    Subscribes to a topic, and writes data to the blackboard.

    When ticking, updates with :attr:`~py_trees.common.Status.RUNNING` if it got no data,
    :attr:`~py_trees.common.Status.SUCCESS` otherwise.

    Blackboard Variables:
        * perception_success (:obj:`bool`): whether the perception node has succeeded in finding the target

    Args:
        name (:obj:`str`): name of the behaviour
        topic_name (:obj:`str`) : name of the error vector topic
    """
    def __init__(self, name, topic_name, topic_type, blackboard_variables):
        super(ToBlackboard, self).__init__(name=name,
                                           topic_name=topic_name,
                                           topic_type=topic_type,
                                           blackboard_variables=blackboard_variables,
                                           clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE
                                           )
        self.blackboard = py_trees.blackboard.Blackboard()
        self.blackboard.perception_success = Bool()
        self.blackboard.currently_servoing = Bool()

    def update(self):
        """
        Call the parent to write the raw data to the blackboard and then check against the
        threshold to determine if the low warning flag should also be updated.
        """
        self.logger.debug("%s.update()" % self.__class__.__name__)
        status = super(ToBlackboard, self).update()
        if status != py_trees.common.Status.RUNNING:
            # we got something
            if type(self.blackboard.perception_success) is not bool:
                self.blackboard.perception_success = self.blackboard.perception_success.data

        return status

def create_root():
    # behaviours
    root = py_trees.composites.Parallel("ServoToTag")
    topics2bb = py_trees.composites.Sequence("Topics2BB")
    perception_success_2_bb = ToBlackboard(name="PerceptionSuccess2BB", topic_name="/cell_culture/perception/success", topic_type=Bool, blackboard_variables={'perception_success': None})

    priorities = py_trees.composites.Selector("Priorities")

    servo_to_april_tag = py_trees.composites.Sequence("ServoToAprilTag")

    # april_tag_found = py_trees.blackboard.CheckBlackboardVariable(
    #     name="AprilTagFound?",
    #     variable_name='tag_found',
    #     expected_value=True,
    # )
    perceive_tag_14 = Perceive(name="Perceive", target='april_tag', num="4", offset=[0, 0, 0.0]) # offsets are in image frame 
    perceive_tag_13 = Perceive(name="Perceive", target='april_tag', num="13", offset=[0.0, 0.0, 0.0])

    servo1 = Servo(name="Servo", finish_thresh_img=0.00001, z="0.594", kp_z=0.01, kp_img=0.000005)
    servo2 = Servo(name="Servo")
    fail = py_trees.behaviours.Failure(name="Fail")
    # idle2 = py_trees.behaviours.Running(name="Idle2")
    idle = py_trees.behaviours.Running(name="Idle")

    # tree
    root.add_children([topics2bb, priorities])
    topics2bb.add_children([perception_success_2_bb])
    # servo_to_april_tag.add_children([perceive_tag_14, servo1])
    servo_to_april_tag.add_children([servo1])
    priorities.add_children([servo_to_april_tag, idle])
    return root


def shutdown(behaviour_tree):
    behaviour_tree.interrupt()

##############################################################################
# Main
##############################################################################


def main():

    rospy.loginfo("Tree")
    rospy.init_node("tree")
    root = create_root()
    behaviour_tree = py_trees_ros.trees.BehaviourTree(root)
    rospy.on_shutdown(functools.partial(shutdown, behaviour_tree))
    if not behaviour_tree.setup(timeout=15):
        console.logerror("failed to setup the tree, aborting.")
        sys.exit(1)
    behaviour_tree.tick_tock(30) # 10x a second

if __name__ == '__main__':
    main()