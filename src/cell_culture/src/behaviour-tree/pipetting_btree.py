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
from cell_culture.src.behaviour.servo import Servo
from cell_culture.src.behaviour.close_gripper import CloseGripper
from cell_culture.src.behaviour.pose_servo import PoseServo
from cell_culture.src.behaviour.perceive import Perceive
from cell_culture.src.behaviour.set_experiment_state import SetExperimentState

class ToBlackboard(py_trees_ros.subscribers.ToBlackboard):

    def __init__(self, name, topic_name, topic_type, blackboard_variables):
        super(ToBlackboard, self).__init__(name=name,
                                           topic_name=topic_name,
                                           topic_type=topic_type,
                                           blackboard_variables=blackboard_variables,
                                           clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE
                                           )
        self.blackboard = py_trees.blackboard.Blackboard()
        self.blackboard.perception_success = Bool()
        self.blackboard.holding_pipette = Bool()
        self.blackboard.pipette_has_tip = Bool()
        self.blackboard.pipette_has_media = Bool()
        self.blackboard.tip_coords_found = Bool()


    def update(self):
        """
        Call the parent to write the raw data to the blackboard
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
    root = py_trees.composites.Parallel("GraspPipette (root)")
    topics2bb = py_trees.composites.Sequence("Topics2BB")
    perception_success_2_bb = ToBlackboard(name="PerceptionSuccess2BB", topic_name="/cell_culture/perception/success", topic_type=Bool, blackboard_variables={'perception_success': None})

    sequence = py_trees.composites.Sequence("Sequence")

    tip_coords_found = py_trees.blackboard.CheckBlackboardVariable(name="TipCoordsFound", variable_name="tip_coords_found", expected_value=True)
    set_tip_coords_found = py_trees.blackboard.SetBlackboardVariable(name="SetTipCoordsFound", variable_name="tip_coords_found", variable_value=True)

    perception_success = py_trees.blackboard.CheckBlackboardVariable(name="PerceptionSuccess", variable_name="perception_success", expected_value=True)

    segment_tip_sequence = py_trees.composites.Sequence("SegmentTipSequence")
    servo_to_well_sequence = py_trees.composites.Sequence("ServoToWellSequence")

    segment_tip_selector = py_trees.composites.Selector("SegmentTipSelector")
    segment_plate_selector = py_trees.composites.Selector("SegmentPlateSelector")
    # perception_success_selector = py_trees.composites.Selector("PerceptionSuccessSelector")
    servo_to_well = py_trees.composites.Sequence("ServoToWell")

    # segment_tip = Perceive(name="SegmentTip", target="tip")
    perceive_plate = Perceive(name="Plate", target="plate_tip_well")
    tip_to_well_servo = Servo(name="TipToWellServo", z="0.497", kp_z=0.006, kp_img=0.000008, finish_thresh_img=0.0000000000001, sleep=0.1)
    # tip_to_well_servo = Servo(name="TipToWellServo", z="~", kp_z=0.006, kp_img=0.000002, finish_thresh_img=1, sleep=0.1)
    insert_into_well = PoseServo(name="InsertIntoWell", x='~', y='~', z='0.482', kp=0.009, sleep=0.1)
    # insert_into_well = PoseServo(name="InsertIntoWell", x='~', y='~', z='~-0.015', kp=0.005, sleep=1)
    raise_out_of_well = PoseServo(name="RaiseOutOfWell", x='~-0.005', y='~', z='0.5', kp=0.009, sleep=1)

    increment_well = SetExperimentState(name="IncrementWell", variable_to_set="desired_well", value="increment")
    
    idle1 = py_trees.behaviours.Running(name="Idle")
    idle2 = py_trees.behaviours.Running(name="Idle")
    idle3 = py_trees.behaviours.Running(name="Idle")
    idle4 = py_trees.behaviours.Running(name="Idle")

    # tree
    root.add_children([topics2bb, sequence])
    topics2bb.add_children([perception_success_2_bb])
    sequence.add_children([segment_plate_selector])

    # segment_tip_selector.add_children([tip_coords_found, segment_tip_sequence, idle1])
    # segment_tip_sequence.add_children([segment_tip, set_tip_coords_found])

    segment_plate_selector.add_children([servo_to_well_sequence, idle2])
    servo_to_well.add_children([perceive_plate, tip_to_well_servo])
    servo_to_well_sequence.add_children([servo_to_well, insert_into_well, raise_out_of_well, increment_well])

    return root


def shutdown(behaviour_tree):
    behaviour_tree.interrupt()


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