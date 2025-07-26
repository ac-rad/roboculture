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
from tip_rack_status import TipRackStatus
from cell_culture.src.behaviour.check_experiment_state import CheckExperimentState
from cell_culture.src.behaviour.set_experiment_state import SetExperimentState

class ToBlackboard(py_trees_ros.subscribers.ToBlackboard):
    """
    When ticking, updates with :attr:`~py_trees.common.Status.RUNNING` if it got no data,
    :attr:`~py_trees.common.Status.SUCCESS` otherwise.

    Blackboard Variables:
        - tag_found (:obj:`bool`): True if an april tag is found in the current image
        - holding_pipette (:obj:`bool`): True if the robot is holding the pipette
        - pipette_has_tip (:obj:`bool`): True if the pipette has a tip attached
        - pipette_has_media (:obj:`bool`): True if the pipette has media


    Args:
        name (:obj:`str`): name of the behaviour
        topic_name (:obj:`str`) : name of the ROS topic to listen to
        topic_type (:obj:`msg`) : type of the ROS message
        blackboard_variables (:obj:`dict`) : dictionary of blackboard variables to write to
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
        self.blackboard.force_low = Bool()
        self.blackboard.holding_pipette = Bool()
        self.blackboard.pipette_has_tip = Bool()
        self.blackboard.pipette_has_media = Bool()


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

            if type(self.blackboard.force_low) is not bool:
                self.blackboard.force_low = self.blackboard.force_low.data

        return status


def create_root():

    # High level behaviours
    root = py_trees.composites.Parallel("GraspPipette (root)")
    topics2bb = py_trees.composites.Sequence("Topics2BB")
    # tag_found_2_bb = ToBlackboard(name="TagFound2BB", topic_name="/april_tag/found", topic_type=Bool, blackboard_variables={'tag_found': None})
    perception_success_2_bb = ToBlackboard(name="PerceptionSuccess2BB", topic_name="/cell_culture/perception/success", topic_type=Bool, blackboard_variables={'perception_success': None})
    ee_force_2_bb = ToBlackboard(name="EEForce2BB", topic_name="/cell_culture/control/ee_force", topic_type=Bool, blackboard_variables={'force_low': None})

    # Selectors
    grasp_pipette_selector = py_trees.composites.Selector("GraspPipette")
    attach_pipette_tip_selector = py_trees.composites.Selector("AttachPipetteTip")
    fill_media_selector = py_trees.composites.Selector("FillMedia")
    remove_pipette_tip_selector = py_trees.composites.Selector("RemovePipetteTip")
    pipette_well_selector = py_trees.composites.Selector("PipetteWell")

    # Sequences
    sequence = py_trees.composites.Sequence("Sequence")
    grasp_pipette_sequence = py_trees.composites.Sequence("GraspPipette")
    attach_pipette_tip_sequence = py_trees.composites.Sequence("AttachPipetteTip")
    fill_media_sequence = py_trees.composites.Sequence("FillMedia")
    remove_pipette_tip_sequence = py_trees.composites.Sequence("RemovePipetteTip")
    servo_to_april_tag1 = py_trees.composites.Sequence("ServoToAprilTag")
    servo_to_april_tag2 = py_trees.composites.Sequence("ServoToAprilTag")
    servo_to_april_tag3 = py_trees.composites.Sequence("ServoToAprilTag")

    # Blackboard variables
    # holding_pipette = py_trees.blackboard.CheckBlackboardVariable(name="HoldingPipette?", variable_name="holding_pipette", expected_value=True)
    holding_pipette = CheckExperimentState(name="check_holding_pipette", variable_to_check="holding_pipette", desired_value=True)
    # set_holding_pipette = py_trees.blackboard.SetBlackboardVariable(name="holding_pipette=True", variable_name="holding_pipette", variable_value=True)
    set_holding_pipette = SetExperimentState(name="set_holding_pipette", variable_to_set="holding_pipette", value=True)

    # pipette_has_tip = py_trees.blackboard.CheckBlackboardVariable(name="PipetteHasTip?", variable_name="pipette_has_tip", expected_value=True)
    # set_pipette_has_tip = py_trees.blackboard.SetBlackboardVariable(name="pipette_has_tip=True", variable_name="pipette_has_tip", variable_value=True)
    pipette_has_tip = CheckExperimentState(name="check_pipette_has_tip", variable_to_check="pipette_has_tip", desired_value=True)
    set_pipette_has_tip = SetExperimentState(name="set_pipette_has_tip", variable_to_set="pipette_has_tip", value=True)

    # pipette_has_no_tip = py_trees.blackboard.CheckBlackboardVariable(name="PipetteHasNoTip?", variable_name="pipette_has_tip", expected_value=False)
    # set_pipette_has_no_tip = py_trees.blackboard.SetBlackboardVariable(name="pipette_has_tip=False", variable_name="pipette_has_tip", variable_value=False)
    pipette_has_no_tip = CheckExperimentState(name="check_pipette_has_tip", variable_to_check="pipette_has_tip", desired_value=False)
    set_pipette_has_no_tip = SetExperimentState(name="set_pipette_has_tip", variable_to_set="pipette_has_tip", value=False)

    check_tip_rack_has_tips = CheckExperimentState(name="check_tip_rack_has_tips", variable_to_check="tip_rack_count", desired_value=0, operator=">", invert=False)
    decrement_tip_rack_count = SetExperimentState(name="decrement_tip_rack_count", variable_to_set="tip_rack_count", value="decrement")

    pipette_has_media = py_trees.blackboard.CheckBlackboardVariable(name="PipetteHasMedia?", variable_name="pipette_has_media", expected_value=True)
    set_pipette_has_media = py_trees.blackboard.SetBlackboardVariable(name="pipette_has_media=True", variable_name="pipette_has_media", variable_value=True)

    is_force_low = py_trees.blackboard.CheckBlackboardVariable(name="IsForceLow?", variable_name="force_low", expected_value=True)

    # Perception behaviours
    perceive_tag_3 = Perceive(name="Perceive", target='april_tag', num="3", offset=[48, 108.0, 0.0])
    perceive_tag_4 = Perceive(name="Perceive", target='april_tag', num="4", offset=[64.0, -104.0, 0.0])
    perceive_tag_6 = Perceive(name="Perceive", target='april_tag', num="6", offset=[60.0, -130.0, 0.0])
    perceive_tag_4_offset = Perceive(name="Perceive", target='april_tag', num="4", use_tip_offsets=True)

    # Robot control behaviours
    servo1 = Servo(name="Servo")
    servo2 = Servo(name="Servo", z="0.40", kp_z=0.01)
    servo3 = Servo(name="Servo")
    # servo_to_tip = Servo(name="Servo", z="0.360", kp_z=0.004, kp_img=0.000005, sleep=0)
    servo_to_tip = Servo(name="Servo", z="0.36", kp_z=0.004, kp_img=0.000005, sleep=0)

    close_gripper = CloseGripper(name="CloseGripper")

    ps_approach_pipette = PoseServo(name="PS", x="~", y="~", z="0.368") # add 0.055 for custom gripper config
    ps_grasp_pipette = PoseServo(name="PS", x="~0.042", y="~", z="~") # Relative: move 0.05m in x direction
    ps_raise_pipette = PoseServo(name="PS", x="~", y="~", z="~0.16")
    
    ps_lower_into_beaker = PoseServo(name="PS", x="~", y="~", z="~-0.28")
    ps_raise_out_of_beaker = PoseServo(name="PS", x="~", y="~", z="~0.28", sleep=1)

    ps_return_to_init_pose = PoseServo(name="PS", x="0.24", y="0.066", z="0.594")
    ps_return_to_init_pose2 = PoseServo(name="PS", x="0.24", y="0.066", z="0.594")
    ps_return_to_init_pose3 = PoseServo(name="PS", x="0.24", y="0.066", z="0.594")
    # ps_return_to_init_pose2 = PoseServo(name="PS", x="0.1902", y="0.1163", z="0.5737")

    spiral = Servo(name="Spiral", spiral=True)
    ps_lower_into_tip = PoseServo(name="PS", x="~", y="~", z="~-0.029", kp=0.0037)
    ps_raise_out_of_tip = PoseServo(name="PS", x="~", y="~", z="~0.25")

    # save_image = Perceive(name="SaveImage", target="april_tag", num="save_image")

    ps_lower_to_remover = PoseServo(name="PS", x="~", y="~", z="0.415")
    ps_into_remover = PoseServo(name="PS", x="~0.07", y="~", z="~")
    ps_lift_and_detach = PoseServo(name="PS", x="~", y="~", z="~0.15")

    # tip_rack_status = TipRackStatus(name="TipRackStatus")


    # Idle
    idle1 = py_trees.behaviours.Running(name="Idle")
    idle2 = py_trees.behaviours.Running(name="Idle")
    idle3 = py_trees.behaviours.Running(name="Idle")
    idle4 = py_trees.behaviours.Running(name="Idle")
    fail = py_trees.behaviours.Failure(name="Fail")

    ### Construct tree ###
    root.add_children([topics2bb, sequence])
    topics2bb.add_children([perception_success_2_bb, ee_force_2_bb])
    sequence.add_children([grasp_pipette_selector, attach_pipette_tip_selector, remove_pipette_tip_selector])
    # sequence.add_children([grasp_pipette_selector, attach_pipette_tip_selector])

    servo_to_april_tag1.add_children([perceive_tag_3, servo1])
    servo_to_april_tag2.add_children([perceive_tag_4, servo2])
    servo_to_april_tag3.add_children([perceive_tag_6, servo3])

    grasp_pipette_sequence.add_children([servo_to_april_tag1, ps_approach_pipette, ps_grasp_pipette, close_gripper, ps_raise_pipette, ps_return_to_init_pose, set_holding_pipette])
    grasp_pipette_selector.add_children([holding_pipette, grasp_pipette_sequence])

    # attach_pipette_tip_sequence.add_children([tip_rack_status, servo_to_april_tag2, ps_pipette_attach_height, perceive_tip, servo_to_tip, spiral, ps_lower_into_tip, ps_raise_out_of_tip, set_pipette_has_tip])
    attach_pipette_tip_sequence.add_children([check_tip_rack_has_tips, servo_to_april_tag2, perceive_tag_4_offset, servo_to_tip, spiral, ps_lower_into_tip, ps_raise_out_of_tip, ps_return_to_init_pose2, set_pipette_has_tip, decrement_tip_rack_count])
    attach_pipette_tip_selector.add_children([pipette_has_tip, attach_pipette_tip_sequence])

    remove_pipette_tip_sequence.add_children([servo_to_april_tag3, ps_lower_to_remover, ps_into_remover, ps_lift_and_detach, ps_return_to_init_pose3, set_pipette_has_no_tip])
    remove_pipette_tip_selector.add_children([pipette_has_no_tip, remove_pipette_tip_sequence])

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