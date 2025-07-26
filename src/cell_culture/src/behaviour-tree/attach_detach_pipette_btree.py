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
from behaviour.close_gripper import CloseGripper
from behaviour.pose_servo import PoseServo
from behaviour.perceive import Perceive
from behaviour.check_experiment_state import CheckExperimentState
from behaviour.set_experiment_state import SetExperimentState

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
    root = py_trees.composites.Parallel("(root)")
    topics2bb = py_trees.composites.Sequence("Topics2BB")
    perception_success_2_bb = ToBlackboard(name="PerceptionSuccess2BB", topic_name="/cell_culture/perception/success", topic_type=Bool, blackboard_variables={'perception_success': None})
    ee_force_2_bb = ToBlackboard(name="EEForce2BB", topic_name="/cell_culture/control/ee_force", topic_type=Bool, blackboard_variables={'force_low': None})
    
    tasks = py_trees.composites.Sequence("Tasks")
    
    #### 1. Grasp Pipette
    grasp_pipette_sel = py_trees.composites.Selector("GraspPipette")
    grasp_pipette_seq = py_trees.composites.Sequence("GraspPipette")
    servo_to_april_tag_14 = py_trees.composites.Sequence("ServoToAprilTag")

    holding_pipette = CheckExperimentState(name="check_holding_pipette", variable_to_check="holding_pipette", desired_value=True)
    set_holding_pipette = SetExperimentState(name="set_holding_pipette", variable_to_set="holding_pipette", value=True)
    
    perceive_tag_14 = Perceive(name="PerceiveTag3", target="april_tag", num="14", offset=[-108, 252.0, 0.0])
    servo_tag_14 = Servo(name="ServoTag14")

    ps_approach_pipette = PoseServo(name="PS", x="~", y="~", z="0.372")
    ps_grasp_pipette = PoseServo(name="PS", x="~0.073", y="~0.04", z="~")
    ps_raise_pipette = PoseServo(name="PS", x="~", y="~", z="~0.16")
    close_gripper = CloseGripper(name="CloseGripper")
    ps_return_to_init_pose = PoseServo(name="PS", x="0.29012432", y="0.1168218", z="0.5939036")
    ps_return_to_init_pose1 = PoseServo(name="PS", x="0.29012432", y="0.1168218", z="0.5939036")
    ps_return_to_init_pose2 = PoseServo(name="PS", x="0.29012432", y="0.1168218", z="0.5939036")
    ps_return_to_init_pose3 = PoseServo(name="PS", x="0.29012432", y="0.1168218", z="0.5939036")
    ps_return_to_init_pose4 = PoseServo(name="PS", x="0.28012432", y="0.1068218", z="0.5939036")
    ps_return_to_init_pose5 = PoseServo(name="PS", x="0.29012432", y="0.1168218", z="0.5939036")


    #### 2. Attach Pipette Tip
    attach_pipette_tip_sel = py_trees.composites.Selector("AttachPipetteTip")
    attach_pipette_tip_seq = py_trees.composites.Sequence("AttachPipetteTip")
    servo_to_april_tag_4 = py_trees.composites.Sequence("ServoToAprilTag")

    pipette_has_tip = CheckExperimentState(name="check_pipette_has_tip", variable_to_check="pipette_status", desired_value=0, invert=True)
    set_pipette_has_tip = SetExperimentState(name="set_pipette_has_tip", variable_to_set="pipette_status", value=1)
    set_pipette_has_no_tip = SetExperimentState(name="set_pipette_has_no_tip", variable_to_set="pipette_status", value=0)
    check_tip_rack_has_tips = CheckExperimentState(name="check_tip_rack_has_tips", variable_to_check="tip_rack_count", desired_value=0, operator=">", invert=False)
    decrement_tip_rack_count = SetExperimentState(name="decrement_tip_rack_count", variable_to_set="tip_rack_count", value="decrement")

    spiral = Servo(name="Spiral", spiral=True)

    perceive_tag_4 = Perceive(name="Perceive", target='april_tag', num="4", offset=[8.0, -130.0, 0.0])
    perceive_tag_4_offset = Perceive(name="Perceive", target='april_tag', num="4", use_tip_offsets=True)

    servo_tag_4 = Servo(name="Servo", z="0.385", kp_z=0.01)
    servo_tag_4_offset = Servo(name="Servo", z="0.357", kp_z=0.006, kp_img=0.000005, sleep=0)

    ps_lower_into_tip = PoseServo(name="PS", x="~", y="~", z="~-0.029", kp=0.0037)
    ps_raise_out_of_tip = PoseServo(name="PS", x="~", y="~", z="~0.25")
    set_pipette_expel = SetExperimentState(name="set_pipette_expel", variable_to_set="pipette_actuator_pos", value=1850)

    # perceive_tip = Perceive(name="Perceive", target='tip', num="")

    #### 3. Remove pipette tip
    remove_pipette_tip_sel = py_trees.composites.Selector("RemovePipetteTip")
    remove_pipette_tip_seq = py_trees.composites.Sequence("RemovePipetteTip")
    servo_to_april_tag_6 = py_trees.composites.Sequence("ServoToAprilTag")
    servo_to_april_tag_3_2 = py_trees.composites.Sequence("ServoToAprilTag") 
    perceive_tag_6 = Perceive(name="Perceive", target='april_tag', num="6", offset=[20, -224.0, 0.0])
    # perceive_tag_3_2 = Perceive(name="Perceive", target='april_tag', num="3", offset=[-18.0, 300, 0.0]) 
    servo_tag_6 = Servo(name="ServoTag6")
    # servo_tag_3_2 = Servo(name="ServoTag6")
    ps_lower_to_remover = PoseServo(name="PS", x="~", y="~", z="0.415")
    ps_into_remover = PoseServo(name="PS", x="~0.055", y="~", z="~", kp=0.006)
    ps_lift_and_detach = PoseServo(name="PS", x="~", y="~", z="~0.15", kp=0.006)
    # use tag on tip remover (3) to make final alignement since it is fixed. 
    # use tag on waste bin (6) to make initial alignment since tag on tip remover will be blocked by pipette tip

    # tree
    root.add_children([topics2bb, tasks])
    topics2bb.add_children([perception_success_2_bb])

    # Global selector (all tasks)
    tasks.add_children([grasp_pipette_sel, attach_pipette_tip_sel, remove_pipette_tip_sel])

    ## 1. Grasp Pipette
    servo_to_april_tag_14.add_children([perceive_tag_14, servo_tag_14])
    grasp_pipette_sel.add_children([holding_pipette, grasp_pipette_seq])
    grasp_pipette_seq.add_children([servo_to_april_tag_14, ps_approach_pipette, ps_grasp_pipette, close_gripper, ps_raise_pipette, ps_return_to_init_pose, set_holding_pipette])

    ## 2. Attach Pipette Tip
    servo_to_april_tag_4.add_children([perceive_tag_4, servo_tag_4])
    attach_pipette_tip_seq.add_children([check_tip_rack_has_tips, servo_to_april_tag_4, perceive_tag_4_offset, servo_tag_4_offset, spiral, ps_lower_into_tip, ps_raise_out_of_tip, ps_return_to_init_pose2, set_pipette_has_tip, set_pipette_expel, decrement_tip_rack_count])
    attach_pipette_tip_sel.add_children([pipette_has_tip, attach_pipette_tip_seq])

    ## 3. Remove pipette tip
    servo_to_april_tag_6.add_children([perceive_tag_6, servo_tag_6])
    # servo_to_april_tag_3_2.add_children([perceive_tag_3_2, servo_tag_3_2])
    remove_pipette_tip_seq.add_children([ps_return_to_init_pose4, servo_to_april_tag_6, ps_lower_to_remover, ps_into_remover, ps_lift_and_detach, ps_return_to_init_pose5, set_pipette_has_no_tip])
    remove_pipette_tip_sel.add_children([remove_pipette_tip_seq])


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
