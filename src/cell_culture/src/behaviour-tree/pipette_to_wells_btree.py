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
from behaviour.monitor import Monitor
from behaviour.scheduler import Scheduler
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
        ## TODO needed?


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

    dispense_media_len = 9
    dispense_yeast_len = 3 # TODO make param
    

    #### 1. Grasp Pipette
    grasp_pipette_sel = py_trees.composites.Selector("GraspPipette")
    grasp_pipette_seq = py_trees.composites.Sequence("GraspPipette")
    servo_to_april_tag_14 = py_trees.composites.Sequence("ServoToAprilTag")

    holding_pipette = CheckExperimentState(name="check_holding_pipette", variable_to_check="holding_pipette", desired_value=True)
    set_holding_pipette = SetExperimentState(name="set_holding_pipette", variable_to_set="holding_pipette", value=True)
    
    perceive_tag_14 = Perceive(name="PerceiveTag3", target="april_tag", num="14", offset=[-108, 252.0, 0.0])
    servo_tag_14 = Servo(name="ServoTag14")

    ps_approach_pipette = PoseServo(name="PS", x="~", y="~", z="0.372")
    ps_grasp_pipette = PoseServo(name="PS", x="~0.069", y="~0.04", z="~") 
    ps_raise_pipette = PoseServo(name="PS", x="~", y="~", z="~0.16")
    close_gripper = CloseGripper(name="CloseGripper")
    ps_return_to_init_pose = PoseServo(name="PS", x="0.29026111", y="0.10688011", z="0.59382127")
      
    stop_perception = SetExperimentState(name="StopPerception", variable_to_set="target", value=0)

    #### 2. Insert into wells
    fill_needs_media_wells_sel = py_trees.composites.Selector("FillNeedsMediaWells") # Selector with commitment
    fill_needs_media_wells_seq = py_trees.composites.Sequence("FillNeedsMediaWells")
    servo_to_well_seq = py_trees.composites.Sequence("ServoToWell")
    perceive_plate = Perceive(name="Plate", target="plate_tip_well", num="needs_media")
    tip_to_well_servo = Servo(name="TipToWellServo", z="0.4915", kp_z=0.007, kp_img=0.000007, finish_thresh_img=1, sleep=0.1)
    check_needs_media_empty2 = CheckExperimentState(name="CheckNeedsMedia", variable_to_check="needs_media", desired_value="")
    check_needs_media_empty3 = CheckExperimentState(name="CheckNeedsMedia", variable_to_check="needs_media", desired_value="")
    insert_into_well = PoseServo(name="InsertIntoWell", x='~', y='~', z='0.482', kp=0.009, sleep=0.1, insertion=True)
    raise_out_of_well = PoseServo(name="RaiseOutOfWell", x='~-0.006', y='~', z='0.506', kp=0.009, sleep=0.5)
    decrement_needs_media = SetExperimentState(name="decrement_needs_media", variable_to_set="needs_media", value="")

    #0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95

    # tree
    root.add_children([topics2bb, tasks])
    topics2bb.add_children([perception_success_2_bb])

    # Global sequence (all tasks)
    # tasks.add_children([fill_needs_media_wells_sel])
    tasks.add_children([grasp_pipette_sel, fill_needs_media_wells_sel])

    ## 1. Grasp Pipette
    servo_to_april_tag_14.add_children([perceive_tag_14, servo_tag_14])
    grasp_pipette_sel.add_children([holding_pipette, grasp_pipette_seq])
    grasp_pipette_seq.add_children([servo_to_april_tag_14, ps_approach_pipette, ps_grasp_pipette, close_gripper, ps_raise_pipette, ps_return_to_init_pose, set_holding_pipette, stop_perception])

    ## 2. Fill needs_media wells
    servo_to_well_seq.add_children([perceive_plate, tip_to_well_servo])
    fill_needs_media_wells_seq.add_children([servo_to_well_seq, insert_into_well, raise_out_of_well, decrement_needs_media, check_needs_media_empty3])
    fill_needs_media_wells_sel.add_children([check_needs_media_empty2, fill_needs_media_wells_seq])

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
