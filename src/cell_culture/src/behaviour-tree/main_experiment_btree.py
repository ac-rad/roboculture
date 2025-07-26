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
from dynamic_reconfigure.client import Client

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
    
    # needs_split = py_trees.blackboard.CheckBlackboardVariable(name="NeedsSplit?", variable_name="needs_split", expected_value=True)

    #### 1. Monitor Wells
    stop_shaker = SetExperimentState(name="StopShaker", variable_to_set="shaker_active", value=False)
    start_shaker = SetExperimentState(name="StartShaker", variable_to_set="shaker_active", value=True)
    stop_perception = SetExperimentState(name="StopPerception", variable_to_set="target", value=0)
    stop_perception2 = SetExperimentState(name="StopPerception2", variable_to_set="target", value=0)
    stop_perception3 = SetExperimentState(name="StopPerception2", variable_to_set="target", value=0)

    monitor_wells_seq = py_trees.composites.Sequence("MonitorLoop")

    scheduler = Scheduler(name="Scheduler", duration=300)

    # MESSY!    
    monitor_perceive0 = Perceive(name="MonitorPerceive0", target="plate_cam_well")
    monitor_perceive1 = Perceive(name="MonitorPerceive1", target="plate_cam_well")
    monitor_perceive2 = Perceive(name="MonitorPerceive2", target="plate_cam_well")
    monitor_perceive3 = Perceive(name="MonitorPerceive3", target="plate_cam_well")
    monitor_perceive4 = Perceive(name="MonitorPerceive4", target="plate_cam_well")
    monitor_perceive5 = Perceive(name="MonitorPerceive5", target="plate_cam_well")
    monitor_perceive6 = Perceive(name="MonitorPerceive6", target="plate_cam_well")
    monitor_perceive7 = Perceive(name="MonitorPerceive7", target="plate_cam_well")

    monitor_servo0 = Servo(name="MonitorServo0", z="0.43", kp_z=0.009, kp_img=0.000002, finish_thresh_img=1, sleep=0.1, monitor=True)
    monitor_servo1 = Servo(name="MonitorServo1", z="0.43", kp_z=0.009, kp_img=0.000002, finish_thresh_img=1, sleep=0.1, monitor=True)
    monitor_servo2 = Servo(name="MonitorServo2", z="0.43", kp_z=0.009, kp_img=0.000002, finish_thresh_img=1, sleep=0.1, monitor=True)
    monitor_servo3 = Servo(name="MonitorServo3", z="0.43", kp_z=0.009, kp_img=0.000002, finish_thresh_img=1, sleep=0.1, monitor=True)
    monitor_servo4 = Servo(name="MonitorServo4", z="0.43", kp_z=0.009, kp_img=0.000002, finish_thresh_img=1, sleep=0.1, monitor=True)
    monitor_servo5 = Servo(name="MonitorServo5", z="0.43", kp_z=0.009, kp_img=0.000002, finish_thresh_img=1, sleep=0.1, monitor=True)
    monitor_servo6 = Servo(name="MonitorServo6", z="0.43", kp_z=0.009, kp_img=0.000002, finish_thresh_img=1, sleep=0.1, monitor=True)
    monitor_servo7 = Servo(name="MonitorServo7", z="0.43", kp_z=0.009, kp_img=0.000002, finish_thresh_img=1, sleep=0.1, monitor=True)

    return_to_home_pose = PoseServo(name="PS", x="0.29028134", y="0.10647178", z="0.59348609")

    turn_off_auto_exposure = SetExperimentState(name="TurnOffAutoExposure", variable_to_set="auto_exposure", value=False)
    turn_on_auto_exposure = SetExperimentState(name="TurnOnAutoExposure", variable_to_set="auto_exposure", value=True)


    #### 2. Check Experiment State
    check_needs_split_or_media_or_yeast = py_trees.composites.Selector("CheckNeedsSplitMediaYeast")
    check_needs_split = CheckExperimentState(name="CheckNeedsSplit", variable_to_check="needs_split", desired_value="", invert=True)
    check_needs_media = CheckExperimentState(name="CheckNeedsMedia", variable_to_check="needs_media", desired_value="", invert=True)
    check_needs_yeast = CheckExperimentState(name="CheckNeedsYeast", variable_to_check="needs_yeast", desired_value="", invert=True)
    check_tip_contaminated = CheckExperimentState(name="CheckTipContaminated", variable_to_check="pipette_status", desired_value=4)

    #### 3. Grasp Pipette
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
    ps_return_to_init_pose4 = PoseServo(name="PS", x="0.28012432", y="0.1368218", z="0.5939036")
    ps_return_to_init_pose5 = PoseServo(name="PS", x="0.29012432", y="0.1168218", z="0.5939036")
    ps_return_to_init_pose6 = PoseServo(name="PS", x="0.29012432", y="0.1168218", z="0.5939036")


    #### 4. Attach Pipette Tip
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

    #### 5. Fill Pipette with Media
    servo_to_april_tag_17 = py_trees.composites.Sequence("ServoToAprilTag")
    perceive_tag_17 = Perceive(name="Perceive", target='april_tag', num="17", offset=[48, 88, 0.0])
    servo_tag_17 = Servo(name="ServoTag17", z="0.594", kp_z=0.01)

    fill_pipette_with_media_sel = py_trees.composites.Selector("FillPipetteWithMedia")
    fill_pipette_with_media_seq = py_trees.composites.Sequence("FillPipetteWithMedia")
    check_fill_pipette_with_media_sel = py_trees.composites.Selector("CheckFillPipetteWithMedia")

    check_pipette_is_not_empty = CheckExperimentState(name="check_pipette_is_not_empty", variable_to_check="pipette_status", desired_value=1, invert=True)
    set_pipette_has_media = SetExperimentState(name="set_pipette_has_media", variable_to_set="pipette_status", value=2)
    check_needs_media_empty1 = CheckExperimentState(name="CheckNeedsMedia", variable_to_check="needs_media", desired_value="")

    set_pipette_inhale = SetExperimentState(name="set_pipette_inhale", variable_to_set="pipette_actuator_pos", value="inhale_media")
    set_pipette_remove_air = SetExperimentState(name="set_pipette_remove_air", variable_to_set="pipette_actuator_pos", value=18, relative=True, delay=4)
    set_pipette_remove_air_2 = SetExperimentState(name="set_pipette_remove_air_2", variable_to_set="pipette_actuator_pos", value=9, relative=True, delay=4)

    ps_lower_into_media_container = PoseServo(name="PS", x="~", y="~", z="0.36")
    ps_raise_out_of_media_container = PoseServo(name="PS", x="~", y="~", z="0.59", sleep=4)

    #### 6. Fill needs_media wells
    fill_needs_media_wells_sel = py_trees.composites.Selector("FillNeedsMediaWells") # Selector with commitment
    fill_needs_media_wells_seq = py_trees.composites.Sequence("FillNeedsMediaWells")
    servo_to_well_seq = py_trees.composites.Sequence("ServoToWell")
    perceive_plate = Perceive(name="Plate", target="plate_tip_well", num="needs_media")
    tip_to_well_servo = Servo(name="TipToWellServo", z="0.4915", kp_z=0.007, kp_img=0.000007, finish_thresh_img=1, sleep=0.1)
    check_needs_media_empty2 = CheckExperimentState(name="CheckNeedsMedia", variable_to_check="needs_media", desired_value="")
    check_needs_media_empty3 = CheckExperimentState(name="CheckNeedsMedia", variable_to_check="needs_media", desired_value="")
    insert_into_well = PoseServo(name="InsertIntoWell", x='~', y='~', z='0.482', kp=0.009, sleep=0.1, insertion=True)
    raise_out_of_well = PoseServo(name="RaiseOutOfWell", x='~-0.006', y='~', z='0.506', kp=0.009, sleep=1.5)
    dispense_media = SetExperimentState(name="DispenseMedia", variable_to_set="pipette_actuator_pos", value=dispense_media_len, relative=True)
    check_pipette_has_media = CheckExperimentState(name="check_pipette_has_media", variable_to_check="pipette_status", desired_value=2)
    set_pipette_is_empty = SetExperimentState(name="set_pipette_is_empty", variable_to_set="pipette_status", value=1)
    decrement_needs_media = SetExperimentState(name="decrement_needs_media", variable_to_set="needs_media", value="")

    #### 7. Aspirate Yeast from needs_split wells
    get_yeast_from_split_wells_sel = py_trees.composites.Selector("GetYeastFromSplitWells") # Selector with commitment
    get_yeast_from_split_wells_seq = py_trees.composites.Sequence("GetYeastFromSplitWells")
    check_get_yeast_from_split_sel = py_trees.composites.Selector("CheckGetYeastFromSplit")
    servo_to_well_seq2 = py_trees.composites.Sequence("ServoToWell")
    perceive_plate2 = Perceive(name="Plate", target="plate_tip_well", num="needs_split")
    tip_to_well_servo2 = Servo(name="TipToWellServo", z="0.4915", kp_z=0.007, kp_img=0.000007, finish_thresh_img=1, sleep=0.1)
    check_needs_split_empty = CheckExperimentState(name="CheckNeedsSplitNotEmpty", variable_to_check="needs_split", desired_value="")
    check_needs_yeast_not_empty = CheckExperimentState(name="CheckNeedsYeastNotEmpty", variable_to_check="needs_yeast", desired_value="", invert=True)
    check_pipette_has_yeast_2 = CheckExperimentState(name="check_pipette_has_yeast", variable_to_check="pipette_status", desired_value=3)
    insert_into_well2 = PoseServo(name="InsertIntoWell", x='~', y='~', z='0.4798', kp=0.009, sleep=0.1, insertion=True)
    raise_out_of_well2 = PoseServo(name="RaiseOutOfWell", x='~-0.006', y='~', z='0.506', kp=0.009, sleep=1.5)
    decrement_needs_split = SetExperimentState(name="decrement_needs_split", variable_to_set="needs_split", value="")
    check_pipette_not_contaminated = CheckExperimentState(name="CheckPipetteNotContaminated", variable_to_check="pipette_status", desired_value=4)

    set_pipette_empty = SetExperimentState(name="set_pipette_empty", variable_to_set="pipette_actuator_pos", value=1850)
    set_pipette_resuspend = SetExperimentState(name="set_pipette_resuspend", variable_to_set="pipette_actuator_pos", value="resuspend")
    set_pipette_inhale_yeast = SetExperimentState(name="set_pipette_inhale_yeast", variable_to_set="pipette_actuator_pos", value=1822)
    set_pipette_remove_air_yeast = SetExperimentState(name="set_pipette_remove_air_yeast", variable_to_set="pipette_actuator_pos", value=1841, delay=5)
    set_pipette_has_yeast = SetExperimentState(name="set_pipette_has_yeast", variable_to_set="pipette_status", value=3)

    #### 8. Fill needs_yeast wells with yeast
    fill_needs_yeast_wells_sel = py_trees.composites.Selector("FillNeedsYeastWells") # Selector with commitment
    fill_needs_yeast_wells_seq = py_trees.composites.Sequence("FillNeedsYeastWells")
    servo_to_well_seq3 = py_trees.composites.Sequence("ServoToWell")
    perceive_plate3 = Perceive(name="Plate", target="plate_tip_well", num="needs_yeast")
    tip_to_well_servo3 = Servo(name="TipToWellServo", z="0.4915", kp_z=0.007, kp_img=0.000007, finish_thresh_img=1, sleep=0.1)
    check_needs_yeast_empty_2 = CheckExperimentState(name="CheckNeedsYeast", variable_to_check="needs_yeast", desired_value="")
    check_needs_yeast_empty_3 = CheckExperimentState(name="CheckNeedsYeast", variable_to_check="needs_yeast", desired_value="")
    insert_into_well3 = PoseServo(name="InsertIntoWell", x='~', y='~', z='0.482', kp=0.009, sleep=0.1, insertion=True)
    raise_out_of_well3 = PoseServo(name="RaiseOutOfWell", x='~-0.006', y='~', z='0.506', kp=0.009, sleep=1.5)
    check_pipette_has_yeast = CheckExperimentState(name="check_pipette_has_yeast", variable_to_check="pipette_status", desired_value=3)
    decrement_needs_yeast = SetExperimentState(name="decrement_needs_yeast", variable_to_set="needs_yeast", value="")
    set_pipette_dispense_yeast = SetExperimentState(name="set_pipette_dispense_yeast", variable_to_set="pipette_actuator_pos", value=dispense_yeast_len, relative=True)

    #### 9. Remove pipette tip
    remove_pipette_tip_sel = py_trees.composites.Selector("RemovePipetteTip")
    remove_pipette_tip_seq = py_trees.composites.Sequence("RemovePipetteTip")
    servo_to_april_tag_6 = py_trees.composites.Sequence("ServoToAprilTag")
    # servo_to_april_tag_3_2 = py_trees.composites.Sequence("ServoToAprilTag") 
    perceive_tag_6 = Perceive(name="Perceive", target='april_tag', num="6", offset=[20, -224.0, 0.0])
    # perceive_tag_3_2 = Perceive(name="Perceive", target='april_tag', num="3", offset=[-18.0, 300, 0.0]) 
    servo_tag_6 = Servo(name="ServoTag6")
    # servo_tag_3_2 = Servo(name="ServoTag6")
    ps_lower_to_remover = PoseServo(name="PS", x="~", y="~", z="0.415")
    ps_into_remover = PoseServo(name="PS", x="~0.055", y="~", z="~", kp=0.006)
    ps_lift_and_detach = PoseServo(name="PS", x="~", y="~", z="~0.15", kp=0.006)
    pipette_is_contaminated = CheckExperimentState(name="PipetteIsContaminated", variable_to_check="pipette_status", desired_value=4, invert=True)


    # tree
    root.add_children([topics2bb, tasks])
    topics2bb.add_children([perception_success_2_bb])

    # Global selector (all tasks)
    tasks.add_children([start_shaker, scheduler, stop_shaker, turn_off_auto_exposure, monitor_wells_seq, return_to_home_pose, turn_on_auto_exposure, stop_perception, check_needs_split_or_media_or_yeast, grasp_pipette_sel, attach_pipette_tip_sel, fill_pipette_with_media_sel, fill_needs_media_wells_sel, get_yeast_from_split_wells_sel, fill_needs_yeast_wells_sel, remove_pipette_tip_sel])
    # tasks.add_children([check_needs_split_or_media_or_yeast, grasp_pipette_sel, attach_pipette_tip_sel, fill_pipette_with_media_sel, fill_needs_media_wells_sel, get_yeast_from_split_wells_sel, fill_needs_yeast_wells_sel, remove_pipette_tip_sel])

    ## 1. Monitor Wells
    # monitor_wells_sel.add_children([monitor_wells_seq, monitor_complete])
    monitor_wells_seq.add_children([monitor_perceive0, monitor_servo0, monitor_perceive1, monitor_servo1, monitor_perceive2, monitor_servo2, monitor_perceive3, monitor_servo3, monitor_perceive4, monitor_servo4, monitor_perceive5, monitor_servo5, monitor_perceive6, monitor_servo6, monitor_perceive7, monitor_servo7])

    ## 2. Check is any wells need splitting or need media
    check_needs_split_or_media_or_yeast.add_children([check_needs_split, check_needs_media, check_needs_yeast, check_tip_contaminated])

    ## 3. Grasp Pipette
    servo_to_april_tag_14.add_children([perceive_tag_14, servo_tag_14])
    grasp_pipette_sel.add_children([holding_pipette, grasp_pipette_seq])
    grasp_pipette_seq.add_children([servo_to_april_tag_14, ps_approach_pipette, ps_grasp_pipette, close_gripper, ps_raise_pipette, ps_return_to_init_pose, set_holding_pipette])

    ## 4. Attach Pipette Tip
    servo_to_april_tag_4.add_children([perceive_tag_4, servo_tag_4])
    attach_pipette_tip_seq.add_children([check_tip_rack_has_tips, ps_return_to_init_pose6, servo_to_april_tag_4, perceive_tag_4_offset, servo_tag_4_offset, spiral, ps_lower_into_tip, ps_raise_out_of_tip, ps_return_to_init_pose2, set_pipette_has_tip, set_pipette_expel, decrement_tip_rack_count])
    attach_pipette_tip_sel.add_children([pipette_has_tip, attach_pipette_tip_seq])

    ## 5. Fill Pipette with Media 
    servo_to_april_tag_17.add_children([perceive_tag_17, servo_tag_17])
    fill_pipette_with_media_seq.add_children([servo_to_april_tag_17, ps_lower_into_media_container, set_pipette_inhale, set_pipette_remove_air, set_pipette_remove_air_2, ps_raise_out_of_media_container, ps_return_to_init_pose3, set_pipette_has_media, stop_perception2])
    fill_pipette_with_media_sel.add_children([check_fill_pipette_with_media_sel, fill_pipette_with_media_seq])
    check_fill_pipette_with_media_sel.add_children([check_pipette_is_not_empty, check_needs_media_empty1])

    ## 6. Fill needs_media wells
    servo_to_well_seq.add_children([perceive_plate, tip_to_well_servo])
    fill_needs_media_wells_seq.add_children([check_pipette_has_media, servo_to_well_seq, insert_into_well, dispense_media, raise_out_of_well, decrement_needs_media, check_needs_media_empty3])
    fill_needs_media_wells_sel.add_children([check_needs_media_empty2, fill_needs_media_wells_seq])

    ## 7. Aspirate Yeast from needs_split wells
    servo_to_well_seq2.add_children([perceive_plate2, tip_to_well_servo2])
    get_yeast_from_split_wells_seq.add_children([set_pipette_empty, servo_to_well_seq2, insert_into_well2, set_pipette_resuspend, set_pipette_inhale_yeast, set_pipette_remove_air_yeast, raise_out_of_well2, decrement_needs_split, set_pipette_has_yeast])
    get_yeast_from_split_wells_sel.add_children([check_get_yeast_from_split_sel, get_yeast_from_split_wells_seq])
    check_get_yeast_from_split_sel.add_children([check_needs_split_empty, check_needs_yeast_not_empty, check_pipette_has_yeast_2, check_pipette_not_contaminated])

    ## 8. Fill needs_yeast wells with yeast
    servo_to_well_seq3.add_children([perceive_plate3, tip_to_well_servo3])
    fill_needs_yeast_wells_seq.add_children([check_pipette_has_yeast, servo_to_well_seq3, insert_into_well3, set_pipette_dispense_yeast, raise_out_of_well3, decrement_needs_yeast, check_needs_yeast_empty_3])
    fill_needs_yeast_wells_sel.add_children([check_needs_yeast_empty_2, fill_needs_yeast_wells_seq])

    ## 9. Remove pipette tip
    servo_to_april_tag_6.add_children([perceive_tag_6, servo_tag_6])
    # servo_to_april_tag_3_2.add_children([perceive_tag_3_2, servo_tag_3_2])
    remove_pipette_tip_seq.add_children([ps_return_to_init_pose4, servo_to_april_tag_6, ps_lower_to_remover, ps_into_remover, ps_lift_and_detach, ps_return_to_init_pose5, set_pipette_has_no_tip])
    remove_pipette_tip_sel.add_children([pipette_is_contaminated, remove_pipette_tip_seq])


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
