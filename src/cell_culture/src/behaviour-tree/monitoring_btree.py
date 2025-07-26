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
        # self.blackboard.error_vector = Vector3()
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
    tasks = py_trees.composites.Sequence("Tasks")
    perception_success_2_bb = ToBlackboard(name="PerceptionSuccess2BB", topic_name="/cell_culture/perception/success", topic_type=Bool, blackboard_variables={'perception_success': None})

    #### 1. Monitor Wells
    stop_shaker = SetExperimentState(name="StopShaker", variable_to_set="shaker_active", value=False)
    start_shaker = SetExperimentState(name="StartShaker", variable_to_set="shaker_active", value=True)
    stop_perception = SetExperimentState(name="StopPerception", variable_to_set="target", value=0)

    monitor_wells_seq = py_trees.composites.Sequence("MonitorLoop")

    scheduler = Scheduler(name="Scheduler", duration=120)

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

    root.add_children([topics2bb, tasks])
    topics2bb.add_children([perception_success_2_bb])

    # Global selector (all tasks)
    tasks.add_children([start_shaker, scheduler, stop_shaker, turn_off_auto_exposure, monitor_wells_seq, return_to_home_pose, turn_on_auto_exposure, stop_perception])
    monitor_wells_seq.add_children([monitor_perceive0, monitor_servo0, monitor_perceive1, monitor_servo1, monitor_perceive2, monitor_servo2, monitor_perceive3, monitor_servo3, monitor_perceive4, monitor_servo4, monitor_perceive5, monitor_servo5, monitor_perceive6, monitor_servo6, monitor_perceive7, monitor_servo7])

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
