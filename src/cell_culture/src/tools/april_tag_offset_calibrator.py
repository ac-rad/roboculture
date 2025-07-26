#!/usr/bin/env python
import rospy
import actionlib
from cell_culture.msg import PerceptionEnableAction, PerceptionEnableResult, PerceptionEnableFeedback, PerceptionEnableGoal
from cell_culture.msg import ControlEnableAction, ControlEnableResult, ControlEnableFeedback, ControlEnableGoal

# Use this script to calibrate the pixel offsets for AprilTags

class ATCalibrator:

    def __init__(self):
        self.at_client = actionlib.SimpleActionClient('cc_perception_node', PerceptionEnableAction)
        self.at_client.wait_for_server()
        self.servo_client = actionlib.SimpleActionClient('control_node', ControlEnableAction)
        self.servo_client.wait_for_server()

        rospy.loginfo('[ATCalibrator] Action clients initialized')

        # self.telop_sub = rospy.Subscriber('/cell_culture/teleop/commands', String, self.offset_cb)

        self.x_offset = 0.0
        self.y_offset = 0.0
        # self.z_offset = 0.0
        self.tag = input("Enter AprilTag ID: ")

        self.perception_ready = False

        self.send_perception_goal(self.tag)
        self.send_control_goal()
   

    def parse_offset(self, offset):
        try:
            tgt = offset[0]
            operator = offset[1]
            val = float(offset[2:])
            assert operator in ["=","+"] and tgt in ["x","y","z","t"], f"Invalid offset command. Format: (tgt)(operator)(val)"
            
            if tgt == 'x':
                if operator == '=':
                    self.x_offset = val
                elif operator == '+':
                    self.x_offset += val
            elif tgt == 'y':
                if operator == '=':
                    self.y_offset = val
                elif operator == '+':
                    self.y_offset += val
            elif tgt == 'z':
                self.send_pose_servo_goal(z=float(offset[2:]))
                return
            elif tgt == 't':
                self.tag = offset[2:]
                rospy.loginfo(f"AprilTag changed to {self.tag}")

            self.at_client.cancel_all_goals()
            self.send_perception_goal(self.tag)
            self.send_control_goal()   

            rospy.loginfo(f"AprilTag offset: x={self.x_offset}, y={self.y_offset}")

        except:
            rospy.loginfo("Invalid offset command")

    def send_perception_goal(self, id):
        goal = PerceptionEnableGoal()
        goal.target = 'april_tag'
        goal.num = str(id)
        goal.x_offset = self.x_offset
        goal.y_offset = self.y_offset
        goal.z_offset = 0
        self.at_client.send_goal(goal)

    def send_control_goal(self):
        action_state = self.at_client.get_state()
        while action_state != actionlib.GoalStatus.SUCCEEDED:
            rospy.sleep(0.1)
            action_state = self.at_client.get_state()

        goal = ControlEnableGoal()
        goal.command = f'servo ~ 0.002 0.00001 0 0.0001'
        self.servo_client.send_goal(goal)

        rospy.loginfo(f"AprilTag offset: x={self.x_offset}, y={self.y_offset}")

    def send_pose_servo_goal(self, x='~', y='~', z='~'):
        goal = ControlEnableGoal()
        goal.command = f'pose_servo {x} {y} ~{z} 0.009'
        self.servo_client.send_goal(goal)

if __name__ == '__main__':
    rospy.init_node('at_calibrator')
    at_calibrator = ATCalibrator()


    msg = '''Input offset command.
        Example: x=4 adds 4 pixels to AprilTag position in positive x direction.
        Likewise, y=-2 adds 2 pixels to AprilTag position in negative y direction.
        z=0.001 adds 0.001m to the z position of the robot.
        '''

    while not rospy.is_shutdown():
        print(msg)
        offset = input()
        at_calibrator.parse_offset(offset)
            