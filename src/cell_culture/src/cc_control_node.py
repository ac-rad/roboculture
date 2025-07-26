#! /usr/bin/env python

import rospy
import numpy as np
import collections
import actionlib
from cell_culture.msg import ControlEnableAction, ControlEnableResult, ControlEnableFeedback
import tf

from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool

from frankapy import FrankaArm, SensorDataMessageType
from franka_interface_msgs.msg import SensorDataGroup, RobotState
from frankapy import FrankaConstants as FC
from frankapy.proto import PosePositionSensorMessage, CartesianImpedanceSensorMessage
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg

class ControlAction(object):

    def __init__(self, name):
        self.init_time = rospy.Time.now().to_time()

        self.dt = 0.01
        self.rate = rospy.Rate(1 / self.dt)

        self.error_vec_sub = rospy.Subscriber('/cell_culture/perception/error_vector', Vector3, self.error_vec_cb)
        self.ee_force_sub = rospy.Subscriber('/robot_state_publisher_node_1/robot_state', RobotState, self.ee_force_cb)
        self.ee_force_status_pub = rospy.Publisher('/cell_culture/control/ee_force', Bool, queue_size=10)
        self.cmd_pub = rospy.Publisher(FC.DEFAULT_SENSOR_PUBLISHER_TOPIC, SensorDataGroup, queue_size=1000)

        self.latest_error_vector = None
        self.loop_id = 0
        self.ee_force = None
        self.init_ee_force = None
        self.prev_ee_force_values = []
        self.currently_servoing = False
        self.in_tilt_pose = False
        self.rotational_stiff_z = 50

        self.config = { # TODO: Use ROS params
            "translational_stiffnesses": [3000.0, 3000.0, 3000.0],
            # "rotational_stiffnesses": [100.0,100.0,30.0],
            "rotational_stiffnesses": [100.0,100.0,50.0],
            "gripper_type": "Franka",
            # "kp_img": 0.00001,
            "kp_pose": 0.007,
            # "ut_mag_cap": 0.0005,
            "ut_mag_cap": 0.0004,
            "finish_threshold_img": 1,
            "finish_threshold_img_monitor": 10,
            "finish_threshold_pose": 0.002,
            "inital_pose": [0.1, -0.0, 0.06],
        }

        # Go to initial pose
        self.start_pose = fa.get_pose()
        self.start_pose.translation += np.array(self.config['inital_pose'])
        fa.goto_pose(self.start_pose, duration=3, dynamic = False, use_impedance=False)
        self.init_pose = fa.get_pose()
        fa.goto_pose(self.init_pose, buffer_time=100000, dynamic = True) # turn on dynamic mode if not already on
        
        # Initialize ros action server
        self.action_name = name
        self.action_server = actionlib.SimpleActionServer(self.action_name, ControlEnableAction, execute_cb=self.execute_cb, auto_start = False)
        self.action_server.start()

        self.tf_listener = tf.TransformListener()
        self.tf_listener.waitForTransform('/panda_link0', '/panda_end_effector', rospy.Time(), rospy.Duration(4.0))
        
        rospy.sleep(1)

        rospy.loginfo("[Control Node] Init pose: %s" % self.init_pose)
        rospy.loginfo("[Control Node] Initialized Control node")

    def error_vec_cb(self, data):
        self.latest_error_vector = np.array([data.x, data.y])

    def ee_force_cb(self, data):
        self.ee_force = data.O_F_ext_hat_K

        # keep track of previous few force values for force spike detection
        if len(self.prev_ee_force_values) < 50:
            self.prev_ee_force_values.append(self.ee_force)
        else:
            self.prev_ee_force_values.pop(0)
            self.prev_ee_force_values.append(self.ee_force)

    def get_end_effector_pose(self):
        try:            
            (trans, rot) = self.tf_listener.lookupTransform('/panda_link0', '/panda_end_effector', rospy.Time(0))
            return trans, rot
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(e)
            return None, None
      
    def execute_cb(self, goal):
        # This function is called when a new goal is received by the action server - goals represent types of actions for the robot to perform 

        result = ControlEnableResult()
        feedback = ControlEnableFeedback()

        rospy.loginfo("[Control Node] Received goal command: %s" % goal.command)

        if goal.command == 'stop':
            rospy.loginfo("[Control Node] Received stop command.")
            self.action_server.set_aborted(result, "Received stop command.")
            return
        
        # Pose servo: move robot to a desired cartesian pose (absolute or relative)
        elif goal.command.startswith('pose_servo'):
            x, y, z, kp, insertion = goal.command.split()[1:]
            rospy.loginfo("[Control Node] Received pose servo command: x=%s, y=%s, z=%s, kp=%s, insertion=%s" % (x, y, z, kp, insertion))
            self.currently_servoing = True

            X1 = np.zeros(3)
            curr_pose = fa.get_pose()
            for i, elem in enumerate([x, y, z]):
                if elem.startswith('~'):
                    # relative movement: use ~ to indicate offset to current pose. eg. ~0.1 will move 0.1m IN the positive direction. Just ~ will not move in that axis.
                    if len(elem) > 1:
                        val = float(elem[1:])
                    else:
                        val = 0.0
                    elem = curr_pose.translation[i] + val
                else:
                    # absolute movement: no ~ indicates absolute position, eg. 0.1 will move TO 0.1m in that axis.
                    elem = float(elem)

                X1[i] = elem

            feedback.status = 'Moving robot to desired pose...'
            self.action_server.publish_feedback(feedback)

            # Robot control loop; continue to send poses dynamically until error vector is below threshold
            error_vector_mag = 1
            while error_vector_mag > self.config['finish_threshold_pose']:
                if self.action_server.is_preempt_requested():
                    rospy.loginfo("[Control Node] Action Server: Preempted")
                    self.action_server.set_preempted()
                    return
                
                # Use force feedback to preempt if force spikes (hit well plate edge with pipette) during insertion
                if insertion == 'True':
                    if self.ee_force[4] > -0.5:
                        rospy.loginfo("[Control Node] Force spike detected; insertion failed. Preempting.")
                        self.action_server.set_preempted()
                        return
                
                error_vector = np.subtract(X1, curr_pose.translation)
                error_vector_mag = np.linalg.norm(error_vector)

                # compute control signal with kp gain
                u_t = np.dot(float(kp), error_vector)
                u_t_mag = np.linalg.norm(u_t)

                if u_t_mag > self.config['ut_mag_cap']:
                    # scale u_t so its magnitude is self.config['ut_mag_cap']
                    u_t = np.multiply(u_t, self.config['ut_mag_cap'] / u_t_mag)
                
                curr_pose.translation = np.add(curr_pose.translation, u_t)
                curr_pose.rotation = self.init_pose.rotation

                self.send_dynamic_pose(curr_pose)

                feedback.status = 'Moving...'
                self.action_server.publish_feedback(feedback)

                self.rate.sleep()

            rospy.loginfo("[Control Node] Robot has reached desired pose.")
            result.success = True
            self.action_server.set_succeeded(result, "Robot reached desired pose.")
            self.currently_servoing = False
                
            return
        
        # Servo: move robot according to a perception error vector, with a target z coordinate
        elif goal.command.startswith('servo'):
        
            z, kp_z, kp_img, finish_thresh_img, rotational_stiff_z = goal.command.split()[1:]
            rospy.loginfo("[Control Node] Received servo command: z=%s, kp_z=%s, kp_img=%s, finish_thresh_img=%s" % (z, kp_z, kp_img, finish_thresh_img))
            self.currently_servoing = True
            self.rotational_stiff_z = float(rotational_stiff_z)

            curr_pose = fa.get_pose()

            if z.startswith('~'): # relative movement
                if len(z) > 1:
                    error_z = float(z[1:])
                else:
                    error_z = 0.0
            else: # absolute movement
                error_z = float(z) - curr_pose.translation[2]
            error_mag_z = np.abs(error_z)

            # Using deque with max length of 10 to keep track of the last 10 error vector magnitudes
            # To prevent perception flickers from causing the robot to stop prematurely 
            last_10_error_vec_mags = collections.deque(maxlen=10)
            count_vec_below_thresh = 0

            error_vector_mag = 10
            # Control loop: continue to send poses dynamically until error vector is below threshold
            while True:
                if self.action_server.is_preempt_requested():
                    rospy.loginfo("[Control Node] Preempted")
                    self.action_server.set_preempted()
                    return
                
                if self.latest_error_vector is None:
                    rospy.loginfo("[Control Node] No error vector received.")
                    return

                error_vector_mag = np.linalg.norm(self.latest_error_vector)
                error_vec_below_thresh = error_vector_mag <= float(finish_thresh_img)

                if len(last_10_error_vec_mags) == 10:
                    oldest_error_vec_mag = last_10_error_vec_mags.popleft()
                    if oldest_error_vec_mag <= float(finish_thresh_img):
                        count_vec_below_thresh -= 1
                last_10_error_vec_mags.append(error_vector_mag)
                if error_vec_below_thresh:
                    count_vec_below_thresh += 1

                # Compute control signal with kp gains
                u_t = np.dot(float(kp_img), -1 * self.latest_error_vector)
                u_t = np.append(u_t, float(kp_z) * error_z)
                u_t_mag = np.linalg.norm(u_t)
                # Rotate u_t to match the robot's coordinate system
                u_t = np.array([u_t[1], u_t[0], u_t[2]])


                if u_t_mag > self.config['ut_mag_cap']:
                    # scale u_t so its magnitude is self.ut_mag_cap
                    u_t = np.multiply(u_t, self.config['ut_mag_cap'] / u_t_mag)
                    
                error_z = error_z - u_t[2]
                error_mag_z = np.abs(error_z)

                curr_pose.translation = np.add(curr_pose.translation, u_t)
                curr_pose.rotation = self.init_pose.rotation
                # print(fa.get_pose().rotation)

                self.send_dynamic_pose(curr_pose)

                # provide feedback
                feedback.status = 'Servoing...'
                self.action_server.publish_feedback(feedback)

                self.rate.sleep()

                # Check if the error vector magnitude is below its threshold for the last 10 timesteps
                # If so, robot has reached the target
                if count_vec_below_thresh == 10 and error_mag_z <= self.config['finish_threshold_pose']:
                    break

            end_pose = fa.get_pose()

            rospy.loginfo("[Control Node] Robot has reached target.")
            result.success = True
            self.currently_servoing = False
            self.action_server.set_succeeded(result, "Robot reached target.")

        # Spiral: move robot in a spiral pattern to perform tip attachment with force feedback
        elif goal.command == 'spiral':
            rospy.loginfo("[Control Node] Received spiral command.")
            feedback.status = 'Performing spiral motion...'
            self.action_server.publish_feedback(feedback)

            self.prev_ee_force_values = []

            points = self.generate_spiral_points()
            curr_pose = fa.get_pose()
            start_spiral_time = rospy.Time.now().to_time()

            # Open a file to log the spiral force data # TODO: make this a parameter
            with open('spiral_force.csv', 'w') as f:
                # write header
                f.write('time, force, pose_x, pose_y, pose_z\n')

                for i,point in enumerate(points):
                    pose = curr_pose.copy()
                    pose.translation[0] += point[0]
                    pose.translation[1] += point[1]
                    pose.translation[2] = 0.362
                    pose.rotation = self.init_pose.rotation
                    self.send_dynamic_pose(pose)

                    # write force and pose to file
                    f.write(f'{rospy.Time.now().to_time()},{self.ee_force[2]},{pose.translation[0]},{pose.translation[1]},{pose.translation[2]}\n')

                    if self.init_ee_force is None:
                        self.init_ee_force = self.ee_force
                        rospy.loginfo("[Control Node] Initial EE force [0]: %s" % self.init_ee_force[0])
                        rospy.loginfo("[Control Node] Initial EE force [1]: %s" % self.init_ee_force[1])
                        rospy.loginfo("[Control Node] Initial EE force [3]: %s" % self.init_ee_force[3])

                    # check difference between current and force value from 8 iterations ago
                    force_threshold_down = 1.5
                    if len(self.prev_ee_force_values) == 50:
                        # Downward force drops when insertion is successful
                        if self.ee_force[2] > self.prev_ee_force_values[-8][2] + force_threshold_down and self.ee_force[2] > 0:
                            rospy.loginfo("[Control Node] Down Force increase detected. Stopping spiral.")
                            rospy.loginfo("[Control Node] Force difference was: %s" % (self.ee_force[2] - self.prev_ee_force_values[-8][2]))
                            break

                        # If z height of end effector is below a certain threshold, we assume we are already inside the tip and stop the spiral
                        ee_z_height = self.get_end_effector_pose()[0][2]
                        if ee_z_height < 0.361: # TODO: make this a parameter
                            rospy.loginfo("[Control Node] Already inside tip. Stopping spiral.")
                            break

                    self.rate.sleep()

                else:  
                    # executed when the loop is not broken
                    # meaning, we have not inserted the pipette correctly. 
                    rospy.loginfo("[Control Node] Error: Spiral search did not result in successful insertion, setting status to failed.")
                    result.success = False
                    self.action_server.set_preempted(result, "Spiral search did not result in successful insertion.")
                    return

            end_sprial_time = rospy.Time.now().to_time()
            rospy.loginfo("[Control Node] Spiral search took %s seconds." % (end_sprial_time - start_spiral_time))
            rospy.loginfo("[Control Node] Spiral search completed at index %s." % i)
            result.success = True
            self.action_server.set_succeeded(result, "Robot reached target.")

            self.init_ee_force = None

       
    def generate_spiral_points(self, num_points=2800, segment_length=0.0001, radius_increment=0.0008, scaling=1):
        points = []
        radius = 0.0002  # starting radius
        total_points_generated = 0

        while total_points_generated < num_points:
            # Calculate the number of points in this circle, trying to keep segment length roughly constant
            circumference = 2 * np.pi * radius
            num_points_in_circle = int(circumference / segment_length)
            
            if num_points_in_circle < 1:
                num_points_in_circle = 1  # Ensure at least one point per circle
            
            for i in range(num_points_in_circle):
                angle = (2 * np.pi * i) / num_points_in_circle
                x = radius * np.cos(angle) * scaling  # Scaling factor for x-axis
                y = radius * np.sin(angle) / scaling  # Scaling factor for y-axis
                points.append((x, y))
                total_points_generated += 1
                if total_points_generated >= num_points:
                    break

            radius += radius_increment  # Increase the radius for the next circle

        return points
    
    def send_dynamic_pose(self, ee_pose):
        # Send a dynamic pose (non blocking) to the robot's end effector
        # Using frankapy api
        timestamp = rospy.Time.now().to_time() - self.init_time

        rot_stiff = self.config['rotational_stiffnesses']
        rot_stiff[2] = self.rotational_stiff_z

        trajectory_generator_proto_msg = PosePositionSensorMessage(
            id=self.loop_id, 
            timestamp=timestamp,
            position=ee_pose.translation, 
            quaternion=ee_pose.quaternion
        )
        feedback_controler_proto = CartesianImpedanceSensorMessage(
                id=self.loop_id, timestamp=timestamp,
                translational_stiffnesses=self.config['translational_stiffnesses'],
                rotational_stiffnesses=self.config['rotational_stiffnesses']
        )
        ros_msg = make_sensor_group_msg(
            trajectory_generator_sensor_msg=sensor_proto2ros_msg(trajectory_generator_proto_msg, SensorDataMessageType.POSE_POSITION),
            feedback_controller_sensor_msg=sensor_proto2ros_msg(feedback_controler_proto, SensorDataMessageType.CARTESIAN_IMPEDANCE)
        )
        
        ros_msg.header.stamp = rospy.Time.now()   
        self.cmd_pub.publish(ros_msg)
        self.loop_id += 1
        self.prev_time = rospy.Time.now().to_time()


if __name__ == '__main__':
    fa = FrankaArm(with_gripper=False, rosnode_name='control_node')
    fa.reset_joints()
    control = ControlAction('control_node')
    rospy.spin()