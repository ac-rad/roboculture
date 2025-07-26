#!/usr/bin/env python
import numpy as np
import rospy
import cv2
import torch
import time
import apriltag
from scipy.spatial.distance import cdist
from skimage.filters import threshold_triangle

from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool

from fastsam import FastSAM, FastSAMPrompt
# from process_well_image import process_well_image
from helpers.process_well_image import ProcessWellImage
from cell_culture.msg import PlateGrowthMsg

from cv_bridge import CvBridge, CvBridgeError
import actionlib
from dynamic_reconfigure.server import Server
from dynamic_reconfigure.client import Client

from cell_culture.msg import PerceptionEnableAction, PerceptionEnableResult, PerceptionEnableFeedback, PerceptionEnableGoal
from cell_culture.msg import ImagingAction, ImagingResult, ImagingFeedback, ImagingGoal
from cell_culture.cfg import CellCultureGlobalConfig, CellCulturePerceptionConfig

from dynamic_reconfigure.encoding import decode_config, encode_config, encode_description, extract_params, get_tree, initial_config
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo

class CCPerception:

    def __init__(self, name):

        rospy.init_node('cc_perception_node')

        self.init_time = time.time()

        self.dt = 0.01
        self.rate = rospy.Rate(1 / self.dt)

        # FastSAM options
        self.model = FastSAM("weights/FastSAM-x.pt")
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu") # TODO: use torch instead of cpu

        # AprilTag options
        self.at_options = apriltag.DetectorOptions(families="tag36h11", quad_decimate=1)
        self.at_detector = apriltag.Detector(self.at_options)
        self.tag_size = 0.127

        with np.load('src/camera_cal_imgs/calibration.npz') as cal: # TODO move calibration param file
            # Extract fx, fy, cx, cy
            self.fx = cal['mtx'][0,0]
            self.fy = cal['mtx'][1,1]
            self.cx = cal['mtx'][0,2]
            self.cy = cal['mtx'][1,2]

        self.config = { ## TODO: move to config file
            "sam_img_size": 1280,
            "sam_conf": 0.9,
            "sam_iou": 0.9,
            "plate_prompt": "microplate", # well plate
            "plate_length": 127.89,
            "plate_width": 85.50,
            "plate_dist": [9.1, 19.2, 38.67], # Distance between centers of wells in each well plate [96, 24, 6] (mm)
            "plate_left_offset": [14.3, 14.7, 24.8], # Distance from left edge to center of first column in each well plate [96, 24, 6] (mm)
            "plate_up_offset": [11.36, 14.1, 22.32],
        }

        # Initialize OpenCV bridge
        self.bridge = CvBridge()

        # Variables
        self.num = "-" ## TODO: remove dash
        self.perception_active = False
        self.callback_active = False

        self.latest_frame = None

        self.prompt_points = [[822, 100], [801, 157]]
        self.tip_coordinates = None
        self.resetting = False
        self.found_plate = False
        self.corners = None
        self.previous_angles = None
        self.pipette_mask = None
        self.ann = None
        self.shifts = 0
        self.mouseX = 0
        self.mouseY = 0
        self.requires_image = None
        self.monitor_idx = 0
        self.monitor_iteration = 0
        self.last_reset_time = time.time()

        # Subscribers and publishers
        self.camera_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.camera_info_callback)
        self.perception_img_pub = rospy.Publisher('/cell_culture/perception/image', Image, queue_size=10)
        self.perception_success_pub = rospy.Publisher('/cell_culture/perception/success', Bool, queue_size=10)
        self.error_vector_pub = rospy.Publisher('/cell_culture/perception/error_vector', Vector3, queue_size=10)
        self.plate_growth_pub = rospy.Publisher('/cell_culture/plate_growth', PlateGrowthMsg, queue_size=10)
        
        # Debugging publishers
        self.seg_image_pub = rospy.Publisher('/cell_culture/perception/seg_image', Image, queue_size=10)
        self.canny_pub = rospy.Publisher('/cell_culture/perception/canny_image', Image, queue_size=10)
        self.bbox_image_pub = rospy.Publisher('/cell_culture/perception/bbox_image', Image, queue_size=10)
        self.annotated_image_pub = rospy.Publisher('/cell_culture/perception/annotated_image', Image, queue_size=10)
        self.preprocess_pub = rospy.Publisher('/cell_culture/perception/preprocess_image', Image, queue_size=10)

        # Dynamic Reconfigure parameters
        self.target = None
        self.plate_type = 96
        self.desired_well = 0
        self.april_tag_id = 0
        self.plate_length_offset = 0
        self.plate_width_offset = 0
        self.plate_left_correction = 0
        self.plate_up_correction = 0
        self.x_tag_offset = 0.0
        self.y_tag_offset = 0.0
        self.z_tag_offset = 0.0
        self.using_homography = False
        self.monitoring_idx = 0

        # Action server
        self.action_name = name
        self.action_server = actionlib.SimpleActionServer(self.action_name, PerceptionEnableAction, execute_cb=self.execute_cb, auto_start = False)
        self.action_server.start()

        self.imaging_action_server = actionlib.SimpleActionServer('imaging', ImagingAction, execute_cb=self.imaging_execute_cb, auto_start = False)
        self.imaging_action_server.start()

        self.target_mapping = {
            0: "None",
            1: "plate_tip_well",
            2: "plate_cam_well",
            3: "plate_cam_center",
            4: "april_tag",
        }
        self.inv_target_mapping = {v: k for k, v in self.target_mapping.items()}

        self.plate_type_mapping = {
            0: 96,
            1: 24,
            2: 6
        }

        self.well_to_offsets = {
            # used for monitoring
            # plate_len, plate_width, plate_left, plate_up
            13: [8.0, 5.2, 0.4, 0.8],
            61: [5.6, 3.8, 0.2, 1.2],
            16: [6.4, 5.8, 1.0, 0.8],
            64: [5.6, 2.6, 0.8, 0.4],
            19: [7.0, 4.8, 3.6, 0.6],
            67: [5.2, 3.8, 2.8, 1.8],
            22: [5.6, 3.8, 5.2, 0.2],
            70: [5.6, 3.2, 5.2, 0.8],
        }

        self.column_to_offsets = {
            # used for tip insertion into plate
            # plate_len, plate_width, plate_left, plate_up
            0: [3.4, 4.4, 0.8, 0.0], 
            1: [3.4, 4.4, 0.8, 0.0],
            2: [3.4, 4.4, 0.8, 0.4],
            3: [3.4, 4.4, 0.8, 0.4],
            4: [1.4, 3.8, 0.0, 0.4],
            5: [1.4, 3.8, 0.0, 0.4],
            6: [1.4, 3.8, 0.0, 0.4],
            7: [1.4, 3.8, 0.0, -0.2],
            8: [0.2, 2.8, 0.0, -0.2],
            9: [0.2, 2.8, 0.0, -0.2],
            10: [0.2, 2.8, 0.0, -0.2],
            11: [0.2, 2.8, 0.0, -0.2],
        }

        self.target_to_image_wells = [
            # 13
            [0 ,1 ,2 ,
             12,13,14,
             24,25,26,
             36,37,38],

            # 61
            [48,49,50,
             60,61,62,
             72,73,74,
             84,85,86],

            # 16
            [3 ,4 ,5 ,
             15,16,17,
             27,28,29,
             39,40,41],

            # 64
            [51,52,53,
             63,64,65,
             75,76,77,
             87,88,89],

            # 19
            [6 ,7 ,8 ,
             18,19,20,
             30,31,32,
             42,43,44],

            # 67
            [54,55,56,
             66,67,68,
             78,79,80,
             90,91,92],

            # 22
            [9 ,10,11,
             21,22,23,
             33,34,35,
             45,46,47],

            # 70
            [57,58,59,
             69,70,71,
             81,82,83,
             93,94,95],
        ]

        self.targets_to_indices = {
            13: 0,
            61: 1,
            16: 2,
            64: 3,
            19: 4,
            67: 5,
            22: 6,
            70: 7,
        }

        self.inv_targets_to_indices = {v: k for k, v in self.targets_to_indices.items()}

        self.inv_target_to_image_wells = {}
        for i, sublist in enumerate(self.target_to_image_wells):
            for item in sublist:
                self.inv_target_to_image_wells[item] = list(self.well_to_offsets.keys())[i]

        self.process_well_image = ProcessWellImage()
        self.plate_growth_msg = PlateGrowthMsg()

        self.perception_reconfigure_server = Server(CellCulturePerceptionConfig, self.perception_reconfigure_callback)
        self.global_reconfigure_client = Client("cell_culture_global_reconfigure", timeout=3, config_callback=self.global_reconfigure_callback)
        self.global_reconfigure_client.update_configuration({"monitoring_idx": 0})

        while self.callback_active is False:
            rospy.loginfo_once("[Perception Node] Waiting for image callback...")
            self.rate.sleep()
        rospy.loginfo("[Perception Node] Initialized Perception Node")

        self.latest_camera_info = None
        self.camera_model = PinholeCameraModel()

    def camera_info_callback(self, msg):
        self.latest_camera_info = msg

    def perception_reconfigure_callback(self, config, level):
        rospy.loginfo("""[Perception Node] Perception Reconfigure Request: {target}, {plate_type}, {april_tag_id}, {desired_well}, {plate_length_offset}, {plate_width_offset}, {plate_left_correction}, {plate_up_correction}, {x_tag_offset}, {y_tag_offset}, {z_tag_offset}, {using_homography}""".format(**config))

        self.target = self.target_mapping[config["target"]]
        self.plate_type = self.plate_type_mapping[config["plate_type"]]
        self.april_tag_id = int(config["april_tag_id"])
        self.desired_well = int(config["desired_well"])
        self.plate_length_offset = config["plate_length_offset"]
        self.plate_width_offset = config["plate_width_offset"]
        self.plate_left_correction = config["plate_left_correction"]
        self.plate_up_correction = config["plate_up_correction"]
        self.x_tag_offset = config["x_tag_offset"]
        self.y_tag_offset = config["y_tag_offset"]
        self.z_tag_offset = config["z_tag_offset"]
        self.using_homography = config["using_homography"]

        return config
    
    def global_reconfigure_callback(self, config):
        self.monitoring_idx = config["monitoring_idx"]

    def execute_cb(self, goal):
        self.perception_active = False

        # When transitioning from april_tag to plate, publish false on perception_success_pub to stop perception
        if self.target is None or goal.target is None or (self.target.startswith('april_tag') and goal.target.startswith('plate')):
            for _ in range(20):
                # publish empty error vector
                error_vector = Vector3()
                error_vector.x = 0
                error_vector.y = 0
                error_vector.z = 0
                self.error_vector_pub.publish(error_vector)

                bool_msg = Bool()
                bool_msg.data = False
                self.perception_success_pub.publish(bool_msg)
                self.rate.sleep()

        self.target = goal.target
        self.num = goal.num
        self.x_tag_offset = goal.x_offset
        self.y_tag_offset = goal.y_offset
        self.z_tag_offset = goal.z_offset
        result = PerceptionEnableResult()
        feedback = PerceptionEnableFeedback()
        feedback.status = 'Perception starting...'
        self.action_server.publish_feedback(feedback)
        rospy.loginfo('[Perception Node] Perception starting...')
        rospy.loginfo('[Perception Node] Target: %s, Num: %s', self.target, self.num)

        current_target = self.target

        # Update dynamic reconfigure parameters from incoming goal
        if current_target == "plate_cam_well":
            # Use well_to_offsets for plate_cam_well
            self.desired_well = self.inv_targets_to_indices[int(self.monitoring_idx)]
            new_config = {
                "desired_well": self.desired_well,
                "target": self.inv_target_mapping[current_target],
                "plate_length_offset": self.well_to_offsets[self.inv_target_to_image_wells[int(self.desired_well)]][0],
                "plate_width_offset": self.well_to_offsets[self.inv_target_to_image_wells[int(self.desired_well)]][1],
                "plate_left_correction": self.well_to_offsets[self.inv_target_to_image_wells[int(self.desired_well)]][2],
                "plate_up_correction": self.well_to_offsets[self.inv_target_to_image_wells[int(self.desired_well)]][3],
                }
            self.perception_reconfigure_server.update_configuration(new_config)
            if self.num not in ["-", ""]: # TODO: make less ugly
                self.perception_reconfigure_server.update_configuration({"desired_well": int(self.num)})
        if current_target == "plate_tip_well":
            # Use column_to_offsets for tip_to_well
            if self.num not in ["-", ""]:
                self.perception_reconfigure_server.update_configuration({"desired_well": int(self.num)})
            
            new_config = {
                # "desired_well": int(self.num), 
                "target": self.inv_target_mapping[current_target],
                "plate_length_offset": self.column_to_offsets[int(self.desired_well) % 12][0],
                "plate_width_offset": self.column_to_offsets[int(self.desired_well) % 12][1],
                "plate_left_correction": self.column_to_offsets[int(self.desired_well) % 12][2],
                "plate_up_correction": self.column_to_offsets[int(self.desired_well) % 12][3],
                }
            self.perception_reconfigure_server.update_configuration(new_config)
        elif current_target.startswith("april_tag"):
            new_config = {
                "april_tag_id": int(self.num),
                "target": self.inv_target_mapping[current_target],
                "x_tag_offset": self.x_tag_offset,
                "y_tag_offset": self.y_tag_offset,
                "z_tag_offset": self.z_tag_offset,
            }
            self.perception_reconfigure_server.update_configuration(new_config)

        # return success when perception starts, i.e. when we start publishing true on self.perception_success_pub
        while not self.perception_active:
            self.rate.sleep()

        # perception has started - receiving perception_success = true
        # return success so Perceive behaviour can return success
        result.success = True
        self.action_server.set_succeeded(result, "Perception started")
        rospy.loginfo('[Perception Node] Perception started')
        return
    
    def imaging_execute_cb(self, goal):
        result = ImagingResult()
        rospy.loginfo('[Perception Node] Imaging, goal: %d', goal.image)

        self.requires_image = goal.image

        result.success = True
        self.imaging_action_server.set_succeeded(result, "Imaging started")

        return
    
    @staticmethod
    def draw_april_tag_frame(overlay, camera_params, tag_size, pose):

        axis_length=0.5*tag_size
        # Define the 3D points of the axes in relation to the center of the AprilTag
        # X axis (blue), Y axis (green), Z axis (red)
        objpoints = np.array([
            [0, 0, 0],  # Center
            [axis_length, 0, 0],  # X-axis endpoint
            [0, axis_length, 0],  # Y-axis endpoint
            [0, 0, -axis_length]  # Z-axis endpoint
        ]).reshape(-1, 1, 3)

        # camera matrix, distortion coefficients
        fx, fy, cx, cy = camera_params
        K = np.array([fx, 0, cx, 0, fy, cy, 0, 0, 1]).reshape(3, 3)
        dcoeffs = np.zeros(5)
        
        # convert rotation matrix to rotation vector
        rvec, _ = cv2.Rodrigues(pose[:3, :3])
        tvec = pose[:3, 3]

        # 3D->2D projection
        ipoints, _ = cv2.projectPoints(objpoints, rvec, tvec, K, dcoeffs)
        ipoints = np.round(ipoints).astype(int)
        ipoints = [tuple(pt) for pt in ipoints.reshape(-1, 2)]

        cv2.line(overlay, ipoints[0], ipoints[1], (255, 0, 0), 2)  # X-axis (blue)
        cv2.line(overlay, ipoints[0], ipoints[2], (0, 255, 0), 2)  # Y-axis (green)
        cv2.line(overlay, ipoints[0], ipoints[3], (0, 0, 255), 2)  # Z-axis (red)
    

    def get_prompt_points(self, img):
        """
        If prompt points are not set, prompt user with gui to select them
        """
        if self.prompt_points == []:
            def mouse_click_pp(event, x, y, flags, param):
                if event == cv2.EVENT_LBUTTONDOWN:
                    self.mouseX, self.mouseY = x, y

            rospy.loginfo("[Perception Node] Prompt points not set, prompting user to select them...")

            cv2.namedWindow("Select Pipette Prompt Points")
            cv2.setMouseCallback("Select Pipette Prompt Points", mouse_click_pp)

            prompt_points = []
            cv2.imshow("Select Pipette Prompt Points", img)
            cv2.waitKey(0)
            prompt_points.append([self.mouseX, self.mouseY])
                            
            cv2.circle(img, prompt_points[0], 1,(255,0,0), 1)

            cv2.imshow("Select Pipette Prompt Points", img)

            cv2.waitKey(0)
            prompt_points.append([self.mouseX, self.mouseY])

            cv2.circle(img, prompt_points[1], 1,(255,0,0), 1)

            cv2.imshow("Select Pipette Prompt Points", img)
            cv2.waitKey(0)
            # close window named "select prompt points"
            cv2.destroyWindow("Select Pipette Prompt Points")

            return prompt_points
        
        else:
            return self.prompt_points
        
    @staticmethod
    def convert_coordinates(x, y, from_size=(1280, 720), to_size=(1920, 1280)):
        # Calculate the scaling factors
        x_scale = to_size[0] / from_size[0]
        y_scale = to_size[1] / from_size[1]

        # Convert the coordinates
        new_x = x * x_scale
        new_y = y * y_scale

        return new_x, new_y
        

    def segment_tip(self, frame, points):
        """
        Use FastSAM to segment the tip of the pipette given the prompt points, and return the pipette tip's coordinates
        """
        rospy.loginfo("[Perception Node] Segmenting tip")
        rospy.loginfo(f"[Perception Node] Prompt Points: {points}")

        everything_results = self.model(
            source=frame, 
            device=self.device, 
            retina_masks=True, 
            imgsz=self.config["sam_img_size"],
            conf=self.config["sam_conf"],
            iou=self.config["sam_iou"]
        )
        prompt_process = FastSAMPrompt(frame, everything_results, device=self.device)

        ann = prompt_process.point_prompt(points=points, pointlabel=[1,1])
        
        binary_mask_uint8 = (ann[0] * 255).astype(np.uint8)
        rospy.logdebug(f"[Perception Node] Shape of segment tip mask: {binary_mask_uint8.shape}")

        # Extract lowest y-coord
        y_coords = np.where(ann[0])[0]
        if y_coords is None or len(y_coords) == 0: 
            return None
        lowest_y = y_coords.max()
        x_coords = np.where(ann[0][lowest_y])[0] 
        all_tip_coordinates = list(zip(x_coords, [lowest_y] * len(x_coords)))
        tip_coordinates = np.mean(all_tip_coordinates, axis=0)
        seg_mask = cv2.dilate(binary_mask_uint8, np.ones((20,20), np.uint8), iterations=1) # Dilate to ensure tip's edges are fully removed for well plate detection
        # seg_mask = binary_mask_uint8  # Dont dilate for visualization
        
        return tip_coordinates, seg_mask, ann[0]
    
    def get_box_corners(self, preprocessed, bridge):

        img = preprocessed.copy()

        # Segment the well plate if not already found
        # if self.initial_segment and not self.found_plate:
        if not self.found_plate:

            perception_success_msg = Bool()
            perception_success_msg.data = False
            self.perception_success_pub.publish(perception_success_msg)

            rospy.loginfo("[Perception Node] Segmenting well plate...")
            everything_results = self.model(
                source=img, 
                device=self.device, 
                retina_masks=True, 
                imgsz=1024, 
                conf=0.9, # 0.8 
                iou=0.9, # 0.8
            )
            prompt_process = FastSAMPrompt(img, everything_results, device=self.device)
            ann = prompt_process.text_prompt(text=self.config["plate_prompt"])
            # seg_img = prompt_process.plot_to_result(img, annotations=ann)
            # self.seg_image_pub.publish(bridge.cv2_to_imgmsg(seg_img, "bgr8"))
            binary_mask_uint8 = (ann[0] * 255).astype(np.uint8)
            self.seg_image_pub.publish(bridge.cv2_to_imgmsg(binary_mask_uint8, "mono8"))
            rospy.loginfo(f"[Perception Node] Shape of initial plate segmentation mask: {binary_mask_uint8.shape}")
            rospy.loginfo(f"[Perception Node] Tip coords are: {self.tip_coordinates}")
            self.last_reset_time = time.time()

        # Edge detection
        canny_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # canny_img = cv2.GaussianBlur(canny_img, (11, 11), 0)
        
        v = int(np.mean(canny_img))
        sigma = 0.33
        lower_thresh = int(max(0, (1.0 - sigma) * v))
        upper_thresh = int(min(255, (1.0 + sigma) * v))

        edges = cv2.Canny(canny_img, threshold1=lower_thresh, threshold2=upper_thresh, apertureSize = 3) 

        self.canny_pub.publish(bridge.cv2_to_imgmsg(edges, "mono8"))

        if not self.found_plate:
            # Mask the canny edges using the well plate mask
            # Expand well mask by 5 pixels
            binary_mask_uint8 = cv2.dilate(binary_mask_uint8, np.ones((2,2), np.uint8), iterations=1)
            edges[binary_mask_uint8 == 0] = 0
            self.found_plate = True

        # 'subtract' pipette mask from edges. Where self.mask is 255, set edges to 0
        # if self.using_pipette:
        inverse_mask = cv2.bitwise_not(self.pipette_mask)
        edges = cv2.bitwise_and(edges, edges, mask=inverse_mask)

        # Create an image mask which is the minAreaRect (stored in self.box) + some padding
        # Use this mask to only keep edges within or close to the bounding box
        # Edges far away from the bounding box will be removed
        if self.corners is not None:
            mask = np.zeros(edges.shape, dtype=np.uint8)
            cv2.drawContours(mask, [self.corners], 0, (255, 255, 255), -1)
            mask = cv2.dilate(mask, np.ones((20,20), np.uint8), iterations=1)
            edges = cv2.bitwise_and(edges, edges, mask=mask)

        # Convert canny image to points
        canny_pc = np.array(tuple(zip(*(np.where(edges != 0)))))

        try:
            rect = cv2.minAreaRect(canny_pc)
        except:
            rospy.loginfo("[Perception Node] Failed to find min area rect")
            return None, None, None, None
                
        # get the 4 corner points of the rectangle
        box = cv2.boxPoints(rect)  
        box = np.intp(box)
        # Swap x and y coordinates for the box points
        box = box[:, [1, 0]]

        centroid = np.mean(box, axis=0)
        corners, (x_hat, y_hat), horizontal = self.identify_box_corners(box, centroid) # [top_left, top_right, bottom_left, bottom_right]
        self.corners = np.intp(corners)

        # bbox_image = np.zeros((edges.shape[0], edges.shape[1], 3), dtype=np.uint8)
        # bbox_image[edges == 0] = [0, 0, 0]  # Black for 0
        # bbox_image[edges != 0] = [255, 255, 255]  # White for 1
        bbox_image = edges.copy()
        bbox_image = cv2.cvtColor(bbox_image, cv2.COLOR_GRAY2BGR)

        cv2.drawContours(bbox_image, [self.corners], 0, (0, 0, 255), 2)

        # draw circles at corners in different colors
        for i, corner in enumerate(corners):
            if i == 0: cv2.circle(bbox_image, corner, 3, (0, 0, 255), 3)
            elif i == 1: cv2.circle(bbox_image, corner, 3, (0, 255, 0), 3)
            elif i == 2: cv2.circle(bbox_image, corner, 3, (255, 0, 0), 3)
            else: cv2.circle(bbox_image, corner, 3, (0, 255, 255), 3)

        # draw line from centroid in direction of principal axis
        cv2.line(bbox_image, (int(centroid[0]), int(centroid[1])), (int(centroid[0] + 100 * x_hat[0]), int(centroid[1] + 100 * x_hat[1])), (255, 0, 0), 2)
        cv2.line(bbox_image, (int(centroid[0]), int(centroid[1])), (int(centroid[0] + 100 * y_hat[0]), int(centroid[1] + 100 * y_hat[1])), (0, 255, 0), 2)

        self.bbox_image_pub.publish(bridge.cv2_to_imgmsg(bbox_image, "bgr8"))

        x_axis = Vector3(x_hat[0], x_hat[1], 0)

        return corners, horizontal, x_axis
    

    def identify_box_corners(self,points,centroid):
    
        angles = [np.rad2deg(np.arctan2(-1*(point[1] - centroid[1]), point[0] - centroid[0]) % (2 * np.pi)) for point in points]

        bbox_angle = np.rad2deg(np.arctan2((points[0][1] - points[1][1]), points[0][0] - points[1][0]) % (2 * np.pi))

        new_points = [points[i] for i in np.argsort(angles)] # [top_right, top_left, bottom_left, bottom_right]
        sorted_angles = sorted(angles)
        
        # if large jump in angle, then we have gone from 0 to 360 or vice versa
        if self.previous_angles is not None:
            diff = sorted_angles[0] - self.previous_angles[0]
            # large positive jump means rotated clockwise
            if diff > 40:
                self.shifts += 1 # shift new_points one to the right
            # large negative jump means rotated counterclockwise
            elif diff < -40:
                self.shifts -= 1 # shift new_points one to the left

        # apply shifts: shift new_points to the right by self.swaps, left if self.swaps is negative
        new_points = np.roll(new_points, self.shifts, axis=0)
        self.previous_angles = sorted_angles

        
        top_left_to_top_right = np.subtract(new_points[0], new_points[1])
        x_hat = top_left_to_top_right / np.linalg.norm(top_left_to_top_right)
        
        top_right_to_bottom_right = np.subtract(new_points[0], new_points[3])
        y_hat = top_right_to_bottom_right / np.linalg.norm(top_right_to_bottom_right)

        if np.linalg.norm(top_left_to_top_right) > np.linalg.norm(top_right_to_bottom_right):
            return new_points, (x_hat, y_hat), True
        else:
            return new_points, (x_hat, y_hat), False # swap x_hat and y_hat to specify which axes to pipette across. If same, will pipette horizontally first. If different, will pipette along long axis first.

    def set_up_template(self, horizontal=True):
        well_points = []

        assert self.plate_type in [6, 24, 96], "Plate type must be 6, 24, or 96"

        if self.plate_type == 96:
            rows, cols = 8, 12
        elif self.plate_type == 24:
            rows, cols = 4, 6
        elif self.plate_type == 6:
            rows, cols = 2, 3

        plate_to_idx = {96: 0, 24: 1, 6: 2}

        well_dist = self.config["plate_dist"][plate_to_idx[self.plate_type]]
        offset_left = self.config["plate_left_offset"][plate_to_idx[self.plate_type]] + self.plate_left_correction
        offset_up = self.config["plate_up_offset"][plate_to_idx[self.plate_type]] + self.plate_up_correction
        
        if not horizontal:
            rows, cols = cols, rows
            offset_left, offset_up = offset_up, offset_left

        for i in range(0,rows):
            for j in range(0,cols):
                well_points.append(np.array([offset_left + well_dist * j, offset_up + well_dist * i]))
        
        return well_points
    
    def reset_box(self):
        # self.target = None
        self.found_plate = False
        self.corners = None
        self.previous_angles = None
        self.shifts = 0
        rospy.loginfo(f'[Perception Node] Perception failed, setting perception_success to False')
        perception_success_msg = Bool()
        perception_success_msg.data = False
        self.perception_success_pub.publish(perception_success_msg)
        error_vector = Vector3()
        error_vector.x = 0
        error_vector.y = 0
        error_vector.z = 0
        self.error_vector_pub.publish(error_vector)

        return

    def image_callback(self, msg):
        self.callback_active = True

        # Convert ROS Image message to OpenCV image
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        if self.using_homography:

            # Apply homography perspective correction ##TODO move to config file, script for homography calibration -> config
            # Obtain using homography_test_2.py

            H = [[ 9.37225626e-01,  8.79987552e-02, -1.68237507e+01],
                [ 1.17843736e-02, 1.00559354e+00,  5.16046127e+01],
                [ 1.40909538e-05,  1.63800866e-04,  1.00000000e+00]]

            H = np.array(H)
            
            height, width = img.shape[:2]
            corrected_frame = cv2.warpPerspective(img, H, (width, height))

            self.latest_frame = corrected_frame
        
        else:
            self.latest_frame = img

    def perceive(self):
        if self.latest_frame is None:
            return
                
        begin_time = time.time()
        
        frame = self.latest_frame.copy()
        # downsize to 1280x720
        frame = cv2.resize(frame, (1280, 720))

        if self.tip_coordinates is not None:
            # plot tip coordinates on image
            cv2.circle(frame, (int(self.tip_coordinates[0]), int(self.tip_coordinates[1])), 1, (0, 0, 255), 2)
        elif (self.target == "plate_tip_well" and self.tip_coordinates is None) or self.target == 'tip': 
            # if tip coordinates are None, segment the tip to get tip coordinates
            rospy.loginfo("[Perception Node] Segmenting tip...")
            try: 
                prompt_points = self.get_prompt_points(frame)
                rospy.loginfo(f"[Perception Node] Prompt points are: {prompt_points}")
                tip_coordinates, seg_mask, ann = self.segment_tip(frame, points=prompt_points)
                assert tip_coordinates is not None and seg_mask is not None

                # Success
                self.tip_coordinates = tip_coordinates
                self.pipette_mask = seg_mask
                self.ann = ann
                # self.perception_active = True
                rospy.loginfo(f"[Perception Node] Tip coordinates are: {tip_coordinates}")
                rospy.loginfo("[Perception Node] Finished segmenting tip")
            
            except:
                rospy.loginfo(f"[Perception Node] Failed to segment tip. Prompt points were: {prompt_points}")
                perception_success_msg = Bool()
                perception_success_msg.data = False
                self.perception_success_pub.publish(perception_success_msg)
                error_vector = Vector3()
                error_vector.x = 0
                error_vector.y = 0
                error_vector.z = 0
                self.error_vector_pub.publish(error_vector)
                self.perception_img_pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding="bgr8"))
                return

        if self.target is None or self.target in ['cancel', 'None']:
            self.perception_img_pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding="bgr8"))
            perception_success_msg = Bool()
            perception_success_msg.data = False
            self.perception_success_pub.publish(perception_success_msg)
            return
        
        elif self.target == 'tip':
            perception_success_msg = Bool()
            perception_success_msg.data = True
            self.perception_success_pub.publish(perception_success_msg)

            self.perception_active = True
            self.tip_coordinates = None
            
        # plate_tip_well, plate_cam_well, plate_cam_center
        elif self.target.startswith('plate'):
            # crop_frame = frame.copy()
            preprocess_img = frame.copy()

            # preprocess image
            # preprocess_img = cv2.medianBlur(preprocess_img,5) # 5 ## TODO move to config file
            preprocess_img = cv2.GaussianBlur(preprocess_img, (17, 17), 0)

            min_val = np.min(preprocess_img)
            max_val = np.max(preprocess_img)
            lower_thresh = 60 ## TODO move to config file
            upper_thresh = 185

            preprocess_img = np.uint8(np.clip((preprocess_img - min_val) / (max_val - min_val) * 255, lower_thresh, upper_thresh))
            preprocess_img = np.uint8(preprocess_img - np.min(preprocess_img))
            preprocess_img = cv2.convertScaleAbs(preprocess_img, alpha=2, beta=0)

            self.preprocess_pub.publish(self.bridge.cv2_to_imgmsg(preprocess_img, "bgr8"))

            time_preprocess = time.time()
            rospy.logdebug(f"[Perception Node] Time to preprocess image: {time_preprocess - begin_time}")

            try:
                corners, horizontal, x_axis = self.get_box_corners(preprocess_img, self.bridge) # [top_left, top_right, bottom_left, bottom_right] 

                if corners is None:
                    rospy.loginfo(f"corners is none")
                    return

                if any(element is None for element in corners):
                    rospy.loginfo("[Perception Node] Returning because None")
                    self.reset_box()
                    return
                
                # Detect if proportions of the box are consistent with a well plate
                # use ratio of edges compared to ratio of self.plate_length and self.plate_width
                # if not consistent, re-segment the plate
                # TODO: support vertical plates
                top_edge = np.linalg.norm(corners[0] - corners[1])
                right_edge = np.linalg.norm(corners[0] - corners[3])
                ratio = top_edge / right_edge
                plate_ratio = self.config['plate_length'] / self.config['plate_width']
                
                if not (0.9 * plate_ratio < ratio < 1.1 * plate_ratio):
                    rospy.loginfo("[Perception Node] Re-segmenting plate: inconsistent proportions")
                    self.reset_box()
                    return

                # Set points corresponding to corners of well plate
                if horizontal:
                    template_points = np.array([[self.config['plate_length'] + self.plate_length_offset, 0], [0, 0], [0, self.config['plate_width'] + self.plate_width_offset],[self.config['plate_length'] + self.plate_length_offset, self.config['plate_width'] + self.plate_width_offset]])
                else:
                    template_points = np.array([[self.config['plate_width'] + self.plate_width_offset, 0], [0, 0], [0, self.config['plate_length'] + self.plate_length_offset],[self.config['plate_width'] + self.plate_width_offset, self.config['plate_length'] + self.plate_length_offset]])
                
                # Set up template points
                well_points = self.set_up_template(horizontal=horizontal)

                H, _ = cv2.findHomography(template_points, corners)

                # apply transformation to well points
                well_points_homogeneous = np.hstack((well_points, np.ones((len(well_points), 1))))
                transformed_well_points_homogeneous = np.dot(H, well_points_homogeneous.T).T
                transformed_well_points = transformed_well_points_homogeneous[:, :2] / transformed_well_points_homogeneous[:, 2][:, np.newaxis]
            
                camera_center = (frame.shape[1] / 2) + 30, (frame.shape[0] / 2) + 70


                if self.target == 'plate_tip_well':
                    desired_well = self.desired_well
                    error_vector = -np.subtract(self.tip_coordinates, transformed_well_points[desired_well])
                    error_vector[1] -= 4 # Manually correct ## TODO make param
                    cv2.circle(frame, (int(self.tip_coordinates[0]), int(self.tip_coordinates[1])), 1, (0, 0, 255), 1)
                    cv2.line(frame, (int(self.tip_coordinates[0]), int(self.tip_coordinates[1])), (int(transformed_well_points[desired_well][0]), int(transformed_well_points[desired_well][1])), (0, 0, 255), 1)
                
                    # show self.ann on image as transparent blue mask
                    mask = np.zeros(frame.shape, dtype=np.uint8)
                    mask[self.ann != 0] = [255, 0, 0]
                    frame = cv2.addWeighted(frame, 1, mask, 1, 0)

                
                elif self.target == 'plate_cam_well':
                    desired_well = self.desired_well
                    error_vector = -np.subtract(camera_center, transformed_well_points[desired_well])
                    cv2.line(frame, (int(camera_center[0]), int(camera_center[1])), (int(transformed_well_points[desired_well][0]), int(transformed_well_points[desired_well][1])), (0, 0, 255), 1)

                    if self.requires_image is not None:
                        # Publish PlateGrowthMsg for visualization
                        
                        crop_frame = self.latest_frame.copy() # 1920x1080

                        wells_to_image = self.target_to_image_wells[self.requires_image]
                        for well in wells_to_image:
                            well_center = transformed_well_points[well]

                            well_center_converted = self.convert_coordinates(well_center[0], well_center[1], from_size=(1280, 720), to_size=(1920, 1080))

                            crop_radius = 12
                            crop = crop_frame[int(well_center_converted[1]) - crop_radius:int(well_center_converted[1]) + crop_radius, int(well_center_converted[0]) - crop_radius:int(well_center_converted[0]) + crop_radius]
                            avg_v = self.process_well_image.process_well_image(crop, well_num=well, iteration=self.monitor_iteration)
                            
                            setattr(self.plate_growth_msg, f'w_{well}', float(avg_v))

                        if self.requires_image == 7:
                            self.monitor_iteration += 1
                            self.plate_growth_pub.publish(self.plate_growth_msg)
                            self.plate_growth_msg = PlateGrowthMsg()
                        
                        self.requires_image = None


                elif self.target == 'plate_cam_center':
                    error_vector = -np.subtract(camera_center, np.mean(corners, axis=0))
                    cv2.line(frame, (int(camera_center[0]), int(camera_center[1])), (int(np.mean(corners, axis=0)[0]), int(np.mean(corners, axis=0)[1])), (0, 0, 255), 1)

                else:
                    rospy.loginfo("[Perception Node] Invalid target")
                    self.rate.sleep()
                    return

                # Publish annotated image
                for point in self.prompt_points:
                    cv2.circle(frame, (int(point[0]), int(point[1])), 1, (255, 0, 0), 1)

                for i, point in enumerate(transformed_well_points):
                    # if i in self.target_to_image_wells[self.targets_to_indices[self.desired_well]]:
                    cv2.circle(frame, (int(point[0]), int(point[1])), 1, (0, 255, 0), 1, cv2.LINE_AA)
                    cv2.circle(frame, (int(point[0]), int(point[1])), 5, (0, 255, 0), 1, cv2.LINE_AA)
                    # cv2.putText(frame, str(i), (int(point[0]), int(point[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                ros_img = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.annotated_image_pub.publish(ros_img)

                # If under 3 seconds since last reset, don't publish error vector yet: wait for perception to stabilize
                if time.time() - self.last_reset_time < 1: ## TODO make param
                    error_vector = Vector3()
                    error_vector.x = 0
                    error_vector.y = 0
                    error_vector.z = 0
                    self.error_vector_pub.publish(error_vector)
                    self.rate.sleep()
                    return
                

                # Publish error vector
                error_vector_msg = Vector3()
                error_vector_msg.x = error_vector[0]
                error_vector_msg.y = error_vector[1]
                error_vector_msg.z = 0
                self.error_vector_pub.publish(error_vector_msg)

                perception_success_msg = Bool()
                perception_success_msg.data = True
                self.perception_success_pub.publish(perception_success_msg)
                self.perception_active = True

                time_end = time.time()
                rospy.loginfo_throttle(10, f"[Perception Node] Frame rate: {1 / (time_end - begin_time)}")

                self.rate.sleep()

            except Exception as e:
                rospy.loginfo(f"[Perception Node] Failed to segment plate. Error: {e}")
                return
            

        elif self.target.startswith('april_tag'): 
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            results = self.at_detector.detect(gray_frame)

            r = None
            for result in results:
                if result.tag_id == int(self.april_tag_id):
                    r = result
                    break

            if r is None: 
                rospy.loginfo(f'[Perception Node] AprilTag {self.april_tag_id} not detected')
                perception_success_msg = Bool()
                perception_success_msg.data = False
                self.perception_success_pub.publish(perception_success_msg)
                # error_vector = Vector3()
                # error_vector.x = 0
                # error_vector.y = 0
                # error_vector.z = 0
                # self.error_vector_pub.publish(error_vector)
                self.perception_img_pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding="bgr8"))
                return

            M, init_error, final_error = self.at_detector.detection_pose(r, camera_params=(self.fx, self.fy, self.cx, self.cy), tag_size=self.tag_size)
            # rospy.loginfo(f'[Perception Node] M: {M}')
            self.draw_april_tag_frame(frame, camera_params=(self.fx, self.fy, self.cx, self.cy), tag_size=self.tag_size, pose=M)

            cv2.putText(frame, str(r.tag_id), (int(r.center[0]), int(r.center[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

            # compute error vector between center of tag and center of image
            error_vector = Vector3()
            tag_center = r.center + np.array([self.x_tag_offset, self.y_tag_offset]) 
            image_center = (frame.shape[1] / 2, frame.shape[0] / 2)
            error_vector.x = tag_center[0] - image_center[0]
            error_vector.y = tag_center[1] - image_center[1]
            error_vector.z = 0
            self.error_vector_pub.publish(error_vector)

            cv2.arrowedLine(frame, (int(image_center[0]), int(image_center[1])), (int(tag_center[0]), int(tag_center[1])), (0, 255, 0), 2)

            self.perception_img_pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding="bgr8"))
            perception_success_msg = Bool()
            perception_success_msg.data = True
            self.perception_success_pub.publish(perception_success_msg)

            self.perception_active = True


if __name__ == '__main__':
    p = CCPerception(name='cc_perception_node')

    while not rospy.is_shutdown():
        p.perceive()

