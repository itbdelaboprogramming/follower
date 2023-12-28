import cv2
import numpy as np
import time as tm
from darknet_yolo import DarknetDNN
import cv2.aruco as aruco
import os

"""
Real-Time Object Tracking with ObjectTracker and DarknetDNN

Overview:
This Python script demonstrates real-time object tracking by integrating the "ObjectTracker" module for object tracking and the "DarknetDNN" module for YOLO (You Only Look Once) object detection. It enables the detection and tracking of objects within a video stream from a camera.

Libraries:
- The script imports necessary libraries, including OpenCV for video processing and NumPy for array operations.
- Custom modules "ObjectTracker" and "DarknetDNN" are utilized for object tracking and YOLO detection, respectively.

Utility Function:
- A utility function "constrain" is defined to limit a value within a specified range.

Initialization:
- The YOLO object detection model is initialized using "DarknetDNN."
- A video capture object ("cap") is created to capture video from the default camera (camera ID 0).
- An instance of the "ObjectTracker" class is initialized for object tracking.
- Variables for bounding box ("bbox"), tracking state ("tracking"), frame count, and start time are initialized.

Secondary YOLO Model:
- A secondary YOLO model ("net2") is mentioned with various configuration settings. Details about its purpose and configuration are provided in the code comments.

Main Loop:
- The script enters a continuous loop for processing video frames.
- Each iteration reads a frame from the camera using "cap.read()" and increments the frame count.

Object Tracking:
- If object tracking is enabled ("tracking" is True):
  - The script attempts to detect and hunt for an object using the YOLO model ("net").
  - Detected objects are highlighted with bounding boxes.
  - Bounding box coordinates are extracted and constrained to fit within the frame.
  - The target for tracking is set using the "ObjectTracker."

Object Tracking Update:
- If tracking is in progress:
  - The tracker is updated with the current frame.
  - If the update is successful, a rectangle is drawn around the tracked object.

FPS Calculation:
- Frames Per Second (FPS) is calculated and displayed to monitor real-time video processing performance.

User Interaction:
- The processed frame with object tracking is displayed in a window named "Object."
- The program can be exited gracefully by pressing 'q' or the 'Esc' key.

Cleanup:
- After exiting the loop, the video capture object ("cap") is released.
- All OpenCV windows are closed to free up system resources.

Note:
- The script mentions the creation of a secondary YOLO model ("net2") with additional configurations. While this code provides the structure, specific details about "net2" are not included, and further information can be found in the relevant code comments.

Overall, this code exemplifies the implementation of real-time object tracking by combining YOLO detection with OpenCV's tracking capabilities.
"""


class ObjectTracker(object):
    def __init__(self, algorithm: int = 0, use_aruco: bool = False, aruco_id: int = 0, enable_transducer: bool = False):
        """
        Initialize an object tracker with various tracking algorithms.

        Args:
            algorithm (int): The tracking algorithm to use (0 to erase, 1 to 7 for specific algorithms).

        Algorithm List (4.8.1) :
            1: DasiamRPN | 2: CSRT | 3: KCF | 4: GOTURN | 5: MIL | 6: Nano
        
        Algorithm List (4.2.0.32) :
            1: Boosting | 2: CSRT | 3: KCF | 4: Median Flow | 5: MIL | 6: MOSSE | 7: TLD

        Other values and 0 will erase the tracker object.
        """
        self.tracker = None
        self.target_bounding_box = None
        self.algorithm = algorithm
        self.tracking_flag = False
        self.tracking_time = tm.time()
        self.frame_height = None
        self.frame_width = None
        self.use_aruco = use_aruco
        self.aruco_id = aruco_id
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        self.enable_transducer = enable_transducer
    
    def create(self):
        """
        Create an object tracker from the OpenCV library based on the selected algorithm.
        """
        # 4.8.1
        current_file_dir = os.path.dirname(os.path.realpath(__file__))
        model_dir = f"{current_file_dir}/model"
        if self.algorithm == 0:
            self.tracker = None
        elif self.algorithm == 1:
            """
            DaSiamRPN ONNX Model download
            network:     https://www.dropbox.com/s/rr1lk9355vzolqv/dasiamrpn_model.onnx?dl=0
            kernel_r1:   https://www.dropbox.com/s/999cqx5zrfi7w4p/dasiamrpn_kernel_r1.onnx?dl=0
            kernel_cls1: https://www.dropbox.com/s/qvmtszx5h339a0w/dasiamrpn_kernel_cls1.onnx?dl=0
            """
            params = cv2.TrackerDaSiamRPN_Params()
            params.model = f"{model_dir}/dasiamrpn_model.onnx"
            params.kernel_cls1 = f"{model_dir}/dasiamrpn_kernel_cls1.onnx"
            params.kernel_r1 = f"{model_dir}/dasiamrpn_kernel_r1.onnx"
            self.tracker = cv2.TrackerDaSiamRPN_create(params)
        elif self.algorithm == 2:
            self.tracker = cv2.TrackerCSRT_create()
        elif self.algorithm == 3:
            self.tracker = cv2.TrackerKCF_create()
        elif self.algorithm == 4:
            """
            GOTURN Model download
            https://github.com/Mogball/goturn-files
            """
            params = cv2.TrackerGOTURN_Params()
            params.modelTxt = f"{model_dir}/goturn.prototxt"
            params.modelBin = f"{model_dir}/goturn.caffemodel"
            self.tracker = cv2.TrackerGOTURN_create(params)
        elif self.algorithm == 5:
            self.tracker = cv2.TrackerMIL_create()
        elif self.algorithm == 6:
            """
            NanoTrack ONNX Model download
            https://github.com/HonglinChu/SiamTrackers/tree/master/NanoTrack/models/nanotrackv2
            """
            params = cv2.TrackerNano_Params()
            params.backbone = f"{model_dir}/nanotrack_backbone_sim.onnx"
            params.neckhead = f"{model_dir}/nanotrack_head_sim.onnx"
            self.tracker = cv2.TrackerNano_create(params)
        else:
            self.tracker = None

        """
        # 4.2.0.32
        if self.algorithm == 0:
            self.tracker = None
        elif self.algorithm == 1:
            #self.tracker = cv2.legacy.TrackerBoosting_create()
            self.tracker = cv2.TrackerBoosting_create()
        elif self.algorithm == 2:
            #self.tracker = cv2.legacy.TrackerCSRT_create()
            self.tracker = cv2.TrackerCSRT_create()
        elif self.algorithm == 3:
            #self.tracker = cv2.legacy.TrackerKCF_create()
            self.tracker = cv2.TrackerKCF_create()
        elif self.algorithm == 4:
            #self.tracker = cv2.legacy.TrackerMedianFlow_create()
            self.tracker = cv2.TrackerMedianFlow_create()
        elif self.algorithm == 5:
            #self.tracker = cv2.legacy.TrackerMIL_create()
            self.tracker = cv2.TrackerMIL_create()
        elif self.algorithm == 6:
            #self.tracker = cv2.legacy.TrackerMOSSE_create()
            self.tracker = cv2.TrackerMOSSE_create()
            #self.tracker = cv2.Tracker("MOSSE")
        elif self.algorithm == 7:
            #self.tracker = cv2.legacy.TrackerTLD_create()
            self.tracker = cv2.TrackerTLD_create()
        else:
            self.tracker = None
        """
         
    def set_target(self, frame: np.ndarray, bounding_box: list):
        """
        Function to set the target that you want to track.
        @param:
         frame: the image or frame that we want to scan.
         bounding_box: the coordinate of object that we want to track within the frame. The format for the bounding_box are [x, y, w, h] where x and y are the coordinate of the top-left corner of the bounding box, w and h are the width and height of the bounding box.
        
        This function will return True if the target initiation is success and will return false otherwise.
        """
        return self.tracker.init(frame, bounding_box)

    def update(self, frame: np.ndarray):
        """Function to update the frame from last tracked object.
        @param:
         frame: the frame of input camera or image
        @return:
         [True || False, [Bounding Box]] will return True if the tracked object still exist in the current frame and false otherwise. It also wil returun the bounding box of tracked object in current frame.
        """
        #success, self.target_bounding_box = self.tracker.update(frame)
        return self.tracker.update(frame)
    
    def clear(self):
        """This method will clear the tracking object"""
        self.tracker = None
    
    def not_tracking(self, frame: np.ndarray, net: DarknetDNN):
        """This method is for detecting a person using DarknetDNN and then initiate a tracking object by selecting the target for object tracking.
        @param:
         frame: the current frame input for detection and tracking.
         net: DarknetDNN object for object detection.
        """
        if self.use_aruco:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict)
            if ids is not None:
                self.tracking_flag = True if [self.aruco_id] in ids else False
        else:
            self.create()
            net.detect_object(frame, 0)
            net.show_target(frame)
            bbox = net.get_target_box()
            self.target_bounding_box = None
            # print(bbox)
            if bbox is not None:
                x1, y1, x2, y2 = bbox
                self.target_bounding_box = [x1, y1, x2 - x1, y2 - y1]
                ret = self.set_target(frame, self.target_bounding_box)
                self.tracking_flag = True
                #print(ret)
    
    def tracking(self, frame: np.ndarray):
        """This method is for tracking and updating the bounding box for the tracked object after detecting it.
        @param:
         frame: the current input frame from camera.
        """
        reset = False
        if self.use_aruco:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict)
            if ids is not None and [self.aruco_id] in ids:
                idx = np.where(ids == self.aruco_id)[0][0]
                [x1, y1] = corners[idx][0][0]
                [x2, y2] = corners[idx][0][2]
                self.target_bounding_box = [x1, y1, x2 - x1, y2 - y1]
                aruco.drawDetectedMarkers(frame, corners, ids)
                cx, cy = self.get_target_center()
                cv2.circle(frame, (cx,cy), 5, (255,0,0), 1)
            else:
                reset = True
        else:
            success, self.target_bounding_box = self.update(frame)
            if success:
                x1, y1, w, h = [int(n) for n in self.target_bounding_box]
                #cx = int(x1 + w/2)
                #cy = int(y1 + h/2)
                cx, cy = self.get_target_center()
                cv2.rectangle(frame, (x1,y1), (x1+w, y1+h), (255,0,0), 2)
                cv2.circle(frame, (cx,cy), 5, (255,0,0), 1)
            else:
                reset = True

        if reset:
            self.clear()
            self.tracking_flag = False
            self.target_bounding_box = None
        pass
    
    def track_object(self, frame: np.ndarray, net: DarknetDNN):
        """This method is for detecting an object first using DarknetDNN then if the detected object being tracked using ObjectTracker
        @param:
         frame: input frame from camera.
         net: DarknetDNN object for object detection.
        """
        self.image_height, self.image_width, self.image_channels = frame.shape
        if not self.tracking_flag:
            self.not_tracking(frame, net)
        else:
            self.tracking(frame)
        pass

    def track_object_with_time(self, frame: np.ndarray, net: DarknetDNN, duration: float):
        """This method is for detecting an object first using DarknetDNN then if the detected object being tracked using ObjectTracker
        @param:
         frame: input frame from camera.
         net: DarknetDNN object for object detection.
         duration: time length before detection algorithm is re executed.
        """
        self.image_height, self.image_width, self.image_channels = frame.shape
        current_time = tm.time()
        elapsed_time = current_time - self.tracking_time
        self.frame_height, self.frame_width, _ = frame.shape

        if elapsed_time >= duration:
            self.tracking_flag = False
            self.tracking_time = current_time

        if not self.tracking_flag:
            self.not_tracking(frame, net)
            self.tracking_time = current_time
            #print(f"{elapsed_time}, detecting")
        else:
            self.tracking(frame)
        pass

    def get_target_center(self):
        """Function to get target center"""
        if self.target_bounding_box is not None:
            x1, y1, w, h = self.target_bounding_box
            return int(x1 + w/2), int(y1 + h/2)
        else:
            return None, None
        
    def get_target_position(self, depth: np.ndarray, obs_threshold: float, ultrasonic_target_direction: float):
        """Function to get target position. Image is divided into 3 sector in x axis. 
           If the target is on the Left, Center or Right it will return move command (string) with that sector, else it will return 'Hold'.
           If the target is on the Top or Bottom it will return cam angle command (string) with that sector, else it will return 'Hold'.
            @param:
            depth: depth image from IntelRealsense in np.ndarray format 
                    (each element is in mm)
            obs_threshold: obstacle stop threshold value in meter
        """
        cx, cy = self.get_target_center()
        obs_left, obs_center, obs_right = self.is_obstacle_within_threshold(depth, obs_threshold)
        if cx is None:
            if self.enable_transducer:
                if ((330.0 < ultrasonic_target_direction <= 359.9) or (0.0 <= ultrasonic_target_direction < 30.0)) and not obs_left and not obs_right:
                    move_cmd = 'Center'
                elif (30.0 <= ultrasonic_target_direction <= 180.0) and not obs_right:
                    move_cmd = 'Right'
                elif (180.0 < ultrasonic_target_direction <= 330.0) and not obs_left:
                    move_cmd = 'Left'
                else:
                    move_cmd = 'Hold'
            else:
                move_cmd = 'Hold'
        else:
            if cx <= self.image_width/3 and not obs_left:
                move_cmd = 'Left'
            elif cx >= 2 * self.image_width/3 and not obs_right:
                move_cmd = 'Right'
            elif not obs_center and not obs_left and not obs_right:
                move_cmd = 'Center'
            else:
                move_cmd = 'Hold'
        
        if cy is None:
            cam_angle_cmd = 'Hold'
        else:
            if cy <= self.image_height/3:
                cam_angle_cmd = 'Up'
            elif cy >= 2 * self.image_height/3:
                cam_angle_cmd = 'Down'
            else:
                cam_angle_cmd = 'Hold'
        
        return move_cmd, cam_angle_cmd
        
    def get_target_distance(self, depth: np.ndarray, ultrasonic_target_distance: float):
        """Function to get the target distance in m.
        @param:
         depth: depth image from IntelRealsense in np.ndarray format
        """
        cx, cy = self.get_target_center()
        if depth is None or cx is None:
            if self.enable_transducer:
                return ultrasonic_target_distance
            else:
                return None
        elif cx > self.frame_width or cx < 0:
            return None
        elif cy > self.frame_height or cy < 0:
            return None
        else:
            
            distance = depth[cy, cx]/1000
            """
            try:
                distance = depth[cy, cx]/1000
            except BaseException as e:
                print(e)
                pass
            """
            return distance
    
    def is_obstacle_within_threshold(self, depth: np.ndarray, threshold: float):
        """Function to check if there is an obstacle within threshold.
        @param:
         depth: depth image from IntelRealsense in np.ndarray format 
                (each element is in mm)
         threshold: stop threshold value in meter
        @variable:
          pixel_list: [[x,y], [x,y], ...] with x and y as image coordinate that has depth value within threshold
          pixel_list[:, 0]: list of x coordinate
          obj_left: True if there is an object on the left (1/3 left of the image)
          obj_center: True if there is an object on the center (1/3 center of the image)
          obj_right: True if there is an object on the right (1/3 right of the image)
        @return:
            bool, bool, bool --> (obj on left, obj on center, obj on right)
        """
        if depth is not None:
            threshold_mm = threshold * 1000
            indexes = np.where((depth > 0.0) & (depth <= threshold_mm))
            row_indexes, col_indexes = indexes
            if len(row_indexes) > 0 and len(col_indexes) > 0:
                pixel_list = np.column_stack((col_indexes, row_indexes))
                obj_left = np.any(pixel_list[:, 0] < self.image_width/3)
                obj_center = np.any((pixel_list[:, 0] >= self.image_width/3) & (pixel_list[:, 0] <= 2 * self.image_width/3))
                obj_right = np.any(pixel_list[:, 0] > 2 * self.image_width/3)
                return obj_left, obj_center, obj_right
            else:
                return False, False, False
        else:
            return False, False, False
