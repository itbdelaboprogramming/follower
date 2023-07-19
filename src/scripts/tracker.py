import cv2
import numpy as np
from darknet_yolo import DarknetDNN

class ObjectTracker(object):
    def __init__(self, algorithm: int = 0):
        """
        The algorithm list are:
        1: Boosting
        2: CSRT
        3: KCF
        4: Median Flow
        5: MIL
        6: MOSSE
        7: TLD

        Other value and 0 will erase the tracker object.
        """
        self.tracker = None
        self.target_bounding_box = None
        self.algorithm = algorithm
        self.tracking_flag = False
    
    def create(self):
        """
        Method to create an object tracker from OpenCV library.
        """
        if self.algorithm == 0:
            self.tracker = None
        elif self.algorithm == 1:
            self.tracker = cv2.legacy.TrackerBoosting_create()
        elif self.algorithm == 2:
            self.tracker = cv2.legacy.TrackerCSRT_create()
        elif self.algorithm == 3:
            self.tracker = cv2.legacy.TrackerKCF_create()
        elif self.algorithm == 4:
            self.tracker = cv2.legacy.TrackerMedianFlow_create()
        elif self.algorithm == 5:
            self.tracker = cv2.legacy.TrackerMIL_create()
        elif self.algorithm == 6:
            self.tracker = cv2.legacy.TrackerMOSSE_create()
        elif self.algorithm == 7:
            self.tracker = cv2.legacy.TrackerTLD_create()
        else:
            self.tracker = None
    
    def set_target(self, frame: np.ndarray, bounding_box: list):
        """
        Function to set the target that you want to track.
        @param:
         frame: the image or frame that we want to scan.
         bounding_box: the coordinate of object that we want to track within the frame. The format for the bounding_box are [x, y, w, h] where x and y are the coordinate of the top-left corner of the bounding box, w and h are the width and height of the bounding box.
        
        This function will return True if the target initiation is success and will return false otherwise.
        """
        self.image_height, self.image_width, self.image_channels = frame.shape
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
        self.create()
        net.detect_object(frame, 0)
        net.show_target(frame)
        bbox = net.get_target_box()
        
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
        success, self.target_bounding_box = self.update(frame)
        if success:
            x1, y1, w, h = [int(n) for n in self.target_bounding_box]
            #cx = int(x1 + w/2)
            #cy = int(y1 + h/2)
            cx, cy = self.get_target_center()
            cv2.rectangle(frame, (x1,y1), (x1+w, y1+h), (255,0,0), 2)
            cv2.circle(frame, (cx,cy), 5, (255,0,0), 1)
        else:
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
        if not self.tracking_flag:
            self.not_tracking(frame, net)
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
        
    def get_target_position(self):
        """Function to get target position. Image is divided into 3 sector in x axis. If the target is on the Left, Center or Right it will return String with that sector, else it will return 'Hold'."""
        cx, cy = self.get_target_center()
        
        if cx is None:
            return 'Hold'
        
        if cx <= self.image_width/3:
            return 'Left'
        elif cx >= 2 * self.image_width/3:
            return 'Right'
        else:
            return 'Center'
        
    def get_target_distance(self, depth: np.ndarray):
        """Function to get the target distance in cm.
        @param:
         depth: depth image from IntelRealsense in np.ndarray format
        """
        if depth is None:
            return None
        else:
            cx, cy = self.get_target_center()
            return round(depth[cy, cx]/10)