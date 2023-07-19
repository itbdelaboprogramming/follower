import os
import cv2
import numpy as np

ROOT_DIR = os.path.dirname(__file__)

class DarknetDNN:
    def __init__(self, dnn_model = "weights/yolov3-tiny.weights", dnn_config = "cfg/yolov3-tiny.cfg"):
        """Class for detecting object using Darknet framework.
        @param: dnn_model = Weights file of the DNN, by default is yolov3-tiny.weights
        @param: dnn_config = Config file of the DNN, by default is yolov3-tiny.cfg
        """
        #Check the installed OpenCV version
        print("Loading on OpenCV version", cv2.__version__)

        #Initiate DNN model using Darknet framework
        print("Initiating Darknet ...")
        self.dnn_model = os.path.join(ROOT_DIR, dnn_model)
        self.dnn_config = os.path.join(ROOT_DIR, dnn_config)
        self.dnn_name_lists = os.path.join(ROOT_DIR, "coco.names")
        print("Loading model from ", self.dnn_model)
        print("Loading config from ", self.dnn_config)
        print("Loading names from ", self.dnn_name_lists)
        self.net = cv2.dnn.readNet(self.dnn_model, self.dnn_config)
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)

        with open(self.dnn_name_lists, "r") as f:
            self.classes = [line.strip() for line in f.readlines()]
        
        self.layer_names = self.net.getLayerNames()

        #Check the type of output layer, some older version OpenCV has a different type of output layer
        #print("Output layer type is", type(self.net.getUnconnectedOutLayers()[0]))
        if isinstance(self.net.getUnconnectedOutLayers()[0], np.int32):
            self.output_layers = [self.layer_names[i - 1] for i in self.net.getUnconnectedOutLayers()]
        else:
            self.output_layers = [self.layer_names[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]
        
        #Blob parameter
        self.blob_scalefactor = 1/255.0
        self.blob_size = (320, 320)
        self.blob_scalar = (0, 0, 0)
        self.blob_swapRB = True
        self.blob_crop = False
        self.blob_ddepth = cv2.CV_32F

        #Threshold for detecting object
        self.confidence_threshold = 0.3
        self.nms_threshold = 0.4
        self.color_threshold  = 0

        #Color HSV Range
        self.lower_hsv = np.array([0, 0, 0])
        self.upper_hsv = np.array([179, 255, 255])

        pass

    def set_hsv_range(self, low: np.ndarray, high: np.ndarray):
        """ Method to set hsv range for color checking.
        @param:
         low: lower bound hsv in np.array([hue, sat, val]) format.
         high: upper bound hsv in np.array([hue, sat, val]) format.
        """
        self.lower_hsv = low
        self.upper_hsv = high
        pass

    def set_color_threshold(self, value: float):
        """ Method for setting the value of color_threshold for color checking (default value is 0).
        @param:
         value: the value for color threshold.
        """
        self.color_threshold = value
        pass

    def set_confidence_threshold(self, value: float):
        """ Method for setting the value of confidence_threshold for object detection (default value is 0.3).
        @param:
         value: the value for confiedence threshold.
        """
        self.confidence_threshold = value
        pass

    def set_nms_threshold(self, value: float):
        """ Method for setting the value of nms_threshold for object detection (default value is 0.4).
        @param:
         value: the value for confiedence threshold.
        """
        self.nms_threshold = value
        pass

    def detect_object(self, image: np.ndarray, target_id:int = None):
        """ Method to detect object within the frame. The target id specifies what object to detect, None by default. The id can be seen as the index of the coco.names list.
        @param
         image: the image to scan for detection in OpenCV Matrix format <numpy.ndarray>.
         target_id: object id to detect. For detecting Person, use id = 0.
        This will return the list of detected object.   
        """
        #Pre-process the input image
        self.image_height, self.image_width, self.image_channels = image.shape
        blob = cv2.dnn.blobFromImage(image, 
                                     self.blob_scalefactor, 
                                     self.blob_size, 
                                     self.blob_scalar, 
                                     self.blob_swapRB, 
                                     self.blob_crop, 
                                     self.blob_ddepth)

        #Pass the blob as input into the DNN
        self.net.setInput(blob)

        #Wait for the output
        outputs:tuple = self.net.forward(self.output_layers)

        #Detected object information
        self.object_classes = []
        self.object_confidences = []
        self.object_boxes = []

        for output in outputs:
            for detection in output:
                #Takes the detection scores
                scores = detection[5:]

                #The object id detected is the largest scores
                class_id = np.argmax(scores)

                #The confidence of the detected object is the scores of the detected object
                confidence = scores[class_id]

                #Filter out if the object has low confidence
                if confidence <= self.confidence_threshold:
                    continue

                #Filter out non-human object
                if target_id is not None and class_id != target_id:
                    continue

                #Get the location of the detected object in the frame input
                cx = int(detection[0] * self.image_width)
                cy = int(detection[1] * self.image_height)
                w = int(detection[2] * self.image_width)
                h = int(detection[3] * self.image_height)
                x1 = int(cx - w/2)
                y1 = int(cy - h/2)
                x2 = int(cx + w/2)
                y2 = int(cy + h/2)

                self.object_boxes.append([x1, y1, x2, y2])
                #self.object_boxes.append([cx, cy, w, h])
                self.object_classes.append(self.classes[class_id])
                self.object_confidences.append(confidence)

        #Perform NMS to the detected object to eliminate redundant detection
        indexes = cv2.dnn.NMSBoxes(self.object_boxes, 
                                   self.object_confidences, 
                                   self.confidence_threshold, 
                                   self. nms_threshold)
        self.final_boxes = []
        self.final_classes = []
        self.final_confidences = []
        self.final_color_confidences = []
        for i in indexes:
            self.final_boxes.append(self.object_boxes[i])
            self.final_classes.append(self.object_classes[i])
            self.final_confidences.append(self.object_confidences[i])
            color_confidences = self.calculate_color_confidences(image,
                                                                 self.object_boxes[i],
                                                                 self.lower_hsv,
                                                                 self.upper_hsv)
            self.final_color_confidences.append(color_confidences)

        self.find_target()

        #return self.final_boxes

    def calculate_color_confidences(self, image: np.ndarray, bbox: list, low_hsv: np.ndarray, upp_hsv: np.ndarray):
        """ Method to calculate total area of pixel within the boundary of two hsv values.
        @param:
         image: image of the frame to scan in np.ndarray format
         bbox: bounding box coordinate to calculate [x1, y1, x2, y2]
         low_hsv: lower bound of the hsv value
         upp_hsv: upper bound of the hsv value

        @return:
         total_area of the color in image
        """
        x1, y1, x2, y2 = bbox
        x1 = np.clip(x1, 0, self.image_width)
        y1 = np.clip(y1, 0, self.image_height)
        x2 = np.clip(x2, 0, self.image_width)
        y2 = np.clip(y2, 0, self.image_height)

        roi = image[y1:y2, x1:x2]
        roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(roi, low_hsv, upp_hsv)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        total_area = 0
        for contour in contours:
            area = cv2.contourArea(contour)
            total_area += area

        return total_area
    
    def find_target(self):
        """Function to get the target by finding target with maximum color confidences."""
        self.target_index = None
        self.target_box = None
        self.target_class = None
        self.target_confidence = None
        self.target_color_confidence = None

        if len(self.final_boxes) != 0:
            self.target_index = np.argmax(self.final_color_confidences)
            self.target_box = self.final_boxes[self.target_index]
            self.target_class = self.final_classes[self.target_index]
            self.target_confidence = self.final_confidences[self.target_index]
            self.target_color_confidence = self.final_color_confidences[self.target_index]
            if self.target_color_confidence < self.color_threshold:
                self.target_index = None
                self.target_box = None
                self.target_class = None
                self.target_confidence = None
                self.target_color_confidence = None
        
    def get_target_box(self):
        """Function to get target bounding box."""
        return self.target_box
    
    def get_target_class(self):
        """Function to get target class."""
        return self.target_class
    
    def get_target_confidence(self):
        """Function to get target confidence."""
        return self.target_confidence
    
    def get_target_color_confidence(self):
        """Function to get target color confidence."""
        return self.target_color_confidence
    
    def get_target_center(self):
        """Function to get target center"""
        if self.target_box is not None:
            x1, y1, x2, y2 = self.target_box
            cx = int((x1 + x2)/2)
            cy = int((y1 + y2)/2)
            return cx, cy
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
        
    def show_target(self, image: np.ndarray, info: bool = False):
        """Method to show all the detected object and target info onto the frame.
        @param:
         image: image of the frame to drawn the info in np.ndarray format.
         info: if True it will show all the info of the object (default is False).
        """
        
        for i, _ in enumerate(self.final_boxes):
            x1, y1, x2, y2 = self.final_boxes[i]
            label = self.final_classes[i]
            confidence = self.final_confidences[i]
            color_confidence = self.final_color_confidences[i]

            font = cv2.FONT_HERSHEY_SIMPLEX
            if i == self.target_index:
                color = (0, 0, 255)
            else:
                color = (0, 255, 0)

            #Draw the bounding box
            cv2.rectangle(image, (x1, y1), (x2, y2), color, 1)
            
            if info:
                #Draw the id of object detected
                cv2.rectangle(image, (x2, y1), (x2 - 20, y1 + 20), (0,0,0), cv2.FILLED)
                cv2.putText(image, f"{i}", (x2 - 20, y1 + 20), font, 0.5, color, 1)

                #Draw the label info
                label_text_size, _ = cv2.getTextSize(f"{label}", font, 0.5, 1)
                cv2.rectangle(image, (x1, y1), (x1 + label_text_size[0], y1 + label_text_size[1]), (0,0,0), cv2.FILLED)
                cv2.putText(image, f"{label}", (x1, y1 + label_text_size[1]), font, 0.5, color, 1)

                #Draw the confidence info
                confidence_text_size, _ = cv2.getTextSize(f"{confidence:.2f}", font, 0.5, 1)
                cv2.rectangle(image, (x1, y1 + label_text_size[1]), (x1 + confidence_text_size[0], y1 + label_text_size[1] + confidence_text_size[1]), (0,0,0), cv2.FILLED)
                cv2.putText(image, f"{confidence:.2f}", (x1, y1 + label_text_size[1] + confidence_text_size[1]), font, 0.5, color, 1)

                #Draw the color confidence info
                color_text_size, _ = cv2.getTextSize(f"{color_confidence}", font, 0.5, 1)
                cv2.rectangle(image, (x1, y1 + label_text_size[1] + confidence_text_size[1]), (x1 + color_text_size[0], y1 + label_text_size[1] + confidence_text_size[1] + color_text_size[1]), (0,0,0), cv2.FILLED)
                cv2.putText(image, f"{color_confidence}", (x1, y1 + label_text_size[1] + confidence_text_size[1] + color_text_size[1]), font, 0.5, color, 1)
                
        pass
    

    """
    Function/method below are just for testing
    """
        
    def show_detected_object(self, frame):
        for i, _ in enumerate(self.object_boxes):
            x1, y1, x2, y2 = self.object_boxes[i]
            #cx, cy, w, h = self.object_boxes[i]
            label = self.object_classes[i]
            color = (0, 255, 0)

            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 1)

            text_size, _ = cv2.getTextSize(label.capitalize(), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(frame, (x1 + 5, y1 + 5), (x1 + 5 + text_size[0], y1 + 5 - text_size[1]), (0,0,0), cv2.FILLED)
            cv2.putText(frame, label.capitalize(), (x1 + 5, y1 + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    
    def show_detected_nms_object(self, frame):
        for i, _ in enumerate(self.final_boxes):
            x1, y1, x2, y2 = self.final_boxes[i]
            #cx, cy, w, h = self.object_boxes[i]
            label = self.final_classes[i]
            color = (0, 0, 255)

            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 1)

            text_size, _ = cv2.getTextSize(label.capitalize(), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(frame, (x1 + 5, y1 + 5), (x1 + 5 + text_size[0], y1 + 5 - text_size[1]), (0,0,0), cv2.FILLED)
            cv2.putText(frame, label.capitalize(), (x1 + 5, y1 + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    


def main():
    net = DarknetDNN()
    cap = cv2.VideoCapture(0)

    while True:
        _, frame = cap.read()

        low_hsv = np.array([0, 221, 102], dtype=np.uint8)
        high_hsv = np.array([73, 255, 255], dtype=np.uint8)

        print(net.get_target_confidence())
        print(f"Position : {net.get_target_position()}")
        print(f"Color confidence : {net.get_target_color_confidence()}")
        print(f"Distance : {net.get_target_distance()}")

        cv2.imshow("Video", frame)

        #exit condition
        key = cv2.waitKey(1)
        if key == 27:
            print(f"Key {key} is pressed.")
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()