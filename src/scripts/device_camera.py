import cv2
import numpy as np

class DeviceCamera:
    """
    This Class is for handling the camera device.

    The class will try to use Intel Realsense python library (pyrelsense2) but it also able to use any camera device by passing the device id argument and set the realsense flag to false.
    
    You can also scan the device id.
    """
    def __init__(self, device_id = None, realsense = True):
        print("Loading camera ...")

        # Check if pyrealsense2 is available
        self.realsense, self.rs = self.check_pyrealsense2() if realsense else (False, None)

        # Camera parameter initialization
        self.device_id = device_id
        self.device_ids = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
        self.capture = None
        self.pipeline = None
        self.winname = None

        # Initialize device
        #print(self.realsense)
        #print(self.rs)
        if self.realsense:
            print("Starting realsense")
            self.winname = "Realsense"
            self.stream_realsense()
        else:
            print("Starting regular stream")
            self.winname = "Regular Stream"
            self.stream_regular()

        # FPS Calculation
        self.tick_frequency = cv2.getTickFrequency()
        self.start_time = cv2.getTickCount()
        self.frame_count = 0
        self.fps = None

        #Font parameters
        self.font_face = cv2.FONT_HERSHEY_SIMPLEX
        self.org = (0, 50)
        self.font_scale = 0.5
        self.font_color = (90, 252, 3)
        self.font_thickness = 1
        self.font_line_type = cv2.LINE_AA
        self.font_bottom_left_origin = False
    
    def check_pyrealsense2(self):
        try:
            import pyrealsense2
            rs = pyrealsense2
            print("Pyrealsense2 is available")
            return True, rs
        except ImportError:
            print("Pyrealsense2 is not available")
            return False, None

    def stream_realsense(self):
        # Configure depth and color streams
        self.pipeline = self.rs.pipeline()
        config = self.rs.config()
        config.enable_stream(self.rs.stream.depth, 640, 480, self.rs.format.z16, 30)
        config.enable_stream(self.rs.stream.color, 640, 480, self.rs.format.bgr8, 30)

        # Start streaming
        self.pipeline.start(config)
        
        # Align the depth stream with color stream
        self.align = self.rs.align(self.rs.stream.color)

    def stream_regular(self):
        # Searching for the first available device id if not specified
        if self.device_id is None:
            self.device_id = self.search_available_device_id()
        
        # Start streaming
        self.capture = cv2.VideoCapture(self.device_id)
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        print("Starting on device ", self.device_id)

    def get_frame(self):
        if self.realsense:
            # Read the incoming frame from Realsense
            frames = self.pipeline.wait_for_frames()

            # Aligned the color frame and depth frame
            aligned_frames = self.align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()

            # Check if the stream is success or not
            if not color_frame or not depth_frame:
                print("Error, impossible to get the frame, make sure that the Intel Realsense camera is correctly connected")
                return None, None
            
            # Convert the frame into Matrix
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            return color_image, depth_image
        else:
            # Read the incoming frame from Regular Camera
            retval, frame = self.capture.read()

            return frame, None

    def show_fps(self, frame):
        self.frame_count += 1
        current_time = cv2.getTickCount()
        elapsed_time = (current_time - self.start_time)/self.tick_frequency

        if elapsed_time >= 0.25:
            self.fps = round(self.frame_count / elapsed_time, 2)
            #print(fps)
            self.start_time = current_time
            self.frame_count = 0

        text_size, _ = cv2.getTextSize(f"FPS: {self.fps}", self.font_face, self.font_scale, self.font_thickness)
        cv2.rectangle(frame,(self.org[0] + text_size[0], self.org[1] - text_size[1]), self.org, (0,0,0), cv2.FILLED)
        cv2.putText(frame, f"FPS: {self.fps}", self.org, self.font_face, self.font_scale, self.font_color, self.font_thickness, self.font_line_type, self.font_bottom_left_origin)
        return frame

    def stop(self):
        if self.realsense:
            self.pipeline.stop()
        else:
            self.capture.release()

    def show_color(self):
        color, depth = self.get_frame()
        cv2.imshow(self.winname, color)
    
    def show_depth(self):
        color, depth = self.get_frame()
        cv2.imshow(self.winname, depth)

    def validate_device_id(self, device_id):
        try:
            cap = cv2.VideoCapture(device_id)
            is_opened = cap.isOpened()
            cap.release()
            return is_opened
        except Exception as e:
            return False
    
    def search_available_device_id(self):
        for device_id in self.device_ids:
            if self.validate_device_id(device_id):
                return device_id
        return None
    
    def available_device_id(self):
        available_device = [device_id for device_id in self.device_ids if self.validate_device_id(device_id)]
        return available_device

def main():
    #net = DarknetDNN()
    camera = DeviceCamera()
    
    while True:
        color, depth = camera.get_frame()

        color = camera.show_fps(color)

        #net.detect_object(color)

        #net.draw_detected_object(color, depth)

        normalized_depth = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX)

        cv2.imshow("Color", color)
        cv2.imshow("Depth", normalized_depth)

        key = cv2.waitKey(1)
        if key == ord('q') or key == 27:
            print(f"Key {key} is pressed.")
            break
    
    camera.stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()


        