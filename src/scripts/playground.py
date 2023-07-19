import cv2
import time
from darknet_yolo import DarknetDNN
from tracker import ObjectTracker
from darknet_yolo_2 import DarknetDNN as DN

def constrain(val, low, high):
    if val < low:
        return low
    elif val > high:
        return high
    else:
        return val

net = DarknetDNN()
cap = cv2.VideoCapture(0)
tr = ObjectTracker()

bbox = None
tracking = False
frame_count = 0
start_time = time.time()

net2 = DN()
net2.set_hsv_range()
net2.set_color_threshold()
net2.set_confidence_threshold()
net2.set_nms_threshold()
net2.detect_object()


while True:
    retval, frame = cap.read()
    #print(frame.shape)
    frame_count += 1
    

    if tracking:
        #tracker2 = cv2.legacy.TrackerMedianFlow_create()
        tr.create(7)
        net.hunt(frame)
        net.draw_hunted_target(frame)
        bbox = net.get_target_bbox()
        #print(bbox)
        if bbox is not None:
            x1, y1, x2, y2 = bbox
            x1 = constrain(x1, 0, 640)
            y1 = constrain(y1, 0, 480)
            x2 = constrain(x2, 0, 640)
            y2 = constrain(y2, 0, 480)
            kotak = [x1, y1, x2-x1, y2-y1]
            #print(kotak)
            ret = tr.set_target(frame, kotak)
            #print(ret)
            #ret = tracker2.init(frame, kotak)
            print(ret)
            tracking = True
    
    if tracking:
        
        #success, box = tracker2.update(frame)
        success, box = tr.update(frame)
        #success, box = tr.tracker.update(frame)

        if success:
            #print("Tracking")
            x1, y1, w, h = [int(v) for v in box]
            cv2.rectangle(frame, (x1, y1), (x1 + w , y1 + h), (255,0,0), 2)
        else:
            #print("Not Tracking")
            #tracker2 = None
            tr.create(0)
            #tr.tracker = None
            tracking = False

    current_time = time.time()
    elapsed_time = current_time - start_time
    if elapsed_time >= 1:
        fps = round(frame_count/elapsed_time, 2)
        start_time = current_time
        frame_count = 0
        print(fps)
    
    cv2.imshow("Object", frame)

    # Exit condition
    key = cv2.waitKey(1)
    if key == 27 or key == ord('q'):
        print(f"Key {key} is pressed")
        break

cap.release()
cv2.destroyAllWindows()



