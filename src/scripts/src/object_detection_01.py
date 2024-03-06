#!/usr/bin/env python3

import rospy
import cv2 as cv
import math 
import sys

from ultralytics import YOLO

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class ROSCam_ObjectDetection:

    def __init__(self):

        rospy.init_node('detect_1')
        rospy.loginfo("YOLO Detection node has begun")

        self.rate = 10.0
        self.Q_size = 10.0

        self.bridge = CvBridge()
        self.error_bridge = CvBridgeError()

        self.font = cv.FONT_HERSHEY_SIMPLEX 
        self.fontScale = 1
        self.color = (255, 255, 255)
        self.thickness = 2

        self.classNames = ["person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat",
              "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
              "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
              "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat",
              "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
              "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli",
              "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant", "bed",
              "diningtable", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone",
              "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
              "teddy bear", "hair drier", "toothbrush"
              ]
        self.model = YOLO("yolo-Weights/yolov8n.pt")
        rospy.Subscriber('/camera/rgb/image_raw',Image, self.cam_compute)
        object_detected_pub = rospy.Publisher('/camera/yolo/rgb/image/raw', Image,queue_size=self.Q_size)

    def cam_compute(self, msg):
        try:
            self.raw_to_cv = self.bridge.imgmsg_to_cv2(img_msg=msg, desired_encoding="bgr8")
        except self.error_bridge as e:
            print (e)

        model_result = self.model(self.raw_to_cv, stream=True)
        
        for r in model_result:
            boxes = r.boxes 
            for box in boxes:

                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int (x1), int(y1), int(x2), int(y2)

                cv.rectangle (self.raw_to_cv, (x1, y1), (x2, y2), (5, 5, 5), 3)

                confidence = math.ceil((box.conf[0]*100))/100
                print("Confidence --->",confidence)

                cls = int(box.cls[0])
                print("Class name -->", self.classNames[cls])

                org = [x1, y1]
                cv.putText(self.raw_to_cv, self.classNames[cls], org, self.font, self.font, self.color, self.thickness)
        cv.imshow('Turtlebot3 Camera', self.raw_to_cv)
        if cv.waitKey(1) == ord('q'):
            cv.destroyAllWindows()

def main(args):
  ic = ROSCam_ObjectDetection()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv.destroyAllWindows()

if __name__=='__main__':
    main(sys.argv)