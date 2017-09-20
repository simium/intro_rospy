#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('intro_rospy')
import sys
import rospy
import cv2
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class face_detector:

    def __init__(self):
        self.image_pub = rospy.Publisher("image_out", Image, queue_size=10)
        self.face_pub = rospy.Publisher("detected_faces",
                                        Int32MultiArray, queue_size=10)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("image_in", Image, self.image_capture_callback)

        # self.face_detect = cv2.CascadeClassifier("/tmp/haarcascade_frontalface_default.xml")
        self.face_detect = cv2.CascadeClassifier(rospy.get_param("cascade_xml_file"))

    def image_capture_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        (rows, cols, channels) = cv_image.shape
        if cols > 0 and rows > 0:
            font = cv2.FONT_HERSHEY_SIMPLEX
            bottomLeftCornerOfText = (64, 64)
            fontScale = 1
            fontColor = (255, 255, 255)
            lineType = 2

            cv2.putText(cv_image,
                        "Face detector ROS node",
                        bottomLeftCornerOfText,
                        font, fontScale, fontColor, lineType)

            # Convert frame to grayscale
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            # Detect faces as rectangles
            rects = self.face_detect.detectMultiScale(gray, scaleFactor=1.3,
                                                      minNeighbors=4, minSize=(30, 30),
                                                      flags=cv2.CASCADE_SCALE_IMAGE)
            # For each detected face...
            for x1, y1, x2, y2 in rects:
                # Draw rectangle in duplicated image
                face_data = Int32MultiArray()
                face_data.data = [x1, y1, x2, y2]
                self.face_pub.publish(face_data)

            cv2.imshow("Face detector window", cv_image)
            cv2.waitKey(3)

            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            except CvBridgeError as e:
                print(e)


def main(args):
    ic = face_detector()
    rospy.init_node("face_detector", anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
        main(sys.argv)
