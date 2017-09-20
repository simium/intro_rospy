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


class face_draw:

    def __init__(self):
        self.bridge = CvBridge()
        
        self.image_pub = rospy.Publisher("image_draw", Image, queue_size=10)

        self.image_sub = rospy.Subscriber("image_in",
                                          Image,
                                          self.image_capture_callback)
        self.face_sub = rospy.Subscriber("detected_faces",
                                         Int32MultiArray,
                                         self.face_detected_callback)

    def image_capture_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        (rows, cols, channels) = cv_image.shape
        if cols > 60 and rows > 60:
            font = cv2.FONT_HERSHEY_SIMPLEX
            bottomLeftCornerOfText = (64, 64)
            fontScale = 1
            fontColor = (255, 255, 255)
            lineType = 2

            cv2.putText(cv_image,
                        "Rectangle draw ROS node",
                        bottomLeftCornerOfText,
                        font, fontScale, fontColor, lineType)

        self.last_frame = cv_image.copy()

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

    def face_detected_callback(self, rect):
        x1 = rect.data[0]
        y1 = rect.data[1]
        x2 = rect.data[2]
        y2 = rect.data[3]

        green = (0, 255, 0)
        # Draw rectangle in duplicated image
        cv2.rectangle(self.last_frame, (x1, y1), (x2+x1, y2+y1), green, 2)
        cv2.imshow("Rectangle draw window", self.last_frame)
        cv2.waitKey(3)


def main(args):
    ic = face_draw()
    rospy.init_node('face_draw', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
        main(sys.argv)
