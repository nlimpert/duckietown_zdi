#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Bool, Int64

from sensor_msgs.msg import CompressedImage, CameraInfo
from duckietown_msgs.msg import Twist2DStamped, WheelsCmdStamped

import numpy as np
import os
import cv2
from PIL import Image


class LaneDetector:

    def __init__(self):
        self.pub_drive_direction_ = rospy.Publisher("/Drive/Direction", String, queue_size=10)
        self.red_line_detected_ = rospy.Publisher("/RedLine/InFront", Bool, queue_size=10)
        self.mid_lane = rospy.Publisher("/MidLane", Int64, queue_size=10)
        #self.pub_drive_cmd_ = self.create_publisher(WheelsCmdStamped, '/None/wheels_driver_node/wheels_cmd', 10)
        self.sub_img_ = rospy.Subscriber("/None/corrected_image/compressed", CompressedImage, self.sub_img_cb, 10 )
        timer_period = 1.0  # seconds
        #self.timer = self.create_timer(timer_period, self.sub_img_cb)
        self.i = 0

        rospy.spin()

    def sub_img_cb(self, msg, info):
        np_arr = np.fromstring(msg.data, np.uint8)
        image  = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

#        image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        kernel = np.ones((5,5),np.float32)/25
        dst = cv2.filter2D(gray,-1,kernel)
        ret,thresh1 = cv2.threshold(dst,127,255,cv2.THRESH_BINARY)


        lower = np.array([22, 93, 0], dtype="uint8")
        upper = np.array([45, 255, 255], dtype="uint8")
        mask = cv2.inRange(image, lower, upper)

        r_low = np.array([170, 100, 0], dtype="uint8")
        r_up = np.array([180, 255, 255], dtype="uint8")
        mask2 = cv2.inRange(image, r_low, r_up)

        red_obj_con = cv2.findContours(mask2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        red_obj_con = red_obj_con[0] if len(red_obj_con) == 2 else red_obj_con[1]


        color = (0, 0, 0)
        thresh1 = cv2.rectangle(thresh1, (0,0), (thresh1.shape[1], thresh1.shape[0] * 2/3), color, -1)

#        cv2.imshow('cv_img', thresh1)
#        cv2.waitKey(1)

        cnts = cv2.findContours(thresh1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        drive_dir = String()
        redline = Bool()
        midlane = Int64()
        redline.data = False
        x1 = 0

        cv2.drawContours(image, cnts, -1, (0, 255, 0), 3)


        if len(cnts) != 0:
            x,y,w,h = cv2.boundingRect(cnts[0])
            cv2.rectangle(image,(x,y),(x+w,y+h), (255,0,0), 2)
            x1 = x
            midlane.data = x
            if y > 300:
                if x < 100:
                    drive_dir.data = "left"
                elif x > 200:
                    drive_dir.data = "right"
                else:
                    drive_dir.data = "forward"
        else:
            midlane.data = 0
        self.mid_lane.publish(midlane)
        self.pub_drive_direction_.publish(drive_dir)
        if len(red_obj_con) != 0:
            for red_obj in red_obj_con:
                x, y, w, h = cv2.boundingRect(red_obj)
                if h * w > 3000 and y > 320 and x1 < x:
                    redline.data = True
            self.i += 1
        self.red_line_detected_.publish(redline)

#        cv2.imshow('cv_img', image)
#        cv2.waitKey(1)




def main(args=None):
    rospy.init_node("lane_detector", anonymous=True)

    ld = LaneDetector()
    rospy.loginfo("Lane detector initialized")

if __name__ == '__main__':
    main()
