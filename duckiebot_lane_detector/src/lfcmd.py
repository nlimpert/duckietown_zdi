#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Bool, Int64

from sensor_msgs.msg import CompressedImage, CameraInfo
from duckietown_msgs.msg import Twist2DStamped, WheelsCmdStamped

import numpy as np
import os
import cv2
from PIL import Image

#def on_low_H_thresh_trackbar(val):
#    global low_H
#    global high_H
#    low_H = val
#    low_H = min(high_H-1, low_H)
#    cv2.setTrackbarPos(low_H_name, window_detection_name, low_H)
#def on_high_H_thresh_trackbar(val):
#    global low_H
#    global high_H
#    high_H = val
#    high_H = max(high_H, low_H+1)
#    cv2.setTrackbarPos(high_H_name, window_detection_name, high_H)
#def on_low_S_thresh_trackbar(val):
#    global low_S
#    global high_S
#    low_S = val
#    low_S = min(high_S-1, low_S)
#    cv2.setTrackbarPos(low_S_name, window_detection_name, low_S)
#def on_high_S_thresh_trackbar(val):
#    global low_S
#    global high_S
#    high_S = val
#    high_S = max(high_S, low_S+1)
#    cv2.setTrackbarPos(high_S_name, window_detection_name, high_S)
#def on_low_V_thresh_trackbar(val):
#    global low_V
#    global high_V
#    low_V = val
#    low_V = min(high_V-1, low_V)
#    cv2.setTrackbarPos(low_V_name, window_detection_name, low_V)
#def on_high_V_thresh_trackbar(val):
#    global low_V
#    global high_V
#    high_V = val
#    high_V = max(high_V, low_V+1)
#    cv2.setTrackbarPos(high_V_name, window_detection_name, high_V)

max_value = 255
max_value_H = 360//2
low_H = 13
low_S = 0
low_V = 0
high_H = 30
high_S = 66
high_V = 255


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

        image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower = np.array([22, 93, 0], dtype="uint8")
        upper = np.array([45, 255, 255], dtype="uint8")
        mask = cv2.inRange(image, lower, upper)

        r_low = np.array([170, 100, 0], dtype="uint8")
        r_up = np.array([180, 255, 255], dtype="uint8")
        mask2 = cv2.inRange(image, r_low, r_up)

        red_obj_con = cv2.findContours(mask2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        red_obj_con = red_obj_con[0] if len(red_obj_con) == 2 else red_obj_con[1]


        cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        drive_dir = String()
        redline = Bool()
        midlane = Int64()
        redline.data = False
        x1 = 0

#        cv2.drawContours(image, cnts, -1, (0, 255, 0), 3) 
#
#        cv2.imshow('cv_img', image)
#        cv2.waitKey(2)

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





def main(args=None):
    rospy.init_node("lane_detector", anonymous=True)

    ld = LaneDetector()
    rospy.loginfo("Lane detector initialized")

if __name__ == '__main__':
    main()
