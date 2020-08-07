#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from duckietown_msgs.msg import Twist2DStamped

class DuckiebotTeleop:
   def __init__(self):
     rospy.Subscriber("cmd_vel", Twist, self.callback)
     self.pub = rospy.Publisher("/None/lane_controller_node/car_cmd", Twist2DStamped, queue_size=10)

     rospy.spin()

   def callback(self, data):
      rospy.loginfo("V: %f", data.linear.x)
      rospy.loginfo("Omega: %f", data.angular.z)

      twist = Twist2DStamped()
      twist.v     = data.linear.x  / 5
      twist.omega = data.angular.z * 1.5
      self.pub.publish(twist)

if __name__ == '__main__':
   rospy.init_node("duckiebot_teleop", anonymous=True)
   DuckiebotTeleop()
