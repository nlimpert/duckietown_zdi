#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from duckietown_msgs.msg import WheelsCmdStamped

class DuckiebotTeleop:
   def __init__(self):
     rospy.Subscriber("cmd_vel", Twist, self.callback)
     self.pub = rospy.Publisher("/duckie1/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size=10)

     rospy.spin()

   def callback(self, data):
      rospy.loginfo("V: %f", data.linear.x)
      rospy.loginfo("Omega: %f", data.angular.z)

      wheels = WheelsCmdStamped()
      wheels.vel_left  = data.linear.x / 10
      wheels.vel_right = data.linear.x / 10
      self.pub.publish(wheels)

if __name__ == '__main__':
   rospy.init_node("duckiebot_teleop", anonymous=True)
   DuckiebotTeleop()
