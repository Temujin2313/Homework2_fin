#!/usr/bin/env python3
# This is the listener node and will subscribe to the chatter topic


#from my_msgs.msg import Floats
#from rospy.numpy_msg import numpy_msg
import rospy
from homework2 .msg import TurtleVel 
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt

class translate:
    
    def __init__(self):
            rospy.loginfo("I got here")
            self.velocity_sub = rospy.Subscriber('/killian/turtle_cmd', TurtleVel, self.my_charter_callback) # Subscriber object)
            self.velocity_pub = rospy.Publisher('/killian/cmd_vel',Twist, queue_size=10)
            self.rate= rospy.Rate(20)

    def my_charter_callback(self, message):

        twist = Twist()
        twist.linear.x = message.linear
        twist.angular.z = message.angular
        self.velocity_pub.publish(twist)
        self.rate.sleep()

def main():
    rospy.init_node('translate_node')
    tr = translate()
    rospy.spin() # Program will enter the listening mode and execute the call back function

if __name__ == '__main__':
    main()