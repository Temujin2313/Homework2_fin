#!/usr/bin/env python3

from distutils.debug import DEBUG

import rospy
from turtlesim.srv import Spawn, Kill
from turtlesim.msg import Pose
from homework2 .msg import TurtleVel 
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from numpy import random
import time
import math
from homework2.srv import Draw, DrawRequest, DrawResponse

class follow:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.theta = 0
        self.kill = rospy.ServiceProxy("/kill", Kill)
        self.draw_waypoints = rospy.ServiceProxy("draw", Draw)
        self.waypoint = rospy.get_param("waypoint")
        self.waypoint_list  = self.waypoint['waypoints']
        self.pos_sub = rospy.Subscriber("/killian/pose", Pose, self.pose_sub)
        self.vel_pub = rospy.Publisher("killian/turtle_cmd", TurtleVel, queue_size=10)
        self.rate = rospy.Rate(20)

    def spawn_turtle(self):
        rospy.wait_for_service("/spawn")
        self.kill('turtle1')
        self.spawn_turtle = rospy.ServiceProxy("/spawn", Spawn)
        name = self.spawn_turtle(5.45555,5.45555,0,"killian")
        return name

    def call_draw_service(self):
        
        self.draw_waypoints()

    def goto_target(self):
        
        waypoint_counter = 0
        kp = .5
        k_angle = 3
        velocity_message = TurtleVel()

        while True:
            
            target_cord = self.waypoint_list[waypoint_counter]

            distance = abs((math.sqrt((target_cord[0]-self.x)**2 + (target_cord[1] - self.y)**2)))
            angle = math.atan2(target_cord[1]-self.y , target_cord[0] - self.x)
            
            velocity_message.linear = kp*distance
            velocity_message.angular = k_angle*(angle - self.theta)
            
            self.vel_pub.publish(velocity_message)

            if not (.2 < distance):
                rospy.loginfo("Reached {}".format(waypoint_counter))
                waypoint_counter += 1
            
            if waypoint_counter >= (len(self.waypoint_list)):
                rospy.loginfo("I got here")
                waypoint_counter = 0
            
            self.rate.sleep()

    def pose_sub(self, pos):
        self.x = pos.x
        self.y = pos.y
        self.theta = pos.theta
    
def main():
    f = follow()
    f.call_draw_service()
    name = f.spawn_turtle()
    f.goto_target()

if __name__ == '__main__':
    try:
        rospy.sleep(1)
        rospy.init_node('follow_node', log_level=DEBUG)
        main()
        rospy.spin()

    except rospy.ROSInitException:
        pass

 
    