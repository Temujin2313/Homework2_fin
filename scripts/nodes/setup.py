#!/usr/bin/env python3
"""This setup node is responsible for drawing all the waypoints for the turtle sim"""

# Import the standard ROS message used in the service

from std_msgs.msg import Empty
from turtlesim.msg import _Pose

# Import the custom service "draw" we created

from homework2.srv import Draw, DrawRequest, DrawResponse
from std_srvs.srv import Empty
from turtlesim.srv import Spawn, SpawnRequest, SetPen, TeleportAbsolute, Kill

# import standar "rospy" API
import rospy
import rosparam
import yaml
import os
import time


class setup:
    
    def __init__(self):
        # The service resets the turtle sim and  clears all drawing

        self.draw_waypoint_service = rospy.Service("draw", Draw, self.callback_service)


        self.waypoint = rospy.get_param("waypoint")
        self.waypoint_list  = self.waypoint['waypoints']
        
        # # Let's create service for teleport and setting the pen to draw
        rospy.wait_for_service("/turtle1/set_pen")
        rospy.wait_for_service("/turtle1/teleport_absolute")
        self.setting_pen = rospy.ServiceProxy('/turtle1/set_pen', SetPen)
        self.resT =rospy.ServiceProxy('/turtle1/teleport_absolute',TeleportAbsolute)

    def callback_service(self, req):
        rospy.wait_for_service('reset')
        clear_bg = rospy.ServiceProxy('reset', Empty)
        clear_bg()
        margin = 0.1
        for iter in range(len(self.waypoint_list)-1, -1, -1):
            point = self.waypoint_list[iter]
            self.setting_pen(0, 0 , 0, 1, 1)
            self.resT(point[0] + margin, point[1]+margin, 0)
            self.setting_pen(0, 0 , 0, 1, 0)
            self.resT(point[0]-margin, point[1]-margin, 0)
            self.setting_pen(0, 0 , 0, 1, 1)
            self.resT(point[0]+margin, point[1]-margin, 0)
            self.setting_pen(0, 0 , 0, 1, 0)
            self.resT(point[0]-margin, point[1]+margin, 0)
            self.resT(point[0], point[1], 0)
            rospy.sleep(.1)
        return DrawResponse()
    
    def call_service(self):
        self.draw_waypoints()

def main():
    rospy.init_node("draw_node", log_level=rospy.DEBUG)

    s = setup()
    time.sleep(1)
    rospy.spin()
    

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass


