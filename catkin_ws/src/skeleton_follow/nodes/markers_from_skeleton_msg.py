#!/usr/bin/env python

"""
    Publish a skeleton message as a list of visualization markers for RViz
        
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2011 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import rospy
from skeleton_follow.msg import Skeleton
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import os
import pyautogui
import time

class SkeletonMarkers():
    def __init__(self):
        rospy.init_node('skeleton_markers')
        
        rospy.on_shutdown(self.shutdown)
        
        rospy.loginfo("Initializing Skeleton Markers Node...")
        screenWidth, screenHeight = pyautogui.size()
        #pyautogui.moveTo(screenWidth / 2, screenHeight / 2)
        print screenWidth, screenHeight
        for i in range(10):
            currentMouseX, currentMouseY = pyautogui.position()
            pyautogui.moveTo(currentMouseX+2, currentMouseY+2)

        self.rate = rospy.get_param('~rate', 20)
        self.scale = rospy.get_param('~scale', 0.07)
        self.lifetime = rospy.get_param('~lifetime', 0) # 0 is forever
        self.ns = rospy.get_param('~ns', 'skeleton_markers')
        self.id = rospy.get_param('~id', 0)
        self.color = rospy.get_param('~color', {'r': 0.0, 'g': 1.0, 'b': 0.0, 'a': 1.0})

        rate = rospy.Rate(self.rate)
        
        # Subscribe to the skeleton topic.
        rospy.Subscriber('skeleton', Skeleton, self.skeleton_handler,queue_size=10)
        
        # Define a marker publisher.
        self.marker_pub = rospy.Publisher('skeleton_markers', Marker, queue_size=10)
        
        # Initialize the marker points list.
        self.markers = Marker()
        self.markers.ns = self.ns
        self.markers.id = self.id
        self.markers.type = Marker.POINTS
        self.markers.action = Marker.ADD
        self.markers.lifetime = rospy.Duration(self.lifetime)
        self.markers.scale.x = self.scale
        self.markers.scale.y = self.scale
        self.markers.color.r = self.color['r']
        self.markers.color.g = self.color['g']
        self.markers.color.b = self.color['b']
        self.markers.color.a = self.color['a']
        self.button_up_flag = 0
        self.button_down_flag = 0 
        self.button_left_flag = 0
        self.button_right_flag = 0
        self.head_position = Point()
        self.lase_press_esc_time = rospy.Time.now()
        self.lase_press_enter_time = rospy.Time.now()       
        while not rospy.is_shutdown():
            self.marker_pub.publish(self.markers)                        
            rate.sleep()
    def supertux2_control(self, msg):
        left_hand_pose = msg.handpose[0]
        right_hand_pose = msg.handpose[1]
        control_flag = 0
        if left_hand_pose == 1 and right_hand_pose == 1:
            print "left and right ok"
            control_flag = 1
        elif right_hand_pose == 1:
            print "right_hand ok"
        elif left_hand_pose == 1:
            print "left hand ok"
        else:
            #pyautogui.keyUp('up')
            #pyautogui.keyUp('down')
            #pyautogui.keyUp('left')
            #pyautogui.keyUp('right')
            print "hand not get"

        for joint in msg.name:      
            position = Point()
            position = msg.position[msg.name.index(joint)]
            image_2d = msg.image_2d[msg.name.index(joint)]
            if joint == "head":
                self.head_position = position
                rpy = msg.rpy[msg.name.index(joint)]
                print "head:"
                print rpy
                if control_flag == 1:
                    if rpy.x > 0.3:
                        if self.button_down_flag == 1:
                            pyautogui.keyUp('down')
                            self.button_down_flag = 0
                        if self.button_up_flag == 0:
                            pyautogui.keyDown('up')
                            self.button_up_flag = 1
                    elif rpy.x < -0.4:
                        if self.button_up_flag == 1:
                            pyautogui.keyUp('up')
                            self.button_up_flag = 0
                        if self.button_down_flag == 0:
                            pyautogui.keyDown('down')
                            self.button_down_flag = 1
                    else:
                        if self.button_up_flag == 1:
                            pyautogui.keyUp("up")
                            self.button_up_flag = 0
                        if self.button_down_flag == 1:
                            pyautogui.keyUp("down")
                            self.button_down_flag = 0

                    if rpy.z > 0.15:
                        if self.button_left_flag == 1:
                            pyautogui.keyUp('left')
                            self.button_left_flag = 0
                        if self.button_right_flag == 0:
                            pyautogui.keyDown('right')
                            self.button_right_flag = 1
                    elif rpy.z < -0.2:
                        if self.button_right_flag == 1:
                            pyautogui.keyUp('right')
                            self.button_right_flag = 0
                        if self.button_left_flag == 0:
                            pyautogui.keyDown('left')
                            self.button_left_flag = 1
                    else:
                        if self.button_right_flag == 1:
                            pyautogui.keyUp("right")
                            self.button_right_flag = 0
                        if self.button_left_flag == 1:
                            pyautogui.keyUp("left")
                            self.button_left_flag = 0
            if control_flag == 0:    
                if joint == "left_hand":
                    print "left_hand:"
                    #print position
                    distance =  self.head_position.z - position.z
                    print distance
                    if distance > 500 and left_hand_pose == 1:
                        pyautogui.press('esc')
                        self.lase_press_esc_time = rospy.Time.now()
                    #print image_2d
                if joint == "right_hand":
                    print "right_hand:"
                    distance =  self.head_position.z - position.z
                    print distance
                    if distance > 500 and right_hand_pose == 1:
                        pyautogui.press('enter')
                        self.lase_press_enter_time = rospy.Time.now()
                    #print image_2d

    def skeleton_handler(self, msg):
        #self.supertux2_control(self,msg)
        self.markers.header.frame_id = msg.header.frame_id
        self.markers.header.stamp = rospy.Time.now()
        self.markers.points = list()
        for joint in msg.name:      
            position = Point()
            position = msg.position[msg.name.index(joint)]
            self.markers.points.append(position)

    def shutdown(self):
        rospy.loginfo('Shutting down Skeleton Marker Node.')
        
if __name__ == '__main__':
    try:
        SkeletonMarkers()
    except rospy.ROSInterruptException:
        pass
