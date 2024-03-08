#!/usr/bin/env python3

# Copyright 2022 Clearpath Robotics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

import rclpy
from rclpy.node import Node
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
from std_msgs.msg import String
import math
import re


class Follower_Navigator(Node):
    def __init__(self):
        super().__init__("follower_navigator")
        #subscibing to leader path array publisher
        self.path_follower = self.create_subscription(String, "/robot1/path_pub", self.follower_callback,10)
        self.index = 0 #to keep track of followers currently achieved goal coordinate in path_array

        self.navigator = TurtleBot4Navigator("robot2")

        #Start on dock
        if not self.navigator.getDockedStatus():
            self.navigator.info('Docking before intialising pose')
            self.navigator.dock()

        # Set initial pose
        initial_pose = self.navigator.getPoseStamped([0.0, 0.5], TurtleBot4Directions.NORTH)
        self.navigator.clearAllCostmaps()
        self.navigator.setInitialPose(initial_pose)
        print('INITIAL POSE DONE')
        # Wait for Nav2
        self.navigator.waitUntilNav2Active()
        self.navigator.undock()
        
    def follower_callback(self,msg):
        self.path_array =[]
        self.coord_extractor(msg.data)
        #if index is not at last coordinate and Follower is not already chasing a goal we assign next goal from array
        if len(self.path_array) > self.index and self.navigator.isTaskComplete():
            if self.index>0:
                #calculating angle between current position and next goal position
                self.theta=self.calculate_angle(self.path_array[self.index-1][0],self.path_array[self.index-1][1],self.path_array[self.index][0],self.path_array[self.index][1])
            else:
                self.theta = TurtleBot4Directions.SOUTH
            self.get_logger().info(str(self.path_array))
            self.navigator.startThroughPoses([self.navigator.getPoseStamped([self.path_array[self.index][0],self.path_array[self.index][1]], self.theta)])
            self.index += 1

    def calculate_angle(self,x1, y1, x2, y2):
        self.angle_radians = math.atan2(y2 - y1, x2 - x1)
        self.angle_degrees = math.degrees(self.angle_radians)
        return self.angle_degrees

#function to extracting coordinates from String() object recieved from /robot1/path_pub
    def coord_extractor(self, data):
        input_string = data
        object_strs = input_string.split('), ')
        points = []
        for obj_str in object_strs:
            # Use regular expression to extract the values
            matches = re.findall(r'x=([-\d.e]+), y=([-\d.e]+), z=([-\d.e]+)', obj_str)
            if matches:
                x, y, z = matches[0]
                x, y, z = float(x), float(y), float(z)
                self.path_array.append((x,y,z))



def main():
    rclpy.init()
    node = Follower_Navigator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
