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
from math import sqrt
from rclpy.node import Node
from nav_msgs.msg import Odometry
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
from std_msgs.msg import String

class PathExtractor(Node):
    def __init__(self):
        super().__init__("path_extractor")
        self.path_extractor = self.create_subscription(Odometry, "/robot1/odom", self.path_callback,10) 
        self.path_publisher = self.create_publisher(String, "/robot1/path_pub",10)
        self.path = []

        self.navigator = TurtleBot4Navigator("robot1")
        initial_pose = self.navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
        self.navigator.setInitialPose(initial_pose)

    def path_callback(self, msg):
        self.coordinate = (msg.pose.pose.position)
        #first coordinate is appended when leader is at some distance(0.2) away from dock
        if len(self.path) == 0 and (abs(self.coordinate.x) > 0.2 or abs(self.coordinate.y) > 0.2):
            self.path.append(self.coordinate)
        
        #if Euclidean distance between successive coordinates >= 0.5 then append next coordinate
        if len(self.path) != 0:
            if self.L2_Norm(msg) >= 0.5:
                self.path.append(self.coordinate)
        #publishing path_array as String() object
        self.path_as_string = String()
        self.path_as_string.data = ", ".join(map(str, self.path)) 
        self.path_publisher.publish(self.path_as_string)
        self.get_logger().info(str(self.path_as_string))
        # self.get_logger().info(str(len(self.path)))
    
    def L2_Norm(self,msg):
        # L2_norm = sqrt((x2-x1)^2 + (y2-y1)^2)
        norm = sqrt((self.path[-1].x - msg.pose.pose.position.x)**2 + (self.path[-1].y - msg.pose.pose.position.y)**2) 
        return norm



def main():
    rclpy.init()
    node = PathExtractor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
