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
from math import sqrt, modf
from rclpy.node import Node
from nav_msgs.msg import Odometry
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String

class PathExtractor(Node):
    def __init__(self):
        super().__init__("path_extractor")
        self.navigator = TurtleBot4Navigator("robot1")
        self.initial_pose = self.navigator.getPoseStamped([0.0, 0.0],TurtleBot4Directions.NORTH)
        self.navigator.setInitialPose(self.initial_pose)
        self.path_pub_sub = self.create_subscription(Odometry, "/robot1/odom", self.pub_callback,10)
        self.path_extractor = self.create_subscription(PoseWithCovarianceStamped, "/robot1/amcl_pose", self.path_callback,10)
        self.navigator.setInitialPose(self.initial_pose)
        self.path_publisher = self.create_publisher(String, "/robot1/path_pub",10)
        self.path = []
        self.path_as_string = String()

    def path_callback(self, msg):
        self.coordinate = (msg.pose.pose.position)
        if len(self.path) == 0 and abs(self.coordinate.x) >0.2:
            self.path.append(self.coordinate)
        
        if len(self.path) != 0:
            if self.L2_Norm(msg) >= 0.3:
                self.path.append(self.coordinate)
        
        self.path_as_string.data = ", ".join(map(str, self.path)) 
        #self.path_publisher.publish(self.path_as_string)
        self.get_logger().info(str(self.path_as_string))
        # self.get_logger().info(str(len(self.path)))
    def L2_Norm(self,msg):
        norm = sqrt((self.path[-1].x - msg.pose.pose.position.x)**2 + (self.path[-1].y - msg.pose.pose.position.y)**2)
        return norm
    
    def pub_callback(self,msg):
        #print('pubpub')
        self.path_publisher.publish(self.path_as_string)
        self.get_logger().info(str(self.path_as_string))



def main():
    rclpy.init()
    node = PathExtractor()
    rclpy.spin(node)

    navigator = TurtleBot4Navigator(name='robot2')
    print('NAV TURTL')

    # Start on dock
    if not navigator.getDockedStatus():
        print('get Status')
        navigator.info('Docking before intialising pose')
        navigator.dock()

    # Set initial pose
    initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    navigator.setInitialPose(initial_pose)
    print('BEFORE WAIT UNTIL NAV2ACTIVE')
     
    # Wait for Nav2
    navigator.waitUntilNav2Active()

    # Set goal poses
    goal_pose = navigator.getPoseStamped([2.0, 1.0], TurtleBot4Directions.EAST)

    # Undock
    navigator.undock()

    # Go to each goal pose
    navigator.startToPose(goal_pose)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
