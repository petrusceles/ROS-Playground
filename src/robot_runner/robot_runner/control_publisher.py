# Copyright 2016 Open Source Robotics Foundation, Inc.
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

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
import math
from tf_transformations import euler_from_quaternion


class ControlPublisher(Node):
    def __init__(self):
        super().__init__('movement')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 100)
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.listener_callback,
            10)
        self.c = 0.0
        # self.curr = [0,0,0]
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        self.i += 1

    def pose_publish(self,v_,w_):
        msg = Twist()
        msg.linear.x = v_
        msg.angular.z = w_
        self.publisher_.publish(msg)
        self.i += 1

    def listener_callback(self, msg):
        robot_pose = msg.pose.pose.position
        robot_o = (msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w)
        robot_eu = euler_from_quaternion(robot_o)
        self.calculate_movement(robot_pose,robot_eu)

    def calculate_movement(self, robot_pose_, robot_eu_):
        curr = [robot_pose_.x, robot_pose_.y, robot_eu_[2]]
        temp = self.generate_position()
        th_dest = math.atan2((temp[1]-curr[1]),(temp[0]-curr[0]))
        dest = [temp[0],temp[1],th_dest]

        omg = 0.0
        vel = 0.0

        err = []
        for i in range(len(curr)):
            err.append(math.fabs(dest[i]-curr[i]))

        k = 2

        if curr[2] < -math.pi/2 and dest[2] > math.pi/2: 
            omg = -k*(math.pi*2 + curr[2]-dest[2])
        elif dest[2] < -math.pi/2 and curr[2] > math.pi/2:
            omg = k*(math.pi*2 + dest[2]-curr[2])
        else :
            omg = k*(dest[2]-curr[2])

        vel = math.sqrt((dest[0]-curr[0])**2 + (dest[1]-curr[1])**2)

        if err[0] < 0.08 and err[1] < 0.08 and err[2] < 0.1:
            self.c+=0.1
            if self.c >= math.pi*2:
                self.c = 0.0

        self.pose_publish(vel,omg)

    def generate_position(self):
        x = math.cos(self.c)/(1+math.sin(self.c)**2)
        y = 1.4*math.sin(self.c)*math.cos(self.c)/(1+math.sin(self.c)**2)
        # print("C     : ", self.c)
        dest = [x,y]
        return dest



def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = ControlPublisher()
    # print(robot_pose)

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
