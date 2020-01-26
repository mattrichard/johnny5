# Copyright 2020 Matt Richard
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

from math import pi, sqrt
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class JoyTeleop(Node):

    def __init__(self):
        super().__init__('johnny5_joy_teleop')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.pub_callback)

        self.x = 0
        self.y = 0

    def joy_callback(self, msg):
        self.x = msg.axes[3]
        self.y = msg.axes[1]

    def pub_callback(self):
        twist = Twist()
        twist.linear.x = min(max(self.y, -1.0), 1.0)
        twist.angular.z = self.x * pi

        self.publisher_.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    teleop = JoyTeleop()
    rclpy.spin(teleop)

    teleop.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
