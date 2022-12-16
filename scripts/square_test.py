#!/usr/bin/env python3

import rospy
import math
from vajra_controller_tiva.msg import enc_pulses
from std_msgs.msg import Float64


class SquareTest:
    wheel_dist: float = 0.86
    wheel_radius: float = 0.184
    _pulse_per_rev: int = 1024
    gear_ratio = 5 / 4
    _counts_per_rev: int = _pulse_per_rev * 4 * gear_ratio

    def count_to_dist(self, count):
        return 2 * math.pi * self.wheel_radius * count / self._counts_per_rev

    def __init__(self, side, speed=20, orientation='cw'):
        self.side = side
        self.speed = speed
        self.count = 0
        self.angle=0
        self.turns = 0
        if orientation == "acw":
            self.speed = -self.speed

    def run(self):
        rospy.init_node('vajra_teleop_new')
        publ = rospy.Publisher('left_wheel/setpoint', Float64)
        pubr = rospy.Publisher('right_wheel/setpoint', Float64)

        while not rospy.is_shutdown() and self.turns < 4:
            distance = self.count_to_dist(self.count)
            if distance <= self.side:
                publ.publish(abs(self.speed))
                pubr.publish(abs(self.speed))
            else:
                self.turns += 1
                while self.angle <= math.pi / 2:
                    publ.publish(self.speed)
                    pubr.publish(-self.speed)
                self.angle = 0
                self.count = 0

    def callback(self, data):
        self.count += (data.enc0 + data.enc1) / 2
        s1 = self.count_to_dist(data.enc0)
        s2 = self.count_to_dist(data.enc1)
        self.angle += (s1 - s2) / self.wheel_dist


if __name__ == "__main__":
    try:
        test = SquareTest(1.5)
        rospy.Subscriber("/enc_pulses", enc_pulses, test.callback)
        test.run()
    except Exception as e:
        print(e)
