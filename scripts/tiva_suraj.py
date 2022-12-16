#! /usr/bin/python3

#This is the original code used on the vehicle. The commented and covariance added version is in vajra_odom.
#This needs to be referred to to integrate the vajra_odom code which presently only uses a FakeTiva class to test.

import math
import time

import rospy
import serial
from geometry_msgs.msg import Pose, Quaternion, Point
from virat_msgs.msg import WheelVel
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler


class TivaOutput:
    wheel_dist: float = 0.67
    wheel_radius: float = 0.12
    rotation_ratio: int = 1024 * 4

    _pulse_per_rev: int = 1024 * 66
    _counts_per_rev: int = _pulse_per_rev * 4

    _flip_motor_channels: bool = False

    def __init__(self, tiva, odom_topic: str = 'odom', wheel_vel_topic: str = "wheel_vel", rate: int = 50,
                 ):

        self.odom_topic = odom_topic
        self.wheel_vel_topic = wheel_vel_topic

        self.rate = rate

        self.tiva = tiva
        time.sleep(0.2)
        self.tiva.readline()

        rospy.init_node('tiva_ros_driver')

        self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=1)
        self.vel_pub = rospy.Publisher(self.wheel_vel_topic, WheelVel, queue_size=1)

        self.odom: Odometry = Odometry()
        self.odom.header.frame_id = "odom"
        self.odom.child_frame_id = "base_footprint"
        self.odom.pose.covariance[0] = 0.1
        self.odom.pose.covariance[1 + 6] = 0.1
        self.odom.pose.covariance[2 + 12] = 0.1
        self.odom.pose.covariance[5 + 30] = 0.1

        self.vel: WheelVel = WheelVel()
        self.vel.header.frame_id = "wheels"

        self.x: float = 0.0
        self.y: float = 0.0
        self.yaw: float = 0.0

        self.vr = 0
        self.vl = 0
        self.dt = 0

    def update(self, dt):
        right = 0
        left = 0
        dt = 0
        for line in self.tiva.readlines():
            # print(">>>", line)
            try:
                # TODO: Check for overflow
                ddt, n1, n2 = map(int, line.decode('ASCII').strip().split(' '))
            except ValueError:
                # print("VALUE ERROR")  # TODO: readlines is returning partial lines :ugh: fix.
                # print(line)
                continue

            dt += ddt
            if self._flip_motor_channels:
                n2, n1 = n1, n2

            th1, th2 = 2 * math.pi * n1 / self._counts_per_rev, 2 * math.pi * n2 / self._counts_per_rev

            # Update ODOM
            s1, s2 = th1 * self.wheel_radius, th2 * self.wheel_radius

            self.x += ((s1 + s2) / 2) * math.cos(self.yaw)
            self.y += ((s1 + s2) / 2) * math.sin(self.yaw)
            self.yaw += (s1 - s2) / self.wheel_dist

            # Update Velocity
            right += n1
            left += n2

        if dt != 0:
            self.vr = right / self._counts_per_rev * 60 * 1000 / dt
            self.vl = left / self._counts_per_rev * 60 * 1000 / dt
            self.dt = dt
        print(f"{dt}\t{right:.4f}\t{left:.4f}\t{self.vr:.4f}\t{self.vl:.4f}")

    def run(self):
        r = rospy.Rate(self.rate)
        msg = Odometry()
        msg.header.frame_id = "map"

        prev_time = time.time()
        while not rospy.is_shutdown():
            cur_time = time.time()
            self.update(cur_time - prev_time)
            prev_time = cur_time

            self.vel.header.stamp = rospy.Time.now()
            self.vel.right = self.vr
            self.vel.left = self.vl
            self.vel.dt = self.dt
            self.vel_pub.publish(self.vel)

            self.odom.header.stamp = rospy.Time.now()
            self.odom.pose.pose = Pose(Point(self.x, self.y, 0.0), Quaternion(*quaternion_from_euler(0, 0, self.yaw)))
            self.odom_pub.publish(self.odom)

            r.sleep()


def main():
    port: str = '/dev/serial/by-id/usb-Texas_Instruments_In-Circuit_Debug_Interface_0E2353E3-if00'
    baud: int = 19200
    with serial.Serial(port, baud, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                                  bytesize=serial.EIGHTBITS, timeout=0) as tiva:
        tiva.write(b'A1\n')
        try:
            tout = TivaOutput(tiva)
            tout.run()
        except Exception as e:
            pass
        finally:
            tiva.write(b'A0\n')


if __name__ == "__main__":
    main()
