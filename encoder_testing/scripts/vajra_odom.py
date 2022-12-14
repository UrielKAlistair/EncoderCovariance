#!/usr/bin/python3
# This code takes in encoder data and uses dead reckoning to deduce the position of the robot (with covariance).
# This is packed into ann odom msg and sent to /odom

import math
import time
import numpy as np
import rospy

from geometry_msgs.msg import Pose, Quaternion, Point
from virat_msgs.msg import WheelVel
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Float64


def sgn(x):
    if x > 1:
        return 1
    elif x == 0:
        return 0
    else:
        return -1


class TivaOutput:
    # Defining Vehicle and encoder constants
    wheel_dist: float = 0.67
    wheel_radius: float = 0.12
    rotation_ratio: int = 1024 * 4

    _pulse_per_rev: int = 1024 * 66
    _counts_per_rev: int = _pulse_per_rev * 4

    _flip_motor_channels: bool = False

    # variance of left wheel measurements, sigma**2 = kl * |Distance travelled by left wheel|
    # Similar thing for right wheel
    # NOTE: The paper from which I got the covariance matrix calls what I call kl as kl**2.
    # Link to paper: https://www.cs.cmu.edu/~motionplanning/papers/sbp_papers/kalman/chong_accurate_odometry_error.pdf
    # https://ecse.monash.edu/techrep/reports/pre-2003/MECSE-6-1996.pdf

    kl = 0.00001
    kr = 0.00001

    def __init__(self, tiva, odom_topic: str = 'odom', wheel_vel_topic: str = "wheel_vel", rate: int = 50):

        self.odom_topic = odom_topic
        self.wheel_vel_topic = wheel_vel_topic

        self.rate = rate

        self.tiva = tiva
        time.sleep(0.2)
        self.tiva.readline()

        self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=1)
        self.vel_pub = rospy.Publisher(self.wheel_vel_topic, WheelVel, queue_size=1)

        self.odom: Odometry = Odometry()
        self.odom.header.frame_id = "odom"
        self.odom.child_frame_id = "base_footprint"

        self.vel: WheelVel = WheelVel()
        self.vel.header.frame_id = "wheels"

        self.x: float = 0.0
        self.y: float = 0.0
        self.yaw: float = 0.0

        self.vr = 0
        self.vl = 0
        self.dt = 0

        # Covariance Propagation Variables

        # This matrix 'A' will be used to compute covariance. Key entries will be edited from I inside functions.
        self.A = np.identity(3)

        ##################################
        # Defining Zero State Parameters #
        ##################################

        # S0 is the matrix representing the covariance of the "zero state" of the robot.
        # S0 is updated in the update function to the previous state's covariance, every time the radius of curvature
        # changes and utilised later to generate the final covariance.
        self.S0 = np.zeros((3, 3))

        # this is the yaw of the vehicle in the zero state.
        self.theta0 = 0

        # This is the radius of curvature the vehicle has been in since the zero state.
        self.r_old = -1

        # This is the distance travelled by the left and right wheels, from the zero state
        self.l_dist = 0
        self.r_dist = 0

        #########################################
        # Defining Zero State Update Parameters #
        #########################################

        # This is the 3 by 3 covariance matrix which we will compute. It will be dumped into a 36 entry array for ros
        # odom later. This 3by3 covar is of the last published state, so it will be used to reset S0 when r changes.
        self.covar3by3 = np.zeros((3, 3))

        # This is the vehicle yaw when the covariance was last published.
        self.theta_last = 0

        # This is the radius of curvature of the path of the vehicle at this instant. -1 corresponds to a straight line.
        self.r = -1

        ###################################

        # TODO: Determine a reasonable number beyond which r can be set to -1, so that we can avoid heavy computation.
        # This number is called r_thresh and is defined arbitrarily by me to be 10 kilometers.
        self.r_thresh = 10000

        # Whenever we are checking if the radius of curvature has changed, we cannot expect a perfect equality.
        # If the r is within the tolerance from r old, they are considered equal.
        # TODO: Determine a reasonable number for r_tolerance
        self.r_tolerance = 10 ** (-5)

    def update(self, computer_dt: float):
        right = 0
        left = 0
        dt = 0  # As counted on the TIVA
        for line in self.tiva.readlines():
            # print(">>>", line)
            try:
                # n is the number of counts
                # TODO: Check for overflow
                # 1 indicates right and 2 indicates left
                ddt, n1, n2 = map(float, line.strip().split(' '))
            except ValueError:
                rospy.logerr(f"TIVA returned an invalid line of data: {line}")
                # print("VALUE ERROR")  # TODO: ```readlines``` is returning partial lines :ugh: fix.
                # print(line)
                continue

            dt += ddt
            if self._flip_motor_channels:
                n2, n1 = n1, n2

            # th is theta, i.e. the angle the wheel rotates by
            th1, th2 = 2 * math.pi * n1 / self._counts_per_rev, 2 * math.pi * n2 / self._counts_per_rev

            # Update ODOM
            # s is the distance moved by each wheel
            s1, s2 = th1 * self.wheel_radius, th2 * self.wheel_radius

            self.x += ((s1 + s2) / 2) * math.cos(self.yaw)
            self.y += ((s1 + s2) / 2) * math.sin(self.yaw)
            self.yaw += (s1 - s2) / self.wheel_dist

            # Update Velocity
            right += n1
            left += n2

            # Update Covariances
            if s1 != s2:
                self.r = (self.wheel_dist * (s1 + s2)) / ((s2 - s1) * 2)
                if self.r > self.r_thresh:
                    self.r = -1
            else:
                self.r = -1  # Have to handle straight line case separately

            # If r has changed, reset Zero State.
            if abs(self.r - self.r_old) > self.r_tolerance:  # r != rold , but with tolerance
                self.r_old = self.r
                self.theta0 = self.theta_last
                self.S0 = self.covar3by3
                self.r_dist = s1
                self.l_dist = s2

            self.r_dist += s1
            self.l_dist += s2

            self.get_covariance()

        if dt != 0:
            self.vr = right / self._counts_per_rev * 60 * 1000 / dt
            self.vl = left / self._counts_per_rev * 60 * 1000 / dt
            self.dt = dt

        # print(f"{dt}\t{right:.4f}\t{left:.4f}\t{self.vr:.4f}\t{self.vl:.4f}")

    def get_covariance(self):

        # Updating matrix A to suit current state
        self.A[0, 2] = self.r * (math.cos(self.theta0) - math.cos(self.yaw))
        self.A[1, 2] = self.r * (math.sin(self.theta0) - math.sin(self.yaw))

        # Defining constants for calculations
        c1 = self.kr * abs(self.r_dist) * self.l_dist + self.kl * abs(self.l_dist) * self.r_dist
        c2 = self.kr * abs(self.r_dist) + self.kl * abs(self.l_dist)
        c3 = self.kr * self.l_dist ** 2 * abs(self.r_dist) + self.kl * self.r_dist ** 2 * abs(self.l_dist)
        U = np.zeros((3, 3))

        # Computing sin and cos of yaw to avoid wasteful recomputation.

        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)

        ####################
        # Defining Matrix U#
        ####################

        if self.r == 0:
            coeff_r0_1 = (self.wheel_dist / 32) * sgn(self.yaw - self.theta0) * (self.kl + self.kr)
            U[0, 0] = coeff_r0_1 * (2 * (self.yaw - self.theta0) - math.sin(2 * self.theta0) + math.sin(2 * self.yaw))
            U[1, 1] = coeff_r0_1 * (2 * (self.yaw - self.theta0) + math.sin(2 * self.theta0) - math.sin(2 * self.yaw))
            U[2, 2] = abs(self.yaw - self.theta0) * (self.kl + self.kr) / (2 * self.wheel_dist)
            U[0, 1] = (math.cos(2 * self.theta0) - math.cos(2 * self.yaw)) * coeff_r0_1
            U[1, 0] = U[0, 1]
            U[0, 2] = sgn(self.yaw - self.theta0) * (sin_yaw - math.sin(self.theta0)) * (self.kr - self.kl) / 4
            U[2, 0] = U[0, 2]
            U[1, 2] = sgn(self.yaw - self.theta0) * (cos_yaw - math.cos(self.theta0)) * (self.kr - self.kl)
            U[2, 0] = U[0, 2]

        elif self.r == -1:

            ksdiff = self.kl - self.kr
            kssum = self.kl + self.kr
            dbyb = self.l_dist / self.wheel_dist
            dbybsq = (self.l_dist / self.wheel_dist) ** 2

            U[0, 0] = abs(self.l_dist) * (
                    (pow(cos_yaw, 2) / 4) * kssum + (dbyb / 2) * sin_yaw * math.cos(
                self.yaw) * ksdiff + (dbybsq / 3) * pow(sin_yaw, 2) * kssum)
            U[1, 1] = U[0, 0] - abs(self.l_dist) * dbyb * sin_yaw * cos_yaw * ksdiff
            U[2, 2] = (abs(self.l_dist) / pow(self.wheel_dist, 2)) * kssum
            U[0, 1] = abs(self.l_dist) * (((1 / 8) - dbybsq / 6) * math.sin(2 * self.yaw) * kssum + math.cos(
                2 * self.yaw) * dbyb * ksdiff / 4)
            U[1, 0] = U[0, 1]
            U[1, 2] = abs(self.l_dist) * (
                    cos_yaw * ksdiff / (2 * self.wheel_dist) - self.l_dist * sin_yaw * (
                    kssum / (2 * pow(self.wheel_dist, 2))))
            U[2, 1] = U[1, 2]
            U[2, 1] = abs(self.l_dist) * (
                    sin_yaw * (ksdiff / 2 * self.wheel_dist) - self.l_dist * cos_yaw * (
                    kssum / (2 * pow(self.wheel_dist, 2))))
            U[1, 2] = U[2, 1]

        else:
            lr2 = (self.l_dist - self.r_dist) ** 2
            lr3 = (self.l_dist - self.r_dist) ** 3
            coeff1 = 2 * self.r * c1 / lr2
            coeff2 = c2 / pow(self.wheel_dist, 2)
            coeff3 = (self.wheel_dist * c3 / lr3)

            U[0, 0] = -coeff1 * (math.sin(self.theta0) - math.sin(self.yaw)) * cos_yaw + pow(
                (self.r * cos_yaw / self.wheel_dist), 2) * c2 - (coeff3 / 4) * (
                              2 * (self.yaw - self.theta0) - math.sin(2 * self.theta0) + math.sin(2 * self.yaw))
            U[1, 1] = -coeff1 * (cos_yaw - math.cos(self.theta0)) * sin_yaw + pow(
                (self.r * sin_yaw / self.wheel_dist), 2) * c2 - (coeff3 / 4) * (
                              2 * (self.yaw - self.theta0) + math.sin(2 * self.theta0) - math.sin(2 * self.yaw))
            U[2, 2] = coeff2
            U[0, 1] = -(coeff1 / 2) * (math.cos(2 * self.yaw) - math.cos(self.yaw + self.theta0)) + (
                    c2 * self.r ** 2 * coeff2 / 2) * math.sin(2 * self.yaw) - (coeff3 / 4) * (
                              math.cos(2 * self.theta0) - math.cos(2 * self.yaw))
            U[1, 0] = U[0, 1]
            U[0, 2] = (c1 / lr2) * (math.sin(self.theta0) - math.sin(self.yaw)) - self.r * coeff2 * cos_yaw
            U[2, 0] = U[0, 2]
            U[1, 2] = (c1 / lr2) * (cos_yaw - math.cos(self.theta0)) - self.r * coeff2 * sin_yaw
            U[2, 1] = U[1, 2]

        #####################

        self.covar3by3 = U + self.A @ self.S0 @ self.A.T

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
            self.odom.pose.pose = Pose(Point(self.x, self.y, 0.0),
                                       Quaternion(*quaternion_from_euler(0, 0, self.yaw)))

            # Coverting 3x3 to 6x6 for ros
            roscovar = [0] * 36
            for i in range(3):
                roscovar[i] = self.covar3by3[0, i]
                roscovar[6 + i] = self.covar3by3[1, i]
                roscovar[30 + i] = self.covar3by3[2, i]

            self.odom.pose.covariance = roscovar

            self.odom_pub.publish(self.odom)

            r.sleep()


class FakeTiva:
    def __init__(self):
        self.told_l = time.time()
        self.told_r = time.time()
        self.del_t_l = 0
        self.del_t_r = 0
        self.left = 0
        self.right = 0

    def callback_left(self, data):

        now = time.time()
        self.del_t_l = now - self.told_l
        self.told_l=now
        self.left = data.data

    def callback_right(self, data):

        now = time.time()
        self.del_t_r = now - self.told_r
        self.told_r = now
        self.right = data.data

    def readline(self):
        pass

    def readlines(self):
        rospy.Subscriber("enc0_rpm", Float64, self.callback_right)
        rospy.Subscriber("enc1_rpm", Float64, self.callback_left)
        for i in range(10):
            yield f"{(self.del_t_l+self.del_t_r)/2} {self.left} {self.right}"
        return


def main():
    rospy.init_node('tiva_ros_driver')
    tiva = FakeTiva()
    try:
        tout = TivaOutput(tiva)
        tout.run()
    except Exception as e:
        print(e)
        pass


if __name__ == "__main__":
    main()
