#!/usr/bin/python3

# This code takes in encoder data and uses dead reckoning to deduce the position of the robot (with covariance).
# This is packed into an odom msg and sent to /odom

# Inputs : enc counts from Tiva. Class FakeTiva made for testing via rostopic.
# Expected input format: timegap for which counts are measured, left count, right count
# Parameters: Vehicle constants wheel separation, wheel radius, pulses per rev etc. defined in init of TivaOutput
# Arbitrary constants: r_thresh r_tolerance kl kr
# Output robot pose and pose covar packaged in odom

import math
import time
import numpy as np
import rospy
import queue

from geometry_msgs.msg import Pose, Quaternion, Point
from nav_msgs.msg import Odometry
from virat_msgs.msg import WheelVel
from tf.transformations import quaternion_from_euler
from vajra_controller_tiva.msg import enc_pulses


def sgn(x):
    if x > 1:
        return 1
    elif x == 0:
        return 0
    else:
        return -1


class OdomOutput:
    # Defining Vehicle and encoder constants
    wheel_dist: float = 2.52  # 2.5 is theoretical, 0.86 is real.
    wheel_radius: float = 0.184

    _pulse_per_rev: int = 1024
    gear_ratio = 5 / 4
    _counts_per_rev: int = _pulse_per_rev * 4 * gear_ratio
    # This is the number of encoder counts corresponding to one revolution of the tyre.
    # A pulse is a square wave pulse.
    # Each pulse triggers 4 counts.
    # 1024 pulses constitute one revolution of the encoder.

    _flip_motor_channels: bool = False

    # variance of left wheel measurements, sigma**2 = kl * |Distance travelled by left wheel|
    # Similar thing for right wheel
    # NOTE: The paper from which I got the covariance matrix calls what I call kl as kl**2.
    # Link to paper: https://www.cs.cmu.edu/~motionplanning/papers/sbp_papers/kalman/chong_accurate_odometry_error.pdf
    # https://ecse.monash.edu/techrep/reports/pre-2003/MECSE-6-1996.pdf

    # TODO: Tune these to reasonable values. They are presently arbitrary.
    kl = 0.00001
    kr = 0.00001

    def __init__(self, odom_topic: str = 'odom_new', wheel_vel_topic: str = "wheel_vel", rate: int = 50):

        self.odom_topic = odom_topic
        self.wheel_vel_topic = wheel_vel_topic

        self.rate = rate

        self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=1)
        self.vel_pub = rospy.Publisher(self.wheel_vel_topic, WheelVel, queue_size=1)
        self.enc_queue = queue.Queue()
        self.first_empty = 0

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

        # TODO: If the radius of curvature is very large, we consider the path to be a straight line.
        #  (this avoids heavy computation.) determine a reasonable threshold for this.
        # This number is called r_thresh and is presently defined arbitrarily to be 10 kilometers.
        self.r_thresh = 10000

        # Whenever we are checking if the radius of curvature has changed, we cannot expect a perfect equality.
        # If the r is within the tolerance from r old, they are considered equal.
        # TODO: Determine a reasonable number for r_tolerance
        self.r_tolerance = 10 ** (-5)

    def enc_callback(self, data):
        # Queues up the encoder data as and when it arrives. Main loop runs on a different speed from the encoder data.
        self.enc_queue.put(data)

    def update(self):
        # Consumes Encoder callback queue. Takes last 10 if more than 10 piled up.
        # updates the vehicle's position
        # updates pose covariance using update_covariance
        # CONVENTION: 1 indicates right and 2 indicates left

        time_thresh = 0.5
        # no of seconds we can get no data before we start throwing warnings.

        dt = 0  # As counted on the TIVA
        n1tot = 0
        n2tot = 0
        num = min(10, self.enc_queue.qsize())

        if self.enc_queue.empty():
            if self.first_empty != 0:
                last_seen = time.time() - self.first_empty
                if last_seen > time_thresh:
                    rospy.logerr(f"Haven't received encoder data in {last_seen}s. Still Waiting.")
                    self.first_empty = 0
                num = 0
            else:
                self.first_empty = time.time()
                num = 0
        else:
            self.first_empty = 0

        if self.enc_queue.qsize() > 20:
            rospy.logwarn(f"Can't keep up with data! {self.enc_queue.qsize()} entries have piled up!")

        for i in range(num):

            # n is the number of counts
            data = self.enc_queue.get()
            n1, n2 = data.enc0, data.enc1

            if dt == 0:
                prev_time = data.Header.stamp.secs + data.Header.stamp.nsecs * (10 ** -9)
                dt += -1
            else:
                dt += 1
                new_time = data.Header.stamp.secs + data.Header.stamp.nsecs * (10 ** -9)
                dt += new_time - prev_time
                prev_time = new_time

            n1tot += n1
            n2tot += n2
            if self._flip_motor_channels:
                n2, n1 = n1, n2
            self.update_pose(n1, n2)

        if dt != 0:
            self.vr = n1tot / self._counts_per_rev * 60 / dt
            self.vl = n2tot / self._counts_per_rev * 60 / dt
            self.dt = int(dt * 1000)

    def update_pose(self, n1, n2):

        # th is theta, i.e. the angle the wheel rotates by
        th1, th2 = 2 * math.pi * n1 / self._counts_per_rev, 2 * math.pi * n2 / self._counts_per_rev

        # Update pose
        # s is the distance moved by each wheel
        s1, s2 = th1 * self.wheel_radius, th2 * self.wheel_radius

        self.x += ((s1 + s2) / 2) * math.cos(self.yaw)
        self.y += ((s1 + s2) / 2) * math.sin(self.yaw)
        self.yaw += (s1 - s2) / self.wheel_dist

        # Update Covariance parameters
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

        self.update_covariance()

    # Accessed by update for the covariance matrix. Separated for modularity.
    def update_covariance(self):

        # Updating matrix A to suit current state
        self.A[0, 2] = self.r * (math.cos(self.theta0) - math.cos(self.yaw))
        self.A[1, 2] = self.r * (math.sin(self.theta0) - math.sin(self.yaw))

        # Defining constants for calculations
        c1 = self.kr * abs(self.r_dist) * self.l_dist + self.kl * abs(self.l_dist) * self.r_dist
        c2 = self.kr * abs(self.r_dist) + self.kl * abs(self.l_dist)
        c3 = self.kr * self.l_dist ** 2 * abs(self.r_dist) + self.kl * self.r_dist ** 2 * abs(self.l_dist)
        U = np.zeros((3, 3))

        # Computing sin and cos of yaw to avoid recomputation.

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

            U[0, 0] = -coeff1 * (math.sin(self.theta0) - math.sin(self.yaw)) * cos_yaw + (
                        (self.r * cos_yaw / self.wheel_dist) ** 2) * c2 - (coeff3 / 4) * (
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

    # Main Loop
    def run(self):

        r = rospy.Rate(self.rate)
        msg = Odometry()
        msg.header.frame_id = "map"

        while not rospy.is_shutdown():

            self.update()

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


def main():
    rospy.init_node('enc_odom')
    enc_to_odom = OdomOutput()
    rospy.Subscriber("/enc_pulses", enc_pulses, enc_to_odom.enc_callback)
    rospy.loginfo("I am now alive.")
    try:
        enc_to_odom.run()
    except Exception as e:
        print(e)


if __name__ == "__main__":
    main()
