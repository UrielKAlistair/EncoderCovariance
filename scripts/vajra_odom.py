#!/usr/bin/python3

# This code takes in encoder data and uses dead reckoning to deduce the position of the robot (with covariance).
# This is packed into an odom msg and sent to /odom (default)

# Inputs : enc counts from a rostopic.
# Expected input format: timegap for which counts are measured, left count, right count
# Parameters: Vehicle constants, wheel separation, wheel radius, pulses per rev etc. defined in init of TivaOutput
# , Covariance constants kl and kr
# Output: robot pose and pose covar packaged in odom

# Reference for Covariance calucations
# https://github.com/Sollimann/CleanIt/blob/main/autonomy/src/slam/README.md

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


class OdomOutput:
    # Defining Vehicle and encoder constants
    wheel_dist: float = 0.45  # scaled experimental data 0.5933988764044943 binary searched 0.4625
    wheel_radius: float = 0.262016
    # 8 inches is real. Expt gave 0.184

    _pulse_per_rev: int = 1024
    gear_ratio = 5 / 4
    _counts_per_rev: int = _pulse_per_rev * 4 * gear_ratio
    # This is the number of encoder counts corresponding to one revolution of the tyre.
    # A pulse is a square wave pulse.
    # Each pulse triggers 4 counts.
    # 1024 pulses constitute one revolution of the encoder.

    _flip_motor_channels: bool = False

    def __init__(self, odom_topic: str = 'enc_odom', wheel_vel_topic: str = "wheel_vel", rate: int = 50):

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

        self.vx: float = 0.0
        self.vy: float = 0.0
        self.wz: float = 0.0

        self.vr = 0
        self.vl = 0
        self.dt = 0

        self.covar3by3 = np.full(shape=(3, 3), fill_value=10 ** (-5))
        self.motion_cov = None
        self.pose_jacobian = None
        self.velocity_jacobian = None

        self.k = 10 ** (-5)
        self.k_right = self.k  # experimental constant for error in right wheel distance measurements
        self.k_left = self.k  # experimental constant for error in left wheel distance measurements

    def enc_callback(self, data):
        # Queues up the encoder data as and when it arrives. Main loop runs on a different speed from the encoder data.
        self.enc_queue.put(data)

    def update(self):
        # Consumes Encoder callback queue. Takes last 10 if more than 10 piled up.
        # updates the vehicle's position
        # updates pose covariance using update_covariance
        # CONVENTION: 1 indicates right and 2 indicates left

        time_thresh = 0.5
        # number of seconds we can get ghosted before we start throwing warnings.

        dt, ddt = 0, 0  # As counted on the TIVA
        n1tot = 0
        n2tot = 0
        num = min(10, self.enc_queue.qsize())

        if self.enc_queue.empty():

            # If the queue is empty, we check if this has happened recently. first_empty = 0 means not recent.
            # The first time it's empty, first_empty is set to the current time.
            # In every successive iteration, we check if any data has been received.
            # If we do get data, first_empty is reset to 0.
            # If we do not, an error is logged to ROS after a time_thresh. Error repeats in intervals of time_thresh.

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
            rospy.logwarn(f"enc_odom can't keep up with encoder data! {self.enc_queue.qsize()} entries have piled up!")

        first_time = True
        for i in range(num):

            # n is the number of counts
            data = self.enc_queue.get()
            n1, n2 = data.enc0, data.enc1

            # I'm taking dt in seconds
            if first_time is True:
                prev_time = data.Header.stamp.secs + data.Header.stamp.nsecs * (10 ** -9)
                first_time = False
            else:
                new_time = data.Header.stamp.secs + data.Header.stamp.nsecs * (10 ** -9)
                ddt = new_time - prev_time
                prev_time = new_time

            if self._flip_motor_channels:
                n2, n1 = n1, n2

            n1tot += n1
            n2tot += n2
            self.update_pose(n1, n2, ddt)
            self.update_covariance()
            dt += ddt

            # ddt is the interval for which n1 and n2 were recorded.
            # This ddt is used to update velocities. Odom message always gives the last known velocity.
            # dt is the total time of all the messages we clear when we empty the queue.
            # dt is used to publish a wheelvel message with the average wheel vel of the cleared queue.
            # I honestly don't know where wheelvel is being used, but I'm publishing it coz it was there in older code.
            # TODO: If this avg is not the expected behaviour of wheelvel, it needs to be changed.

        if dt != 0:
            self.vr = n1tot / self._counts_per_rev * 60 / dt
            self.vl = n2tot / self._counts_per_rev * 60 / dt
            self.dt = int(dt * 1000)

    def update_pose(self, n1, n2, dt):  # updates x,y,yaw and the jacobians using s1 and s2

        # th is theta, i.e. the angle the wheel rotates by
        th1, th2 = 2 * math.pi * n1 / self._counts_per_rev, 2 * math.pi * n2 / self._counts_per_rev

        # Update pose
        # s is the distance moved by each wheel
        s1, s2 = th1 * self.wheel_radius, th2 * self.wheel_radius

        # Differential drive equations
        ds = (s1 + s2) / 2
        dyaw = (s1 - s2) / (2 * self.wheel_dist)
        cos_yaw = math.cos(self.yaw + (dyaw / 2))
        sin_yaw = math.sin(self.yaw + (dyaw / 2))
        dx = ds * cos_yaw
        dy = ds * sin_yaw

        # updating the velocities
        if dt != 0:
            self.vx = dx / dt
            self.vy = dy / dt
            self.wz = dyaw / dt

        # updating the coordinates
        self.yaw += dyaw
        self.x += dx
        self.y += dy

        s1, s2 = s2, s1
        # updating the values of the matrices
        self.motion_cov = np.array([
            [self.k_right * abs(s1), 0],
            [0, self.k_left * abs(s2)]
        ])

        self.velocity_jacobian = np.array([
            [(cos_yaw / 2) - ((ds / (2 * self.wheel_dist)) * sin_yaw),
             (cos_yaw / 2) + ((ds / (2 * self.wheel_dist)) * sin_yaw)],
            [(sin_yaw / 2) + ((ds / (2 * self.wheel_dist)) * cos_yaw),
             (sin_yaw / 2) - ((ds / (2 * self.wheel_dist)) * cos_yaw)],
            [1 / self.wheel_dist, -1 / self.wheel_dist],
        ])

        self.pose_jacobian = np.array([
            [1, 0, -dy],
            [0, 1, dx],
            [0, 0, 1]
        ])

    def update_covariance(self):  # calculates the covariance
        vel_cov = self.velocity_jacobian.dot(self.motion_cov).dot(np.transpose(self.velocity_jacobian))
        final_cov = self.pose_jacobian.dot(self.covar3by3).dot(np.transpose(self.pose_jacobian))
        self.covar3by3 = final_cov + vel_cov

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
            self.odom.twist.twist.linear.x = self.vx
            self.odom.twist.twist.linear.y = self.vy
            self.odom.twist.twist.angular.z = self.wz

            # Coverting 3x3 to 6x6 (36 entry list) for ros
            roscovar = [0] * 36

            for i in range(2):
                roscovar[i] = self.covar3by3[0, i]
                roscovar[6 + i] = self.covar3by3[1, i]
                roscovar[30 + i] = self.covar3by3[2, i]

            roscovar[5] = self.covar3by3[0, 2]
            roscovar[10] = self.covar3by3[1, 2]
            roscovar[35] = self.covar3by3[2, 2]

            self.odom.pose.covariance = roscovar
            self.odom_pub.publish(self.odom)
            r.sleep()


def main():
    rospy.init_node('encoder_odometry')
    enc_to_odom = OdomOutput()
    rospy.Subscriber("/enc_pulses", enc_pulses, enc_to_odom.enc_callback)
    rospy.loginfo("enc_odom is now alive. Publishing...")
    try:
        enc_to_odom.run()
    except Exception as e:
        print(e)


if __name__ == "__main__":
    main()
