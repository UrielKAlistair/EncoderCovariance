import numpy as np
import math
import rospy
import pandas as pd
from std_msgs.msg import Float64, Int16
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

from datetime import datetime

dateTime = str(datetime.now())
dateTime = (
    dateTime.replace(" ", "").replace(":", "").replace("-", "").replace(".", "")
)
fileName = (
    "/home/uka/Desktop/dump"
    + dateTime
    + ".csv"
)

header_csv = ["v", "delta", "x", "y", "Vx", "Vy", "yaw", "yaw_ins"]

max_steer = np.radians(40.0)  # [rad] max steering angle
L = 1.7  # [m] Wheel base of vehicle
dt = 0.1
Lf = 1.1
Lr = L - Lf  # [m]
Cf = 1600.0 * 2.0  # N/rad
Cr = 1700.0 * 2.0  # N/rad
Iz = 1500.0  # kg/m2 To be calculated
m = 700.0  # kg

data_store = []


# non-linear lateral bicycle model
# class NonLinearBicycleModel:
#     def __init__(self, x=0.0, y=0.0, yaw=0.0, vx=0.01, vy=0, omega=0.0):
#         self.x = x
#         self.y = y
#         self.yaw = yaw
#         self.vx = vx      
#         self.vy = vy
#         self.omega = omega
#         # Aerodynamic and friction coefficients
#         self.c_a = 1.36
#         self.c_r1 = 0.01

#     def update(self, throttle, delta):
#         delta = np.clip(delta, -max_steer, max_steer)
#         self.x = (
#             self.x
#             + self.vx * math.cos(self.yaw) * dt
#             - self.vy * math.sin(self.yaw) * dt
#         )
#         self.y = (
#             self.y
#             + self.vx * math.sin(self.yaw) * dt
#             + self.vy * math.cos(self.yaw) * dt
#         )
#         self.yaw = self.yaw + self.omega * dt
#         self.yaw = normalize_angle(self.yaw)
#         Ffy = -Cf * math.atan2(((self.vy + Lf * self.omega) / self.vx - delta), 1.0)
#         Fry = -Cr * math.atan2((self.vy - Lr * self.omega) / self.vx, 1.0)
#         R_x = self.c_r1 * self.vx
#         F_aero = self.c_a * self.vx**2
#         F_load = F_aero + R_x
#         self.vx = (
#             self.vx
#             + (throttle - Ffy * math.sin(delta) / m - F_load / m + self.vy * self.omega)
#             * dt
#         )
#         self.vy = (
#             self.vy + (Fry / m + Ffy * math.cos(delta) / m - self.vx * self.omega) * dt
#         )
#         self.omega = self.omega + (Ffy * Lf * math.cos(delta) - Fry * Lr) / Iz * dt


# reference: https://www.coursera.org/lecture/intro-self-driving-cars/lesson-2-the-kinematic-bicycle-model-Bi8yE,
# we used the "Rear Alex Bicycle model" as mentioned in that tutorial. TODO: update Read.me
class LinearBicycleModel(object):
    """
    Class representing the state of a vehicle.
    :param x: (float) x-coordinate
    :param y: (float) y-coordinate
    :param yaw: (float) yaw angle
    :param v: (float) speed
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, actuatorAngle=0):
        """Instantiate the object."""
        self.x = x
        self.y = y
        self.yaw = yaw
        self.yaw_ins = yaw
        self.v = v
        self.Vx = 0
        self.Vy = 0
        self.actuatorAngle = 0
        self.delta = 0.0
        self.stepper_Subscriber = rospy.Subscriber(
            "stepperMotor_Topic", Int16, self.getStepper_data
        )
        self.velocity_Subscriber = rospy.Subscriber(
            "rpm_topic", Float64, self.getCurrentVelocity_data
        )
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        self.r = rospy.Rate(10.0)

        """ data_pd = pd.DataFrame([data_store], columns=header_csv)
        data_pd.to_csv(fileName, mode="a", index=False, header=True) """

    def getStepper_data(self, data):
        self.actuatorAngle = data.data
        self.delta = self.actAngleToStAngle(self.actuatorAngle)
        print(data.data)

    def getCurrentVelocity_data(self, data):
        # rpm=0.0
        # rpm=data.data
        self.v = data.data
        self.update(self.v, np.radians(self.delta))

    def actAngleToStAngle(self, actuatorAngle):
        # to be better estimated
        return actuatorAngle * 0.04645905209

    def update(self, throttle, delta):
        """
        Update the state of the vehicle.
        Stanley Control uses bicycle model.
        :param a: (float) Acceleration
        :param delta: (float) Steering
        """

        self.current_time = rospy.Time.now()
        delta = np.clip(delta, -max_steer, max_steer)

        self.x += self.v * np.cos(self.yaw) * dt
        self.y += self.v * np.sin(self.yaw) * dt
        self.Vx = self.v * np.cos(self.yaw)
        self.Vy = self.v * np.sin(self.yaw)
        self.yaw_ins = self.v / L * np.tan(delta)
        self.yaw += self.v / L * np.tan(delta) * dt
        self.yaw_ins = normalize_angle(self.yaw_ins)
        print(self.Vx, self.Vy, self.yaw)

        # since all odometry is 6DOF we'll need a quaternion created from yaw
        self.odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.yaw)

        # first, we'll publish the transform over tf
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0.0),
            self.odom_quat,
            self.current_time,
            "base_link",
            "odom",
        )
        # self.v += throttle * dt

        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = self.current_time
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(
            Point(self.x, self.y, 0.0), Quaternion(*self.odom_quat)
        )

        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(
            Vector3(self.Vx, self.Vy, 0), Vector3(0, 0, self.yaw_ins)
        )

        # publish the message
        self.odom_pub.publish(odom)

        data_store = [
            self.v,
            self.delta,
            self.x,
            self.y,
            self.Vx,
            self.Vy,
            self.yaw,
            self.yaw_ins,
        ]
        data_pd = pd.DataFrame([data_store], columns=header_csv)
        data_pd.to_csv(fileName, mode="a", index=False, header=False)


def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi].
    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle


if __name__ == "__main__":
    rospy.init_node("bicycle_model", anonymous=True)
    bicycle = LinearBicycleModel()
    rospy.spin()
