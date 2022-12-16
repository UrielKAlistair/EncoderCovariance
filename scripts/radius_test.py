#!/usr/bin/python3

from vajra_controller_tiva.msg import enc_pulses
import rospy
import matplotlib.pyplot as plt


def cum_sum(arr):
    for i in range(1, len(arr)):
        arr[i] += arr[i - 1]
    return arr


enc_array_l = []
enc_array_r = []


def callback(data):
    global enc_array
    enc_array_l.append(data.enc1)
    enc_array_r.append(data.enc0)


def main():
    rospy.init_node('radius_test')
    rospy.Subscriber("/enc_pulses", enc_pulses, callback)
    print("I am now alive")
    while not rospy.is_shutdown():
        rospy.spin()
    print("DED")
    print("Right:\n", enc_array_r)
    print("Left:\n", enc_array_l)

    csl = cum_sum(enc_array_l)
    csr = cum_sum(enc_array_r)
    print("\n FINAL VALUES \n", csl[-1], csr[-1], "\n")
    plt.plot(csl)
    plt.plot(csr)
    plt.show()


if __name__ == "__main__":
    main()
