from time import sleep
from turtle import delay
import rospy
from std_msgs.msg import Int16, Float64
rospy.init_node("bicycle_model_tester", anonymous=True)
step_pub = rospy.Publisher("stepperMotor_Topic", Int16, queue_size=5)
rpm_pub = rospy.Publisher("rpm_topic", Float64, queue_size=5)

step = 100
rpm = 10

while True:
    step_pub.publish(step)
    rpm_pub.publish(rpm)
    sleep(0.1)  
