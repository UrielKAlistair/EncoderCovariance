from time import sleep, time
import rospy
from std_msgs.msg import String

rospy.init_node("fake_encoders", anonymous=True)
left_pub = rospy.Publisher("fake_encoder_left", String, queue_size=5)

left_count = 10000
right_count = 9000

told = time()

while True:
    left_pub.publish(f"{time()-told} {left_count} {right_count}")
    told = time()
    sleep(0.1)
