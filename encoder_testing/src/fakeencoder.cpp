#include "ros/ros.h"
#include <std_msgs/Int32.h>
#include <cstdlib>
#include <iostream>
#include <time.h>

int main(int argc, char **argv)
{
  srand(time(0));
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;
  ros::Publisher my_pub = n.advertise<std_msgs::Int32>("chatter", 1000);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    std_msgs::Int32 msg;

    msg.data = rand();
    my_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;
}
