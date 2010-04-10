#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>

void listenCallback(const std_msgs::StringConstPtr& msg) {
  ROS_INFO("Recieved"); 
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("image", 10, listenCallback);
  ros::spin();
  return 0;
}

