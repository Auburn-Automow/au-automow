#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/CvBridge.h>
#include <cv.h>
#include <highgui.h>

using namespace cv;

int main(int argc, char** argv) {
  ros::init(argc, argv, "image_getter");
  ros::NodeHandle n;
  ros::Publisher img_pub = n.advertise<sensor_msgs::Image>("image", 10);
  ros::Rate loop_rate(5);

  VideoCapture cap(0);
  if(!cap.isOpened()) return -1;
  Mat frame;
  IplImage* img; 

  while(ros::ok()) {
    std::cout << "i" << std::endl;
    cap >> frame;
    img = new IplImage(frame);
    sensor_msgs::Image::Ptr ros_img = sensor_msgs::CvBridge::cvToImgMsg(img);
    sensor_msgs::Image msg(*ros_img);

    img_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

