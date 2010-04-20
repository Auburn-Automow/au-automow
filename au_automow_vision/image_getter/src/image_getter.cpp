#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/CvBridge.h>
#include <cv.h>
#include <highgui.h>

using namespace cv;

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_getter");
    ros::NodeHandle n;
    ros::Publisher img_pub = n.advertise<sensor_msgs::Image>("image", 1);
    ros::Rate loop_rate(5);
    
    VideoCapture cap(0);
    if(!cap.isOpened()) {
        ROS_WARN("No camera detected");
        return -1;
    }
    Mat frame;
    IplImage* img;
    
    while(ros::ok()) {
        cap >> frame;
        img = new IplImage(frame);
        IplImage* out = cvCreateImage( cvSize(img->width/2,img->height/2), img->depth, img->nChannels );
        cvPyrDown( img, out );
        sensor_msgs::Image::Ptr ros_img = sensor_msgs::CvBridge::cvToImgMsg(out);
        sensor_msgs::Image msg(*ros_img);
        
        img_pub.publish(msg);
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}

