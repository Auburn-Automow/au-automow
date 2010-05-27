//////////////////////// > There are alot of images that need to be double checked that they are garbage collected.
//////////////////////// <
#include <ros/param.h>
#include <string>
#include <iostream>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <cv_bridge/CvBridge.h>
#include <cstdio>
#include <cstdlib>
#include <boost/thread.hpp>
#include <highgui.h>
#include <cv.h>
#include <list>

#define PIX_2_M 0.00377

// Uncomment this line to have the lines drawn on the original image and transmitted
#define __TX_POINT_CLOUD
#define __TX_PROCESSED_IMAGE
#define __TX_DEBUG_IMAGE
#define __INVERT_GRAYSCALE

sensor_msgs::CvBridge bridge_;

// Load the bird's eye view conversion matrix 
CvMat *birdeye_mat;

IplImage *img_template, *img_1chan, *img_thresh, *temp, *templ, *captured_img_bird;

#ifdef __TX_POINT_CLOUD
ros::Publisher point_cloud_publisher;
#endif

#ifdef __TX_PROCESSED_IMAGE
ros::Publisher processed_image_publisher;
#endif

#ifdef __TX_DEBUG_IMAGE
ros::Publisher temp_match_image_publisher;
ros::Publisher chan_image_publisher;
#endif

ros::NodeHandle *n;
int thresh_subtract;
double percent_coverage;
short sequence_count;
sensor_msgs::PointCloud::_points_type g_points;
boost::mutex write_mux;

struct make_point_cloud {
  const uchar* ptr;
  int start;
  int end;
  sensor_msgs::PointCloud::_points_type points;
  make_point_cloud(const uchar* ptr, int start, int end) : ptr(ptr), start(start), end(end) { }
  void operator() () {
      for(int m = start; m < end; m++) {
        if(ptr[m] != 0) {
          uchar col_raw = m % img_thresh->widthStep;
          uchar row_raw = m / img_thresh->widthStep;
          
          uchar col = col_raw - 193;
          uchar row = img_thresh->height - row_raw;
          
          geometry_msgs::Point32 found_point;
          found_point.x = col * PIX_2_M;
          found_point.y = row * PIX_2_M;
          found_point.z = 0;
          points.push_back(found_point);
        }
      }
      boost::mutex::scoped_lock lock(write_mux);
      while (!points.empty()) {
        g_points.push_back(points.back());
        points.pop_back();
      }
  }
};

inline void blackoutImage(IplImage *img) {
    for (int n = img->height; n != 0; --n) {
        for (int k = img->width; k != 0; --k) {
            uchar* val = (uchar*)img->imageData + (n * img->widthStep) + k;
            *val = 0;
        }
    }
}

/* countPixels() counts the number of pixels in an image that equal a given value
 */
long long countPixels(IplImage *img, int value, int target = 0) {
    long long count = 0;
    for (int n = img->height; n != 0; --n) {
      uchar *ptr = (uchar*)(img->imageData + n * img->widthStep);
      for (int k = img->width; k != 0; --k) {
        if (ptr[k] == value)
          count++;
        if (count > target) 
          return count;
      }
    }
    return count;
};

uchar maxPixelValue(IplImage *img) {
  uchar max = 0;
  for (int n = img->height; n != 0; --n) { 
    for (int k = img->width; k != 0; --k) {
        uchar pixel_value = *((uchar*)img->imageData + (n * img->widthStep) + k);
        if (max < pixel_value)
            max = pixel_value;
    }
  }
  
  return max;
};

void findLinesInImage(IplImage *img) {
    int maxPixVal;
    
    // plain grass detection
    long long int count = 0;
    int tooManyPoints = 0;
    
    // values for determining coordinates for point cloud
    int max = 0; //img_small->widthStep * img_small->height;
    uchar *ptr;      // used to iterate through the small image

    /////////////////////////////////////// Begin Line Detection /////////////////////////////////////////

    cvMatchTemplate(img, templ, temp, CV_TM_CCORR);
    cvNormalize(temp, temp, 1, 0, CV_MINMAX);
    
    cvCvtScale(temp, img_1chan, 255);
    
    maxPixVal = maxPixelValue(img_1chan);
    
#ifdef __TX_DEBUG_IMAGE
{
    sensor_msgs::Image::Ptr processed_ros_img = sensor_msgs::CvBridge::cvToImgMsg(img_1chan);
    chan_image_publisher.publish(*processed_ros_img);
}
#endif

    cvThreshold(img_1chan, img_thresh, maxPixVal - thresh_subtract, 255, CV_THRESH_BINARY);
    
    count = countPixels(img_thresh, 255, (img_thresh->height * img_thresh->width) * percent_coverage);
   
    if(count == (img_thresh->height * img_thresh->width) * percent_coverage)
        tooManyPoints = 1;
    
#ifdef __TX_DEBUG_IMAGE
{
    sensor_msgs::Image::Ptr processed_ros_img = sensor_msgs::CvBridge::cvToImgMsg(img_thresh);
    temp_match_image_publisher.publish(*processed_ros_img);
}
#endif
    
    ptr = (uchar*)img_thresh->imageData;
    
    max = img_thresh->widthStep * img_thresh->height;
    
    sensor_msgs::PointCloud point_cloud;
    
    point_cloud.header.frame_id = "camera_frame";
    point_cloud.header.stamp = ros::Time::now();
    
    if (!tooManyPoints) {
        make_point_cloud x(ptr, 0, max);
        int one_fifth = max/5;
        boost::thread calc1(make_point_cloud(ptr, 0, one_fifth));
        boost::thread calc2(make_point_cloud(ptr, one_fifth+1, one_fifth*2));
        boost::thread calc3(make_point_cloud(ptr, (one_fifth * 2) + 1, one_fifth * 3));
        boost::thread calc4(make_point_cloud(ptr, (one_fifth * 3) + 1, one_fifth * 4));
        boost::thread calc5(make_point_cloud(ptr, (one_fifth * 4) + 1, max));
        calc5.join();
        calc4.join();
        calc3.join();
        calc2.join();
        calc1.join();

        point_cloud.points = g_points;
    }
    
    point_cloud_publisher.publish(point_cloud);
};

void imageReceived(const sensor_msgs::ImageConstPtr& ros_img) {
    g_points.clear();
    sequence_count++;
    if (sequence_count == 100) {
        ros::param::get("/line_processing/thresh_subtract", thresh_subtract);
        ros::param::get("/line_processing/percent_coverage", percent_coverage);
        sequence_count = 0;
    }
    
    // Convert the image received into an IPLimage
    IplImage *captured_img = bridge_.imgMsgToCv(ros_img);
    
   
    /*
    std::string path;
    char dir[FILENAME_MAX];
    
    if (getcwd(dir, sizeof(dir))) { 
      path = std::string(dir) + "/frame0004.jpg";
    }
    
    captured_img = cvLoadImage(path.c_str(), CV_LOAD_IMAGE_UNCHANGED);
    */ 

    cvWarpPerspective(captured_img, captured_img_bird, birdeye_mat,
                      CV_INTER_LINEAR | CV_WARP_INVERSE_MAP | CV_WARP_FILL_OUTLIERS); 
    
    //cvResize(captured_img_bird, , NULL);
//////////////////////// > Can this line be removed and change the findLinesInImage parameter to captured_img_bird? seems like an unneccsary copy
    // // Copy the bird's eye image back to the original 
    // cvCopy(captured_img_bird, captured_img, NULL);
    // 
    // // Detect lines in the captured image and store the resulting lines
    // findLinesInImage(captured_img);
    findLinesInImage(captured_img_bird);
//////////////////////// <
    
#ifdef __TX_PROCESSED_IMAGE
    // Convert processed image into ros_img_msg
//////////////////////// > We need to make sure that these variables are recycled properly
    sensor_msgs::Image::Ptr processed_ros_img = sensor_msgs::CvBridge::cvToImgMsg(captured_img_bird);
    // sensor_msgs::Image msg(*processed_ros_img);
//////////////////////// <
    // Publish image to topic /processed_image
    processed_image_publisher.publish(*processed_ros_img);
#endif
}

int main(int argc, char** argv) {
    // Initialize the node
    ros::init(argc, argv, "line_processing");
    n = new ros::NodeHandle;
    
    ros::param::get("/line_processing/thresh_subtract", thresh_subtract);
    ros::param::get("/line_processing/percent_coverage", percent_coverage);
    sequence_count = 0;
    
    // load white grass image template
    char dir[FILENAME_MAX]; 
    std::string path;
    
    if (getcwd(dir, sizeof(dir))) { 
      path = std::string(dir) + "/birdeye_convert_mat.xml";
    }
    
    // Load the bird's eye view conversion matrix 
    birdeye_mat = (CvMat*)cvLoad(path.c_str());
    if (birdeye_mat == NULL) { 
        ROS_ERROR("Birds eye matrix not loaded properly. Set the birds_eye param");
        return -1;
    }
    // Register the node handle with the image transport
    image_transport::ImageTransport it(*n);
    // Set the image buffer to 1 so that we process the latest image always
    image_transport::Subscriber sub = it.subscribe("/image_raw", 1, imageReceived);
    point_cloud_publisher = n->advertise<sensor_msgs::PointCloud>("image_point_cloud", 5);
#ifdef __TX_PROCESSED_IMAGE
    processed_image_publisher = n->advertise<sensor_msgs::Image>("processed_image", 1);
#endif
#ifdef __TX_DEBUG_IMAGE
    temp_match_image_publisher = n->advertise<sensor_msgs::Image>("temp_match_image", 1);
    chan_image_publisher = n->advertise<sensor_msgs::Image>("chan_image", 1);
#endif
   
    if (getcwd(dir, sizeof(dir))) { 
      path = std::string(dir) + "/template_jpg.jpg";
    }
    
    templ = cvLoadImage(path.c_str(), CV_LOAD_IMAGE_UNCHANGED);
    // Create image for the template matching output
    img_template = cvCreateImage(cvSize(320 - templ->width + 1, 240 - templ->height + 1), IPL_DEPTH_8U, 3);
    
    // image pointers
    img_1chan = cvCreateImage(cvGetSize(img_template), IPL_DEPTH_8U, 1);
    img_thresh = cvCreateImage(cvGetSize(img_template), IPL_DEPTH_8U, 1);
    temp = cvCreateImage(cvGetSize(img_template), IPL_DEPTH_32F, 1);
    // create bird's eye view
    captured_img_bird = cvCreateImage(cvSize(320, 240), IPL_DEPTH_8U, 3);
    
    sequence_count = 0;
    
    // Run until killed
    ros::spin();
    // Clean exit
    return 0;
}

