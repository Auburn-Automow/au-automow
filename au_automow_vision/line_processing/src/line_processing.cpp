
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
#include <highgui.h>
#include <cv.h>

#define PIX_2_M 0.00377

// Uncomment this line to have the lines drawn on the original image and transmitted
#define __TX_POINT_CLOUD
//#define __TX_PROCESSED_IMAGE
//#define __TX_DEBUG_IMAGE
#define __INVERT_GRAYSCALE

sensor_msgs::CvBridge bridge_;

// Load the bird's eye view conversion matrix 
CvMat *birdeye_mat;

IplImage *img_template, *img_1chan, *img_thresh, *temp;

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
uchar countPixels(IplImage *img, int value, int target = 0) {
	uchar count = 0;
	for (int n = img->height; n != 0; --n) {
		for (int k = img->width; k != 0; --k) {
			if (*((uchar*)img->imageData + (n * img->widthStep) + k) == value)
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
        if (max < *((uchar*)img->imageData + (n * img->widthStep) + k))
          max = *((uchar*)img->imageData + (n * img->widthStep) + k);
    }
  }
  
  return max;
};

void findLinesInImage(IplImage *img) {
    int maxPixVal;

    // plain grass detection
    int count = 0;
    int tooManyPoints = 0;

    // load white grass image template
    char dir[FILENAME_MAX]; 
    std::string path;

    if (getcwd(dir, sizeof(dir))) { 
      path = std::string(dir) + "/template_jpg.jpg";
    }

    IplImage *templ = cvLoadImage(path.c_str(), CV_LOAD_IMAGE_UNCHANGED);

    if (templ) {

      // values for determining coordinates for point cloud
      int max = 0; //img_small->widthStep * img_small->height;
      int row_raw=0, col_raw=0;
      int row=0, col=0;
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
      
      count = countPixels(img_thresh, 255);

      if(count >= (img_thresh->height * img_thresh->width) * percent_coverage)
         tooManyPoints = 1;

      if(tooManyPoints)
         blackoutImage(img_thresh);

#ifdef __TX_DEBUG_IMAGE
      {
        sensor_msgs::Image::Ptr processed_ros_img = sensor_msgs::CvBridge::cvToImgMsg(img_thresh);
        temp_match_image_publisher.publish(*processed_ros_img);
      }
#endif
      
      ptr = (uchar*)img_thresh->imageData;

      max = img_thresh->widthStep * img_thresh->height;

      sensor_msgs::PointCloud point_cloud;

      for(int m=0; m < max; m++) {
         if(ptr[m] != 0) {
          col_raw = m % img_thresh->widthStep;
          row_raw = m / img_thresh->widthStep;
        
          col = col_raw - 193;
          row = img_thresh->height - row_raw;
          
          geometry_msgs::Point32 found_point;
          found_point.x = col * PIX_2_M;
          found_point.y = row * PIX_2_M;
          found_point.z = 0;
          point_cloud.points.push_back(found_point);
          // place coordinates here: x = col -- y = row -- z = 0
        }
      }
     
      point_cloud_publisher.publish(point_cloud);

    //findLRFpoints(img_thresh, img, mags);
    //delete templ;
  }
  else {  
    ROS_INFO("error loading template.jpg");
  }
};

void imageReceived(const sensor_msgs::ImageConstPtr& ros_img) {
    ros::param::get("/line_processing/thresh_subtract", thresh_subtract);
    ros::param::get("/line_processing/percent_coverage", percent_coverage);

    // Convert the image received into an IPLimage
    IplImage *captured_img;
    captured_img = bridge_.imgMsgToCv(ros_img);
    
    // create bird's eye view
    IplImage *captured_img_bird;
    captured_img_bird = cvCreateImage(cvGetSize(captured_img), IPL_DEPTH_8U, 3);
	
    cvWarpPerspective(captured_img, captured_img_bird, birdeye_mat,
                      CV_INTER_LINEAR | CV_WARP_INVERSE_MAP | CV_WARP_FILL_OUTLIERS); 
    
    // Copy the bird's eye image back to the original 
    cvCopy(captured_img_bird, captured_img, NULL);
    
    // Detect lines in the captured image and store the resulting lines
    findLinesInImage(captured_img);
    
#ifdef __TX_PROCESSED_IMAGE
    // Convert processed image into ros_img_msg
    sensor_msgs::Image::Ptr processed_ros_img = sensor_msgs::CvBridge::cvToImgMsg(captured_img_bird);
    sensor_msgs::Image msg(*processed_ros_img);
    // Publish image to topic /processed_image
    processed_image_publisher.publish(msg);
#endif
    
    // Cleanup
    // cvReleaseMemStorage(&line_storage);
}

int main(int argc, char** argv) {
    // Initialize the node
    ros::init(argc, argv, "line_processing");
    n = new ros::NodeHandle;

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
    
    IplImage *templ = cvLoadImage(path.c_str(), CV_LOAD_IMAGE_UNCHANGED);
    // Create image for the template matching output
    img_template = cvCreateImage(cvSize(640 - templ->width + 1, 480 - templ->height + 1), IPL_DEPTH_8U, 3);
    
    // image pointers
    img_1chan = cvCreateImage(cvGetSize(img_template), IPL_DEPTH_8U, 1);
    img_thresh = cvCreateImage(cvGetSize(img_template), IPL_DEPTH_8U, 1);
    temp = cvCreateImage(cvGetSize(img_template), IPL_DEPTH_32F, 1);
    
    // Run until killed
    ros::spin();
    // Clean exit
    return 0;
}

