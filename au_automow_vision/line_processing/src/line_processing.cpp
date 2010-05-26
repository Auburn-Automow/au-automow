
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

#define MAX_TRENDS 3
#define MAX_ANGLE_SEP 0.1745 // 0.1745 rad = 10 degrees

#define SORT_BY_RHO 0
#define SORT_BY_THETA 1

#define RESIZE_FACTOR 2 // relies on pyrdown() in image_getter()

#define WIN_DENSITY_HEIGHT (40 / RESIZE_FACTOR)
#define WIN_DENSITY_WIDTH (40 / RESIZE_FACTOR)

#define CV_PI_2 (CV_PI / 2)
#define CV_2PI (2 * CV_PI)

#define MAX_SCANS 100

// Uncomment this line to have the lines drawn on the original image and transmitted
#define __TX_POINT_CLOUD
#define __TX_PROCESSED_IMAGE
#define __TX_DEBUG_IMAGE
#define __INVERT_GRAYSCALE

sensor_msgs::CvBridge bridge_;

// Load the bird's eye view conversion matrix 
CvMat *birdeye_mat;

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

int findMaxRho(IplImage *img) {
    return sqrt(pow(img->height,2) + pow(img->width,2));
};

void invertGrayscale(IplImage *img, int max_val) {
	uchar *data = (uchar *)img->imageData;
	
	for(int i=0;i<img->height;i++) for(int j=0;j<img->width;j++) for(int k=0;k<img->nChannels;k++)
		data[i*img->widthStep+j*img->nChannels+k]=max_val-data[i*img->widthStep+j*img->nChannels+k];
}

void findLRFpoints(IplImage *img, IplImage *original, double mags[]) {
	CvPoint origin;
	origin.x = 252;
	origin.y = img->height - 1;
	
	int hold;
	int col = 0, row = 0;
	
	int x_off = -(MAX_SCANS - 1) / 2;
	int y_off = 0;
	int x = x_off, y = y_off;
	
	uchar *ptr_origin = (uchar*)(img->imageData + origin.x + (origin.y * img->widthStep));
	uchar *ptr = ptr_origin;
	
	for(int n=0; n < MAX_SCANS; n++) {
		
		while((ptr - (uchar*)img->imageData) % img->widthStep != 0 &&	// left end column
			  (ptr - (uchar*)img->imageData) % img->widthStep != (img->widthStep - 1) &&	// right end column
			  !(ptr >= (uchar*)img->imageData && ptr < (uchar*)(img->imageData + img->widthStep))) { // top row
			
			if (y != 0) {	// -Y shift
				ptr -= img->widthStep;
				y++;
				//printf("Y SHIFT\n");
			}
			else if (x != 0) {	// + X shift
				if (x > 0) {
					ptr++;
					x--;
					//printf("+X SHIFT\n");
				}
				else if (x < 0) {	// -X shift
					ptr--;
					x++;
					//printf("-X SHIFT\n");
				}
			}
			else if (x == 0 && y == 0) {
				x = x_off;
				y = y_off;
			}
			
			// Check if pixel is part of a line
			if (ptr[0] != 0) {
				hold = ptr - (uchar*)img->imageData;
				
				col = hold % img->widthStep;
				row = hold / img->widthStep;
				
				mags[n] = sqrt( pow( (origin.y - row), 2 ) + pow( (origin.x - col), 2 ) );
				
				printf("col: %3d\trow: %3d\tmag[%d]: %f\n", col, row, n, mags[n]);
				
				//display found points on original image
				cvCircle(original, cvPoint(col, row), 2, cvScalar(0,0,255,0), 3, 8, 0);
				
				break;
			}
		}
		
		// Iterate offsets for next line
		x_off++;
		
		if (x_off < 1)
			y_off--;
		else
			y_off++;
		
		ptr = ptr_origin;
	}
};

void blackoutImage(IplImage *img) {
	for(int n=0; n < img->height; n++) {
		uchar *ptr = (uchar*)(img->imageData + n * img->widthStep);
		
		for(int k=0; k < img->width; k++) {
			ptr[k] = 0;
		}
	}
};

/* countPixels() counts the number of pixels in an image that equal a given value
 */
int countPixels(IplImage *img, int value) {
	int count = 0;
	
	for(int n=0; n < img->height; n++) {
		uchar *ptr = (uchar*)(img->imageData + n * img->widthStep);
		
		for(int k=0; k < img->width; k++) {
			if(ptr[k] == value)
				count++;
		}
	}

	return count;
};

void correctNegativeRho(CvSeq *lines) {
    float line[2];

    for(int n=0; n < lines->total; n++) {
        cvSeqPopFront(lines, &line);

        if(line[0] < 0)
        {
            line[0] = -line[0];
            line[1] = line[1] - CV_PI;
        }

        cvSeqPush(lines, line);
    }
};

void sortLines(CvSeq *unsorted, int sort_type) {
    if(sort_type != SORT_BY_RHO && sort_type != SORT_BY_THETA)
        printf("sort type error!\n");

    //cvSeqSort(unsorted, cmpSort, &sort_type);
};

int maxPixelValue(IplImage *img) {
    int max = 0;
    for(int n=0; n < img->height; n++)
    {
        uchar *ptr = (uchar *)(img->imageData + n * img->widthStep);
        
        for(int k=0; k < img->width; k++)
        {
            if(max < ptr[k])
                max = ptr[k];
        }
    }
    
    return max;
};

/*
int binaryPixelDensityFinder(IplImage *img, int window_height, int window_width, struct window_density window_densities[]) {
    int max_density = 0;
    int density = 0;
    int index = 0;
    
    for(int n=0; n < img->height; n+=window_height) { // shift down rows to next row-window
        uchar *ptr1 = (uchar*)(img->imageData + n*img->widthStep);
        
        for(int m=0; m < img->width; m+=window_width) { // shift along columns to next window
            uchar *ptr2 = ptr1 + m;

            for(int k=0; k < window_height; k++) { // shift down by single rows within window
                uchar *ptr3 = ptr2 + k*img->widthStep;

                for(int w=0; w < window_width; w++) // sum along columns in row
                        density+= *(ptr3+w);
            }
            
            if(density > max_density)
                max_density = density;
            
            window_densities[index].index = ptr2;
            window_densities[index++].density = density;

            density = 0;
        }
    }

    return max_density;
};


void DensityFilter(struct window_density densities[], int max_density, int window_height, int window_width, int widthStep, int max_windows) {
    for(int n=0; n < max_windows; n++) {
		  #ifndef __INVERT_GRAYSCALE
        if(densities[n].density < max_density*0.75) {
		  #endif
		  #ifdef __INVERT_GRAYSCALE
		  if(densities[n].density < max_density*0.2) {
		  #endif
            for(int k=0; k < window_height; k++) {
                uchar *ptr = densities[n].index + k*widthStep;
                
                for(int m=0; m < window_width; m++)
                    *(ptr+m) = 0;
            }
        }
    }
};
*/
void averageToTrends(CvSeq *group, CvSeq *trend_lines) {
    float rho_sum = 0;
    float theta_sum = 0;
    float bundle[2];
    
    for(int line_num=0; line_num < group->total; line_num++) {
        float *line = (float*)cvGetSeqElem(group, line_num);
        rho_sum += line[0];
        theta_sum += line[1];
    }
    
    bundle[0] = rho_sum/group->total;
    bundle[1] = theta_sum/group->total;
    cvSeqPush(trend_lines, bundle);
};

void discoverTrendLines(CvSeq *lines_to_be_averaged, CvSeq *trend_lines) {
    CvMemStorage *storage_theta_group = cvCreateMemStorage(0);
    CvSeq *theta_group = cvCreateSeq(0, sizeof(CvSeq), 2*sizeof(float), storage_theta_group);
    
    int initial_line = 1;
    float theta_hold;
    
    sortLines(lines_to_be_averaged, SORT_BY_THETA);
    
    for(int line_num=0; line_num < lines_to_be_averaged->total; line_num++) {
        float *line = (float*)cvGetSeqElem(lines_to_be_averaged, line_num);
        float theta = line[1];
        
        if(initial_line) {
            theta_hold = theta;
            cvSeqPush(theta_group, line);
            initial_line = 0;
        }
        else {
            if(theta > theta_hold + MAX_ANGLE_SEP || line_num+1 == lines_to_be_averaged->total) {
                averageToTrends(theta_group, trend_lines);
                cvClearSeq(theta_group);
                initial_line = 1;
            }
            else {
                theta_hold = theta;
                cvSeqPush(theta_group, line);
            }
        }
    }

    cvReleaseMemStorage(&storage_theta_group);
};

void findLinesInImage(IplImage *img, double mags[]) {
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
      // Create image for the template matching output
      IplImage *img_template = cvCreateImage(cvSize(img->width - templ->width + 1, img->height - templ->height + 1), IPL_DEPTH_8U, 3);
      
      // image pointers
      IplImage *img_1chan = cvCreateImage(cvGetSize(img_template), IPL_DEPTH_8U, 1);
      IplImage *img_thresh = cvCreateImage(cvGetSize(img_template), IPL_DEPTH_8U, 1);
      IplImage *temp = cvCreateImage(cvGetSize(img_template), IPL_DEPTH_32F, 1);

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
  }
  else {  
    ROS_INFO("error loading template.jpg");
  }
};

void findTrendLines(CvSeq *found_lines, CvSeq *trend_lines, int max_rho) {
    CvMemStorage *storage_rho_group = cvCreateMemStorage(0);
    
    CvSeq *rho_group = cvCreateSeq(0, sizeof(CvSeq), 2*sizeof(float), storage_rho_group);
    
    // trend variables
    float rho_hold;
    int initial_line = 1;
            
    /////////////////////////////////////// Begin Trend Line Discovery /////////////////////////////////////////
    correctNegativeRho(found_lines);
    sortLines(found_lines, SORT_BY_RHO);
                    
    for(int line_num=0; line_num < found_lines->total; line_num++)
    {
        float *line = (float*)cvGetSeqElem(found_lines, line_num);
        float rho = line[0];
                
        if(initial_line) {
            rho_hold = rho;
            cvSeqPush(rho_group, line);
            initial_line = 0;
        }
        else {
            float diff = rho - rho_hold;

            if(diff < max_rho/2.0)
                cvSeqPush(rho_group, line);
            
            if(diff >= max_rho/2.0 || line_num+1 == found_lines->total) {
                discoverTrendLines(rho_group, trend_lines);

                cvClearSeq(rho_group);
                initial_line = 1;
            }
        }   
    }

    cvReleaseMemStorage(&storage_rho_group);
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
	
    // holding array for "LRF" line detection magnitudes
    double mags[MAX_SCANS] = {0};

    cvWarpPerspective(captured_img, captured_img_bird, birdeye_mat,
                      CV_INTER_LINEAR | CV_WARP_INVERSE_MAP | CV_WARP_FILL_OUTLIERS); 
    
    // Copy the bird's eye image back to the original 
    cvCopy(captured_img_bird, captured_img, NULL);
    
    // Detect lines in the captured image and store the resulting lines
    findLinesInImage(captured_img, mags);
    
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
#ifdef __TX_POINT_CLOUD
    point_cloud_publisher = n->advertise<sensor_msgs::PointCloud>("image_point_cloud", 5);
#endif
#ifdef __TX_PROCESSED_IMAGE
    processed_image_publisher = n->advertise<sensor_msgs::Image>("processed_image", 1);
#endif
#ifdef __TX_DEBUG_IMAGE
    temp_match_image_publisher = n->advertise<sensor_msgs::Image>("temp_match_image", 1);
    chan_image_publisher = n->advertise<sensor_msgs::Image>("chan_image", 1);
#endif
    // Run until killed
    ros::spin();
    // Clean exit
    return 0;
}

